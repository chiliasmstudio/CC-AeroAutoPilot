--[[
    Create: Avionics & CC: Tweaked
    Unified Quad-Engine Avionics Flight Computer (四軸有線模組化統一飛控大腦)
    Version: v3.6.0
    
    架構說明 (Decoupled PHP+Vue Architecture):
    - 飛控核心與動力後端 (Flight Core - Backend): 獨立封裝 PID、感測器獲取、PWM 集體升力混控、200m 標定演算法與烏龜通訊。
    - 顯示驅動與視圖前端 (Display Drivers - Frontend Views): 獨立分流，支援 DirectGPU、Tom's Peripherals 與 原生 Monitor/Terminal。
    - 全螢幕二級視圖選單 (Full-Screen View Selector Menu):
        * 右上角常駐 [ ☰ MENU ] / [ ✕ CLOSE ] 按鈕。
        * 點擊後全螢幕展開 5 大獨立視圖切換：
          1. OVERVIEW (全功能總覽儀表)
          2. PFD (主飛行儀表：大姿態 + 大高度)
          3. ECAM (發動機動力監控：4 具大圓錶 + 200m 校正專區)
          4. NAV (快速導航與預設高度：0m/80m/150m/200m/300m/鎖定)
          5. SYS (系統診斷與硬體自檢：4 軸烏龜即時狀態)
    - 內建向量點陣字體引擎 (5x7 Pixel Font Engine): 保證在任何版本 Tom's GPU 均能 100% 完美顯現文字與數值。
    - 單檔整合發布 (Single-File Distribution): 下載此檔案即可支援所有硬體，支援自動探測或命令列指定驅動。

    使用方式:
      run.lua              (自動檢測最佳顯示硬體: DirectGPU -> Tom's GPU -> Monitor -> Terminal)
      run.lua tom          (強制使用 Tom's Peripherals GPU 驅動)
      run.lua directgpu    (強制使用 CC-DirectGPU-Mod 驅動)
      run.lua normal       (強制使用 CC: Tweaked 原生螢幕/終端機驅動)
--]]

local VERSION = "v3.6.0"
local args = {...}
local requestedDriver = args[1] and string.lower(args[1]) or "auto"

-- ========================================================
-- PART 1: 飛控與動力控制核心 (FLIGHT & PROPULSION CORE - BACKEND)
-- ========================================================
local FlightCore = {}

-- 1.1 內建 PID 控制器
local PID = {}
PID.__index = PID

function PID.new(kp, ki, kd, minOutput, maxOutput)
    local self = setmetatable({}, PID)
    self.kp = kp or 1.0
    self.ki = ki or 0.0
    self.kd = kd or 0.0
    self.minOutput = minOutput or -1.0
    self.maxOutput = maxOutput or 1.0
    self.integral = 0
    self.prevError = 0
    self.lastTime = os.epoch("utc") / 1000
    return self
end

function PID:reset()
    self.integral = 0
    self.prevError = 0
    self.lastTime = os.epoch("utc") / 1000
end

function PID:update(error)
    local now = os.epoch("utc") / 1000
    local dt = now - self.lastTime
    if dt <= 0 then dt = 0.05 end
    self.lastTime = now

    local p = self.kp * error
    self.integral = self.integral + error * dt
    local iMax = math.abs(self.maxOutput) * 0.5
    if self.integral > iMax then self.integral = iMax end
    if self.integral < -iMax then self.integral = -iMax end
    local i = self.ki * self.integral

    local derivative = (error - self.prevError) / dt
    self.prevError = error
    local d = self.kd * derivative

    local output = p + i + d
    if output > self.maxOutput then output = self.maxOutput end
    if output < self.minOutput then output = self.minOutput end
    return output
end

-- 1.2 飛控狀態
FlightCore.state = {
    mode = "IDLE",            -- "IDLE", "HOLD_ALT", "CALIBRATING"
    targetAlt = 200.0,        -- 預設目標高度 200m
    virtualAlt = 100.0,       -- 虛擬平滑軌跡高度
    baseThrottle = 1.0,       -- 浮點基準推力 (0.01 ~ 15.0)
    statusMsg = "System Ready",
    stableTimer = 0,          -- 穩定停留計時器 (秒)
    calibStartAlt = 0,        -- 校正啟動高度
    calibTargetAlt = 200.0,   -- 校正目標固定 200m
    calibPhase = "GROUND_SEARCH", -- "GROUND_SEARCH", "CLIMB_TO_200", "STABILIZE_200"
    calibThrottle = 0.0,      -- 校正微調油門
    rampRate = 0.0001         -- 適應性加力速率
}

FlightCore.altPID = PID.new(0.5, 0.02, 0.6, -1.5, 1.5)

FlightCore.engines = { FL = nil, FR = nil, BL = nil, BR = nil }
FlightCore.engineOutputs  = { FL = 0, FR = 0, BL = 0, BR = 0 }
FlightCore.virtualOutputs = { FL = 0.0, FR = 0.0, BL = 0.0, BR = 0.0 }

FlightCore.altiSensor = nil
FlightCore.gimbalSensor = nil
FlightCore.gimbalAvailable = false
FlightCore.masterModem = nil
FlightCore.outputSides = {"bottom", "top", "left", "right", "back"}
FlightCore.pwmTick = 0

local function matchPrefix(lbl, prefix)
    local u = string.upper(lbl or "")
    return u == prefix or u:sub(1, #prefix + 1) == (prefix .. "_") or u:sub(1, #prefix + 1) == (prefix .. "-") or u:sub(1, #prefix + 1) == (prefix .. " ")
end

function FlightCore.initSensorsAndModem()
    FlightCore.altiSensor = peripheral.find("altitude_sensor")
    FlightCore.gimbalSensor = peripheral.find("gimbal_sensor")
    FlightCore.masterModem = peripheral.find("modem")
    if FlightCore.masterModem then
        pcall(function() FlightCore.masterModem.open(101) end)
    end
end

function FlightCore.getGimbalData()
    if not FlightCore.gimbalSensor then
        FlightCore.gimbalSensor = peripheral.find("gimbal_sensor")
    end
    if FlightCore.gimbalSensor then
        local ok, angles = pcall(function() return FlightCore.gimbalSensor.getAngles() end)
        if ok and type(angles) == "table" then
            FlightCore.gimbalAvailable = true
            return angles[1] or 0, angles[2] or 0
        end
    end
    FlightCore.gimbalAvailable = false
    return 0, 0
end

function FlightCore.scanQuadTurtles()
    FlightCore.initSensorsAndModem()
    FlightCore.engines.FL = nil
    FlightCore.engines.FR = nil
    FlightCore.engines.BL = nil
    FlightCore.engines.BR = nil

    local unmapped = {}
    local pNames = peripheral.getNames()

    for _, name in ipairs(pNames) do
        local pType = peripheral.getType(name)
        if pType == "turtle" or pType == "computer" or pType == "redstone_relay" or pType == "modem" then
            local p = peripheral.wrap(name)
            local label = (p.getLabel and p.getLabel()) or name

            if matchPrefix(label, "FL") then
                FlightCore.engines.FL = { name = name, p = p, label = label }
            elseif matchPrefix(label, "FR") then
                FlightCore.engines.FR = { name = name, p = p, label = label }
            elseif matchPrefix(label, "BL") then
                FlightCore.engines.BL = { name = name, p = p, label = label }
            elseif matchPrefix(label, "BR") then
                FlightCore.engines.BR = { name = name, p = p, label = label }
            else
                table.insert(unmapped, { name = name, p = p, label = label })
            end
        end
    end

    local slots = {"FL", "FR", "BL", "BR"}
    for _, slot in ipairs(slots) do
        if not FlightCore.engines[slot] and #unmapped > 0 then
            local fallbackNode = table.remove(unmapped, 1)
            FlightCore.engines[slot] = fallbackNode
        end
    end
end

function FlightCore.resolvePwmOutput(val)
    if val <= 0.0001 then return 0 end
    if val >= 14.999 then return 15 end
    local base = math.floor(val)
    local frac = val - base
    local threshold = frac * 20
    if FlightCore.pwmTick < threshold then
        return math.min(15, base + 1)
    else
        return base
    end
end

function FlightCore.outputToEngine(node, power)
    if not node or not node.p then return end
    pcall(function()
        if node.p.setAnalogOutput then
            for _, s in ipairs(FlightCore.outputSides) do
                node.p.setAnalogOutput(s, power)
            end
        end
    end)
end

function FlightCore.applyQuadThrust(baseThrust, deltaAlt, deltaPitch, deltaRoll)
    FlightCore.pwmTick = (FlightCore.pwmTick + 1) % 20

    local rawFL = math.max(0.0, math.min(15.0, baseThrust + deltaAlt))
    local rawFR = math.max(0.0, math.min(15.0, baseThrust + deltaAlt))
    local rawBL = math.max(0.0, math.min(15.0, baseThrust + deltaAlt))
    local rawBR = math.max(0.0, math.min(15.0, baseThrust + deltaAlt))

    FlightCore.virtualOutputs.FL = rawFL
    FlightCore.virtualOutputs.FR = rawFR
    FlightCore.virtualOutputs.BL = rawBL
    FlightCore.virtualOutputs.BR = rawBR

    FlightCore.engineOutputs.FL = FlightCore.resolvePwmOutput(rawFL)
    FlightCore.engineOutputs.FR = FlightCore.resolvePwmOutput(rawFR)
    FlightCore.engineOutputs.BL = FlightCore.resolvePwmOutput(rawBL)
    FlightCore.engineOutputs.BR = FlightCore.resolvePwmOutput(rawBR)

    FlightCore.outputToEngine(FlightCore.engines.FL, FlightCore.engineOutputs.FL)
    FlightCore.outputToEngine(FlightCore.engines.FR, FlightCore.engineOutputs.FR)
    FlightCore.outputToEngine(FlightCore.engines.BL, FlightCore.engineOutputs.BL)
    FlightCore.outputToEngine(FlightCore.engines.BR, FlightCore.engineOutputs.BR)

    if not FlightCore.masterModem then
        FlightCore.masterModem = peripheral.find("modem")
    end
    if FlightCore.masterModem then
        pcall(function()
            FlightCore.masterModem.transmit(100, 101, {
                FL = FlightCore.engineOutputs.FL,
                FR = FlightCore.engineOutputs.FR,
                BL = FlightCore.engineOutputs.BL,
                BR = FlightCore.engineOutputs.BR
            })
        end)
    end
end

-- 飛控對外操作指令 (API)
function FlightCore.setTargetAlt(alt)
    FlightCore.state.targetAlt = math.max(0, alt)
    FlightCore.state.statusMsg = string.format("Target Altitude set: %.0fm", FlightCore.state.targetAlt)
end

function FlightCore.adjustTargetAlt(delta)
    FlightCore.state.targetAlt = math.max(0, FlightCore.state.targetAlt + delta)
end

function FlightCore.lockCurrentAlt()
    local cur = FlightCore.altiSensor and FlightCore.altiSensor.getHeight() or 0
    FlightCore.state.targetAlt = math.floor(cur + 0.5)
    FlightCore.state.virtualAlt = cur
    FlightCore.state.statusMsg = string.format("Target locked to current: %.0fm", FlightCore.state.targetAlt)
end

function FlightCore.adjustBaseThrottle(delta)
    if FlightCore.state.baseThrottle < 1.0 and delta > 0 then
        FlightCore.state.baseThrottle = math.min(15.0, math.floor((FlightCore.state.baseThrottle + 0.05) * 100 + 0.5) / 100)
    elseif FlightCore.state.baseThrottle <= 1.0 and delta < 0 then
        FlightCore.state.baseThrottle = math.max(0.0, math.floor((FlightCore.state.baseThrottle - 0.05) * 100 + 0.5) / 100)
    else
        FlightCore.state.baseThrottle = math.max(0.0, math.min(15.0, FlightCore.state.baseThrottle + delta))
    end
end

function FlightCore.startCalibration()
    FlightCore.scanQuadTurtles()
    local cur = FlightCore.altiSensor and FlightCore.altiSensor.getHeight() or 100
    FlightCore.state.calibStartAlt = cur
    FlightCore.state.calibTargetAlt = 200.0
    FlightCore.state.virtualAlt = cur
    FlightCore.state.targetAlt = 200.0
    FlightCore.state.calibPhase = "GROUND_SEARCH"
    FlightCore.state.calibThrottle = 0.0
    FlightCore.state.baseThrottle = 0.0
    FlightCore.state.rampRate = 0.0001
    FlightCore.state.mode = "CALIBRATING"
    FlightCore.state.stableTimer = 0
    FlightCore.altPID:reset()
    FlightCore.state.statusMsg = "Calib: Searching Lift to 200m..."
end

function FlightCore.holdAltitude()
    FlightCore.state.mode = "HOLD_ALT"
    local cur = FlightCore.altiSensor and FlightCore.altiSensor.getHeight() or FlightCore.state.targetAlt
    FlightCore.state.virtualAlt = cur
    FlightCore.altPID:reset()
    FlightCore.state.statusMsg = string.format("Holding Altitude: %.0fm", FlightCore.state.targetAlt)
end

function FlightCore.stopEngines()
    FlightCore.state.mode = "IDLE"
    FlightCore.altPID:reset()
    FlightCore.applyQuadThrust(0, 0, 0, 0)
    FlightCore.state.statusMsg = "All 4 Engines Stopped"
end

-- 1.3 核心閉環動力迴圈 (20Hz)
function FlightCore.step()
    local currAlt = FlightCore.altiSensor and FlightCore.altiSensor.getHeight() or 0
    local currVspeed = FlightCore.altiSensor and FlightCore.altiSensor.getVerticalSpeed() or 0

    if FlightCore.state.mode == "HOLD_ALT" then
        if FlightCore.state.virtualAlt < FlightCore.state.targetAlt then
            FlightCore.state.virtualAlt = math.min(FlightCore.state.targetAlt, FlightCore.state.virtualAlt + 0.05)
        elseif FlightCore.state.virtualAlt > FlightCore.state.targetAlt then
            FlightCore.state.virtualAlt = math.max(FlightCore.state.targetAlt, FlightCore.state.virtualAlt - 0.05)
        end

        local altError = FlightCore.state.virtualAlt - currAlt
        local deltaAlt = FlightCore.altPID:update(altError)

        if currVspeed > 0.8 then deltaAlt = deltaAlt - 0.3 end
        if currVspeed < -0.8 then deltaAlt = deltaAlt + 0.3 end

        FlightCore.applyQuadThrust(FlightCore.state.baseThrottle, deltaAlt, 0, 0)

    elseif FlightCore.state.mode == "CALIBRATING" then
        local climbDist = currAlt - FlightCore.state.calibStartAlt

        if FlightCore.state.calibPhase == "GROUND_SEARCH" then
            FlightCore.state.calibThrottle = math.min(15.0, FlightCore.state.calibThrottle + FlightCore.state.rampRate)
            FlightCore.state.baseThrottle = math.floor(FlightCore.state.calibThrottle * 1000 + 0.5) / 1000

            if climbDist < 0.6 and currVspeed < 0.15 then
                FlightCore.state.rampRate = math.min(0.008, FlightCore.state.rampRate + 0.00003)
            end

            FlightCore.applyQuadThrust(FlightCore.state.baseThrottle, 0, 0, 0)
            FlightCore.state.statusMsg = string.format("Search: %4.2f/15 (R:%5.4f)", FlightCore.state.baseThrottle, FlightCore.state.rampRate * 20)

            if climbDist >= 0.6 or currVspeed >= 0.18 then
                FlightCore.state.rampRate = 0.0
                FlightCore.state.virtualAlt = currAlt
                FlightCore.state.calibPhase = "CLIMB_TO_200"
                FlightCore.altPID:reset()
                FlightCore.state.statusMsg = string.format("Airborne! Climbing to 200m (B:%4.2f)", FlightCore.state.baseThrottle)
            end

        elseif FlightCore.state.calibPhase == "CLIMB_TO_200" then
            if currVspeed < 0.10 and currAlt < 195.0 and FlightCore.state.baseThrottle < 14 then
                FlightCore.state.calibThrottle = math.min(15.0, FlightCore.state.calibThrottle + 0.0004)
                FlightCore.state.baseThrottle = math.floor(FlightCore.state.calibThrottle * 1000 + 0.5) / 1000
            elseif currVspeed > 0.65 and FlightCore.state.baseThrottle > 0.02 then
                FlightCore.state.calibThrottle = math.max(0.01, FlightCore.state.calibThrottle - 0.001)
                FlightCore.state.baseThrottle = math.floor(FlightCore.state.calibThrottle * 1000 + 0.5) / 1000
            end

            if currAlt >= 195.0 then
                FlightCore.state.virtualAlt = 200.0
                FlightCore.state.calibPhase = "STABILIZE_200"
                FlightCore.state.stableTimer = 0
                FlightCore.altPID:reset()
            else
                FlightCore.state.virtualAlt = math.min(200.0, currAlt + 1.0)
            end

            local altError = FlightCore.state.virtualAlt - currAlt
            local deltaAlt = FlightCore.altPID:update(altError)
            FlightCore.applyQuadThrust(FlightCore.state.baseThrottle, deltaAlt, 0, 0)
            FlightCore.state.statusMsg = string.format("Climbing: %.0f/200m (B:%4.2f V:%+.2f)", currAlt, FlightCore.state.baseThrottle, currVspeed)

        elseif FlightCore.state.calibPhase == "STABILIZE_200" then
            FlightCore.state.virtualAlt = 200.0
            local altError = 200.0 - currAlt
            local deltaAlt = FlightCore.altPID:update(altError)

            if currVspeed > 0.6 then deltaAlt = deltaAlt - 0.25 end
            if currVspeed < -0.6 then deltaAlt = deltaAlt + 0.25 end

            FlightCore.applyQuadThrust(FlightCore.state.baseThrottle, deltaAlt, 0, 0)

            if math.abs(altError) <= 1.0 and math.abs(currVspeed) <= 0.20 then
                FlightCore.state.stableTimer = FlightCore.state.stableTimer + 0.05
                FlightCore.state.statusMsg = string.format("200m Steady Hover: %2.1fs/5.0s", FlightCore.state.stableTimer)

                if FlightCore.state.stableTimer >= 5.0 then
                    FlightCore.state.mode = "HOLD_ALT"
                    FlightCore.state.targetAlt = 200.0
                    FlightCore.state.virtualAlt = 200.0
                    FlightCore.state.statusMsg = string.format("CALIB COMPLETE! Best Base: %4.2f", FlightCore.state.baseThrottle)
                end
            else
                FlightCore.state.stableTimer = 0
                if altError > 0.6 and FlightCore.state.baseThrottle < 14 and currVspeed < 0.10 then
                    FlightCore.state.baseThrottle = math.min(15.0, FlightCore.state.baseThrottle + 0.001)
                elseif altError < -0.6 and FlightCore.state.baseThrottle > 0.01 and currVspeed > -0.10 then
                    FlightCore.state.baseThrottle = math.max(0.005, FlightCore.state.baseThrottle - 0.001)
                end
                FlightCore.state.statusMsg = string.format("200m Hover Trim (B:%4.2f)", FlightCore.state.baseThrottle)
            end
        end

    elseif FlightCore.state.mode == "IDLE" then
        FlightCore.applyQuadThrust(0, 0, 0, 0)
    end
end

-- ========================================================
-- PART 2: 顯示驅動器模組 (DISPLAY DRIVERS - FRONTEND VIEWS)
-- ========================================================
local Drivers = {}

-- --------------------------------------------------------
-- 2.1 驅動 A: CC-DirectGPU-Mod 全彩航空儀表
-- --------------------------------------------------------
Drivers.directgpu = {
    name = "VTOL Airship " .. VERSION .. " (DirectGPU)",
    shortTag = "DirectGPU",
    currentView = "OVERVIEW",
    isMenuOpen = false,
    isAvailable = function()
        return peripheral.find("directgpu") ~= nil
    end,
    init = function(self)
        self.gpu = peripheral.find("directgpu")
        self.displayId = self.gpu.autoDetectAndCreateDisplay()
        if not self.displayId then error("DirectGPU 未檢測到 Monitor 螢幕！") end
        self.buttons = {}
        self.currentView = "OVERVIEW"
        self.isMenuOpen = false
    end,
    addBtn = function(self, x, y, w, h, text, bg, fg, act)
        table.insert(self.buttons, {x=x, y=y, w=w, h=h, text=text, bg=bg, fg=fg, action=act})
    end,
    drawA350Dial = function(self, cx, cy, r, val, maxVal, label, sig)
        local labelSize = math.max(12, math.floor(r * 0.48))
        local lblOffset = math.floor(#label * (labelSize * 0.32))
        self.gpu.drawText(self.displayId, label, cx - lblOffset, cy - r - math.floor(labelSize * 1.3), 200, 230, 255, "Arial", labelSize, "bold")

        local arcPts, arcPtsOuter = {}, {}
        for deg = 210, -30, -10 do
            local rad = math.rad(deg)
            table.insert(arcPts, { math.floor(cx + r * math.cos(rad) + 0.5), math.floor(cy - r * math.sin(rad) + 0.5) })
            table.insert(arcPtsOuter, { math.floor(cx + (r + 1) * math.cos(rad) + 0.5), math.floor(cy - (r + 1) * math.sin(rad) + 0.5) })
        end
        self.gpu.drawPolylines(self.displayId, arcPts, 160, 180, 205)
        self.gpu.drawPolylines(self.displayId, arcPtsOuter, 120, 140, 165)

        for _, deg in ipairs({210, 90, -30}) do
            local rad = math.rad(deg)
            self.gpu.drawLine(self.displayId, math.floor(cx + (r - 3) * math.cos(rad) + 0.5), math.floor(cy - (r - 3) * math.sin(rad) + 0.5),
                                           math.floor(cx + (r + 6) * math.cos(rad) + 0.5), math.floor(cy - (r + 6) * math.sin(rad) + 0.5), 180, 200, 225)
        end

        local redPts = {}
        for deg = 10, -30, -8 do
            local rad = math.rad(deg)
            table.insert(redPts, { math.floor(cx + r * math.cos(rad) + 0.5), math.floor(cy - r * math.sin(rad) + 0.5) })
        end
        self.gpu.drawPolylines(self.displayId, redPts, 255, 55, 55)

        local ratio = math.min(1.0, math.max(0.0, val / (maxVal or 15.0)))
        local nRad = math.rad(210 - ratio * 240)
        local nx = math.floor(cx + (r - 3) * math.cos(nRad) + 0.5)
        local ny = math.floor(cy - (r - 3) * math.sin(nRad) + 0.5)
        self.gpu.drawLine(self.displayId, cx, cy, nx, ny, 50, 255, 100)
        self.gpu.drawCircle(self.displayId, cx, cy, math.max(2, math.floor(r * 0.14)), 200, 220, 240, true)

        local boxW = math.max(46, math.floor(r * 1.55))
        local boxH = math.max(20, math.floor(r * 0.70))
        local boxX = cx - math.floor(boxW / 2)
        local boxY = cy + math.floor(r * 0.28)
        self.gpu.fillRect(self.displayId, boxX, boxY, boxW, boxH, 12, 18, 28)
        self.gpu.drawPolylines(self.displayId, {{boxX, boxY}, {boxX+boxW, boxY}, {boxX+boxW, boxY+boxH}, {boxX, boxY+boxH}, {boxX, boxY}}, 40, 150, 200)

        local valStr = string.format("%4.1f", val)
        local numFontSize = math.max(12, math.floor(boxH * 0.70))
        self.gpu.drawText(self.displayId, valStr, boxX + 4, boxY + 2, 70, 255, 120, "Arial", numFontSize, "bold")

        local sigStr = string.format("%d/15", sig or 0)
        local sigFontSize = math.max(10, math.floor(boxH * 0.55))
        self.gpu.drawText(self.displayId, sigStr, cx - math.floor(#sigStr * 3.5), boxY + boxH + 4, 150, 190, 225, "Arial", sigFontSize, "plain")
    end,
    draw = function(self)
        local dispInfo = self.gpu.getDisplayInfo(self.displayId)
        local screenW = dispInfo.pixelWidth or 320
        local screenH = dispInfo.pixelHeight or 240
        self.gpu.clear(self.displayId, 15, 20, 30)
        self.buttons = {}

        -- 1. 頂部標題列與右上角 MENU 切換鈕
        local headerH = math.max(28, math.floor(screenH * 0.09))
        self.gpu.fillRect(self.displayId, 0, 0, screenW, headerH, 25, 40, 65)
        local titleFontSize = math.max(12, math.floor(headerH * 0.48))
        local viewTag = self.isMenuOpen and "SELECT VIEW" or self.currentView
        self.gpu.drawText(self.displayId, string.format("VTOL Airship %s - [%s]", VERSION, viewTag), 12, math.floor((headerH - titleFontSize) / 2), 240, 245, 255, "Arial", titleFontSize, "bold")

        local menuBtnW = math.max(70, math.floor(screenW * 0.20))
        local menuBtnH = headerH - 6
        local menuBtnX = screenW - menuBtnW - 4
        local menuBtnY = 3
        if self.isMenuOpen then
            self:addBtn(menuBtnX, menuBtnY, menuBtnW, menuBtnH, "[X CLOSE]", {190, 45, 45}, {255, 255, 255}, function() self.isMenuOpen = false end)
        else
            self:addBtn(menuBtnX, menuBtnY, menuBtnW, menuBtnH, "[= MENU]", {35, 80, 150}, {255, 255, 255}, function() self.isMenuOpen = true end)
        end

        -- 2. 視圖分流路由
        if self.isMenuOpen then
            self:renderMenu(screenW, screenH, headerH)
        else
            if self.currentView == "OVERVIEW" then
                self:renderOverview(screenW, screenH, headerH)
            elseif self.currentView == "PFD" then
                self:renderPFD(screenW, screenH, headerH)
            elseif self.currentView == "ECAM" then
                self:renderECAM(screenW, screenH, headerH)
            elseif self.currentView == "NAV" then
                self:renderNAV(screenW, screenH, headerH)
            elseif self.currentView == "SYS" then
                self:renderSYS(screenW, screenH, headerH)
            else
                self:renderOverview(screenW, screenH, headerH)
            end
        end

        -- 繪製所有註冊按鈕
        for _, btn in ipairs(self.buttons) do
            self.gpu.fillRect(self.displayId, btn.x, btn.y, btn.w, btn.h, btn.bg[1], btn.bg[2], btn.bg[3])
            local btnFontSize = math.max(11, math.floor(btn.h * 0.44))
            local tx = btn.x + math.max(2, math.floor((btn.w - #btn.text * (btnFontSize * 0.58)) / 2))
            local ty = btn.y + math.floor((btn.h - btnFontSize) / 2)
            self.gpu.drawText(self.displayId, btn.text, tx, ty, btn.fg[1], btn.fg[2], btn.fg[3], "Arial", btnFontSize, "bold")
        end

        self.gpu.updateDisplay(self.displayId)
    end,
    renderMenu = function(self, sw, sh, hh)
        local cardW = math.floor((sw - 24) / 2)
        local cardH = math.floor((sh - hh - 28) / 3)
        local startY = hh + 8

        local function addMenuCard(col, row, title, desc, targetView, color)
            local cx = 8 + (col - 1) * (cardW + 8)
            local cy = startY + (row - 1) * (cardH + 6)
            self:addBtn(cx, cy, cardW, cardH, title, color, {255, 255, 255}, function()
                self.currentView = targetView
                self.isMenuOpen = false
            end)
        end

        addMenuCard(1, 1, "1. OVERVIEW (ALL-IN-ONE)", "Classic cockpit view", "OVERVIEW", {30, 65, 110})
        addMenuCard(2, 1, "2. PFD (PRIMARY FLIGHT)", "Large Horizon & Alt", "PFD", {25, 95, 75})
        addMenuCard(1, 2, "3. ECAM (ENGINES & LIFT)", "4 Big Dials & Calib", "ECAM", {110, 45, 25})
        addMenuCard(2, 2, "4. NAV (PRESETS & CRUISE)", "0/80/150/200/300m", "NAV", {20, 100, 130})

        local btmW = sw - 16
        local cy = startY + 2 * (cardH + 6)
        self:addBtn(8, cy, btmW, cardH, "5. SYS (SYSTEM DIAGNOSTICS & TURTLES)", {75, 45, 110}, {255, 255, 255}, function()
            self.currentView = "SYS"
            self.isMenuOpen = false
        end)
    end,
    renderOverview = function(self, sw, sh, hh)
        local currAlt = FlightCore.altiSensor and FlightCore.altiSensor.getHeight() or 0
        local currVspeed = FlightCore.altiSensor and FlightCore.altiSensor.getVerticalSpeed() or 0
        local currPitch, currRoll = FlightCore.getGimbalData()

        local instY = hh + 6
        local instH = math.floor((sh - hh) * 0.52)
        local leftW = math.max(140, math.floor(sw * 0.35))
        local pfdX = 6

        self.gpu.fillRect(self.displayId, pfdX, instY, leftW, instH, 20, 26, 38)
        self.gpu.drawText(self.displayId, "PRIMARY FLIGHT DISPLAY", pfdX + 10, instY + 6, 170, 200, 230, "Arial", 10, "bold")

        local gR = math.min(36, math.floor(instH * 0.30))
        local gCX = pfdX + math.floor(leftW * 0.28)
        local gCY = instY + math.floor(instH * 0.54)
        self.gpu.drawCircle(self.displayId, gCX, gCY, gR, 30, 40, 55, true)
        self.gpu.drawCircle(self.displayId, gCX, gCY, gR, 80, 170, 240, false)
        local altText = string.format("%.0f", currAlt)
        local altFontSize = math.max(16, math.floor(gR * 0.65))
        self.gpu.drawText(self.displayId, altText, gCX - math.floor(#altText * (altFontSize * 0.32)), gCY - math.floor(altFontSize * 0.55), 255, 255, 255, "Arial", altFontSize, "bold")
        self.gpu.drawText(self.displayId, "ALT (M)", gCX - 16, gCY + math.floor(gR * 0.38), 160, 205, 240, "Arial", 9, "bold")

        local textX = pfdX + math.floor(leftW * 0.54)
        local rowSpacing = math.floor((instH - 26) / 5)
        local pfdFontSize = math.max(12, math.min(18, math.floor(rowSpacing * 0.68)))

        self.gpu.drawText(self.displayId, string.format("TGT: %.0fm", FlightCore.state.targetAlt), textX, instY + 18, 80, 230, 255, "Arial", pfdFontSize, "bold")
        self.gpu.drawText(self.displayId, string.format("V.S: %+.2f", currVspeed), textX, instY + 18 + rowSpacing, 255, 205, 75, "Arial", pfdFontSize, "bold")
        local mCol = (FlightCore.state.mode == "HOLD_ALT") and {80, 255, 120} or (FlightCore.state.mode == "CALIBRATING" and {80, 200, 255} or {255, 80, 80})
        self.gpu.drawText(self.displayId, string.format("MODE: %s", FlightCore.state.mode:sub(1,7)), textX, instY + 18 + rowSpacing * 2, mCol[1], mCol[2], mCol[3], "Arial", pfdFontSize, "bold")
        self.gpu.drawText(self.displayId, string.format("P:%+4.1f* R:%+4.1f*", currPitch, currRoll), textX, instY + 18 + rowSpacing * 3, 180, 220, 255, "Arial", math.max(10, pfdFontSize - 2), "plain")
        if FlightCore.gimbalAvailable then
            self.gpu.drawText(self.displayId, "GIMBAL: ACTIVE", textX, instY + 18 + rowSpacing * 4, 80, 255, 120, "Arial", math.max(11, pfdFontSize - 1), "bold")
        else
            self.gpu.drawText(self.displayId, "GIMBAL: NO SENSOR", textX, instY + 18 + rowSpacing * 4, 255, 60, 60, "Arial", math.max(11, pfdFontSize - 1), "bold")
        end

        local ecamX = pfdX + leftW + 6
        local ecamW = sw - ecamX - 6
        self.gpu.fillRect(self.displayId, ecamX, instY, ecamW, instH, 18, 24, 34)

        local slotW = math.floor((ecamW - 8) / 4)
        local slots = {"FL", "FR", "BL", "BR"}
        local dialR = math.min(math.floor(slotW * 0.35), math.floor(instH * 0.30))
        local engCenterY = instY + math.floor(instH * 0.44)

        for i, slot in ipairs(slots) do
            local engCenterX = ecamX + 4 + math.floor((i - 0.5) * slotW)
            local node = FlightCore.engines[slot]
            local lbl = node and (node.label or node.name or slot) or slot
            if #lbl > 5 then lbl = lbl:sub(1, 5) end
            self:drawA350Dial(engCenterX, engCenterY, dialR, FlightCore.virtualOutputs[slot], 15.0, lbl, FlightCore.engineOutputs[slot])
        end

        local statY = instY + instH + 6
        local statH = math.max(20, math.floor(sh * 0.07))
        self.gpu.fillRect(self.displayId, pfdX, statY, sw - 12, statH, 25, 32, 45)
        self.gpu.drawText(self.displayId, string.format("STATUS: %s", FlightCore.state.statusMsg), pfdX + 8, statY + 4, 80, 230, 255, "Arial", math.max(11, math.floor(statH * 0.55)), "bold")

        local btnAreaY = statY + statH + 6
        local btnAreaH = sh - btnAreaY - 6
        local rowH = math.max(18, math.floor((btnAreaH - 12) / 3))

        local bY1 = btnAreaY
        local bW1 = math.floor((sw - 12 - 16) / 5)
        self:addBtn(pfdX, bY1, bW1, rowH, "+10m", {30, 100, 45}, {240, 255, 240}, function() FlightCore.adjustTargetAlt(10) end)
        self:addBtn(pfdX + bW1 + 4, bY1, bW1, rowH, "+1m", {45, 130, 55}, {240, 255, 240}, function() FlightCore.adjustTargetAlt(1) end)
        self:addBtn(pfdX + (bW1 + 4)*2, bY1, bW1, rowH, "-1m", {210, 95, 20}, {255, 245, 230}, function() FlightCore.adjustTargetAlt(-1) end)
        self:addBtn(pfdX + (bW1 + 4)*3, bY1, bW1, rowH, "-10m", {190, 40, 40}, {255, 235, 235}, function() FlightCore.adjustTargetAlt(-10) end)
        self:addBtn(pfdX + (bW1 + 4)*4, bY1, bW1, rowH, "SET CURR", {0, 135, 150}, {230, 250, 255}, function() FlightCore.lockCurrentAlt() end)

        local bY2 = bY1 + rowH + 6
        local bW2 = math.floor((sw - 12 - 12) / 4)
        self:addBtn(pfdX, bY2, bW2, rowH, "BASE +", {0, 110, 95}, {230, 250, 245}, function() FlightCore.adjustBaseThrottle(1.0) end)
        self:addBtn(pfdX + bW2 + 4, bY2, bW2, rowH, "BASE -", {55, 70, 80}, {235, 240, 245}, function() FlightCore.adjustBaseThrottle(-1.0) end)
        self:addBtn(pfdX + (bW2 + 4)*2, bY2, bW2, rowH, "RE-SCAN", {25, 100, 190}, {230, 245, 255}, function() FlightCore.scanQuadTurtles(); FlightCore.state.statusMsg="Hardware Re-scanned!" end)
        self:addBtn(pfdX + (bW2 + 4)*3, bY2, bW2, rowH, "CALIBRATE", {110, 30, 155}, {250, 230, 255}, function() FlightCore.startCalibration() end)

        local bY3 = bY2 + rowH + 6
        local bW3 = math.floor((sw - 12 - 6) / 2)
        local bH3 = math.max(rowH, sh - bY3 - 6)
        local holdBg = (FlightCore.state.mode == "HOLD_ALT") and {45, 140, 60} or {30, 90, 40}
        self:addBtn(pfdX, bY3, bW3, bH3, " [ HOLD ALT ] ", holdBg, {255, 255, 255}, function() FlightCore.holdAltitude() end)
        local stopBg = (FlightCore.state.mode == "IDLE") and {200, 45, 45} or {160, 30, 30}
        self:addBtn(pfdX + bW3 + 6, bY3, bW3, bH3, " [ STOP / IDLE ] ", stopBg, {255, 255, 255}, function() FlightCore.stopEngines() end)
    end,
    renderPFD = function(self, sw, sh, hh)
        local currAlt = FlightCore.altiSensor and FlightCore.altiSensor.getHeight() or 0
        local currVspeed = FlightCore.altiSensor and FlightCore.altiSensor.getVerticalSpeed() or 0
        local currPitch, currRoll = FlightCore.getGimbalData()

        local mainH = math.floor((sh - hh) * 0.65)
        local mainY = hh + 6

        -- 左半部：超大人工地平線球 (Horizon / Attitude Gyro)
        local leftW = math.floor(sw * 0.52)
        self.gpu.fillRect(self.displayId, 6, mainY, leftW, mainH, 20, 26, 38)
        local hCX = 6 + math.floor(leftW / 2)
        local hCY = mainY + math.floor(mainH / 2)
        local hR = math.min(math.floor(leftW * 0.40), math.floor(mainH * 0.42))

        -- 姿態球底色
        self.gpu.drawCircle(self.displayId, hCX, hCY, hR, 25, 40, 60, true)
        self.gpu.drawCircle(self.displayId, hCX, hCY, hR, 80, 180, 255, false)

        -- 地平線與橫滾指標
        local rollRad = math.rad(-currRoll)
        local lx1 = hCX - math.floor(hR * 0.85 * math.cos(rollRad))
        local ly1 = hCY - math.floor(hR * 0.85 * math.sin(rollRad))
        local lx2 = hCX + math.floor(hR * 0.85 * math.cos(rollRad))
        local ly2 = hCY + math.floor(hR * 0.85 * math.sin(rollRad))
        self.gpu.drawLine(self.displayId, lx1, ly1, lx2, ly2, 255, 215, 0)
        self.gpu.drawCircle(self.displayId, hCX, hCY, 4, 255, 80, 80, true)

        self.gpu.drawText(self.displayId, string.format("PITCH: %+5.1f*  ROLL: %+5.1f*", currPitch, currRoll), 12, mainY + mainH - 18, 200, 230, 255, "Arial", 12, "bold")

        -- 右半部：超大高度與垂直速度數值卡片
        local rightX = 6 + leftW + 6
        local rightW = sw - rightX - 6
        self.gpu.fillRect(self.displayId, rightX, mainY, rightW, mainH, 24, 32, 46)

        local cardFontSize = math.max(13, math.floor(mainH * 0.10))
        self.gpu.drawText(self.displayId, "CURRENT ALTITUDE", rightX + 10, mainY + 12, 160, 200, 240, "Arial", cardFontSize, "plain")
        local altNumSize = math.max(24, math.floor(mainH * 0.24))
        self.gpu.drawText(self.displayId, string.format("%.1f m", currAlt), rightX + 10, mainY + 14 + cardFontSize, 255, 255, 255, "Arial", altNumSize, "bold")

        local midY = mainY + 14 + cardFontSize + altNumSize + 8
        self.gpu.drawText(self.displayId, string.format("TARGET ALT: %.0f m", FlightCore.state.targetAlt), rightX + 10, midY, 80, 230, 255, "Arial", cardFontSize, "bold")
        self.gpu.drawText(self.displayId, string.format("VERT SPEED: %+.2f m/s", currVspeed), rightX + 10, midY + cardFontSize + 6, 255, 205, 75, "Arial", cardFontSize, "bold")
        self.gpu.drawText(self.displayId, string.format("STATUS: %s", FlightCore.state.statusMsg), rightX + 10, midY + (cardFontSize + 6) * 2, 100, 255, 150, "Arial", math.max(11, cardFontSize - 1), "bold")

        -- 下半部快捷操作按鈕
        local bY = mainY + mainH + 6
        local bH = sh - bY - 6
        local bW = math.floor((sw - 12 - 20) / 6)
        self:addBtn(6, bY, bW, bH, "+10m", {30, 100, 45}, {240, 255, 240}, function() FlightCore.adjustTargetAlt(10) end)
        self:addBtn(6 + (bW+4), bY, bW, bH, "+1m", {45, 130, 55}, {240, 255, 240}, function() FlightCore.adjustTargetAlt(1) end)
        self:addBtn(6 + (bW+4)*2, bY, bW, bH, "-1m", {210, 95, 20}, {255, 245, 230}, function() FlightCore.adjustTargetAlt(-1) end)
        self:addBtn(6 + (bW+4)*3, bY, bW, bH, "-10m", {190, 40, 40}, {255, 235, 235}, function() FlightCore.adjustTargetAlt(-10) end)
        local holdBg = (FlightCore.state.mode == "HOLD_ALT") and {45, 140, 60} or {30, 90, 40}
        self:addBtn(6 + (bW+4)*4, bY, bW, bH, "HOLD ALT", holdBg, {255, 255, 255}, function() FlightCore.holdAltitude() end)
        local stopBg = (FlightCore.state.mode == "IDLE") and {200, 45, 45} or {160, 30, 30}
        self:addBtn(6 + (bW+4)*5, bY, bW, bH, "STOP", stopBg, {255, 255, 255}, function() FlightCore.stopEngines() end)
    end,
    renderECAM = function(self, sw, sh, hh)
        local instY = hh + 6
        local instH = math.floor((sh - hh) * 0.60)
        self.gpu.fillRect(self.displayId, 6, instY, sw - 12, instH, 18, 24, 34)

        local slotW = math.floor((sw - 20) / 4)
        local slots = {"FL", "FR", "BL", "BR"}
        local dialR = math.min(math.floor(slotW * 0.38), math.floor(instH * 0.34))
        local engCenterY = instY + math.floor(instH * 0.46)

        for i, slot in ipairs(slots) do
            local engCenterX = 8 + math.floor((i - 0.5) * slotW)
            local node = FlightCore.engines[slot]
            local lbl = node and (node.label or node.name or slot) or slot
            if #lbl > 6 then lbl = lbl:sub(1, 6) end
            self:drawA350Dial(engCenterX, engCenterY, dialR, FlightCore.virtualOutputs[slot], 15.0, lbl, FlightCore.engineOutputs[slot])
        end

        local statY = instY + instH + 6
        local statH = math.max(20, math.floor(sh * 0.07))
        self.gpu.fillRect(self.displayId, 6, statY, sw - 12, statH, 25, 32, 45)
        self.gpu.drawText(self.displayId, string.format("BASE THRUST: %4.2f/15  |  STATUS: %s", FlightCore.state.baseThrottle, FlightCore.state.statusMsg), 12, statY + 4, 80, 230, 255, "Arial", 12, "bold")

        local bY = statY + statH + 6
        local bH = sh - bY - 6
        local bW = math.floor((sw - 12 - 20) / 6)
        self:addBtn(6, bY, bW, bH, "BASE +", {0, 110, 95}, {230, 250, 245}, function() FlightCore.adjustBaseThrottle(1.0) end)
        self:addBtn(6 + (bW+4), bY, bW, bH, "BASE -", {55, 70, 80}, {235, 240, 245}, function() FlightCore.adjustBaseThrottle(-1.0) end)
        self:addBtn(6 + (bW+4)*2, bY, bW, bH, "RE-SCAN", {25, 100, 190}, {230, 245, 255}, function() FlightCore.scanQuadTurtles(); FlightCore.state.statusMsg="Hardware Re-scanned!" end)
        self:addBtn(6 + (bW+4)*3, bY, bW, bH, "CALIB 200m", {110, 30, 155}, {250, 230, 255}, function() FlightCore.startCalibration() end)
        local holdBg = (FlightCore.state.mode == "HOLD_ALT") and {45, 140, 60} or {30, 90, 40}
        self:addBtn(6 + (bW+4)*4, bY, bW, bH, "HOLD ALT", holdBg, {255, 255, 255}, function() FlightCore.holdAltitude() end)
        local stopBg = (FlightCore.state.mode == "IDLE") and {200, 45, 45} or {160, 30, 30}
        self:addBtn(6 + (bW+4)*5, bY, bW, bH, "STOP", stopBg, {255, 255, 255}, function() FlightCore.stopEngines() end)
    end,
    renderNAV = function(self, sw, sh, hh)
        local currAlt = FlightCore.altiSensor and FlightCore.altiSensor.getHeight() or 0
        local currVspeed = FlightCore.altiSensor and FlightCore.altiSensor.getVerticalSpeed() or 0

        local statH = 34
        self.gpu.fillRect(self.displayId, 6, hh + 6, sw - 12, statH, 20, 28, 40)
        self.gpu.drawText(self.displayId, string.format("ALT: %.1fm  ->  TARGET: %.0fm  (V.S: %+.2f m/s)", currAlt, FlightCore.state.targetAlt, currVspeed), 14, hh + 12, 80, 230, 255, "Arial", 13, "bold")

        local gridY = hh + 6 + statH + 8
        local btnAreaH = sh - gridY - 6
        local rowH = math.floor((btnAreaH - 12) / 3)
        local colW = math.floor((sw - 12 - 8) / 2)

        self:addBtn(6, gridY, colW, rowH, "[ 0m LANDING ]", {160, 50, 50}, {255, 255, 255}, function() FlightCore.setTargetAlt(0) end)
        self:addBtn(6 + colW + 8, gridY, colW, rowH, "[ 80m TREETOP ]", {35, 110, 60}, {255, 255, 255}, function() FlightCore.setTargetAlt(80) end)

        self:addBtn(6, gridY + rowH + 6, colW, rowH, "[ 150m CRUISE ]", {25, 90, 140}, {255, 255, 255}, function() FlightCore.setTargetAlt(150) end)
        self:addBtn(6 + colW + 8, gridY + rowH + 6, colW, rowH, "[ 200m CALIBRATE ]", {100, 40, 140}, {255, 255, 255}, function() FlightCore.setTargetAlt(200) end)

        self:addBtn(6, gridY + (rowH + 6)*2, colW, rowH, "[ 300m HIGH-ALT ]", {20, 110, 150}, {255, 255, 255}, function() FlightCore.setTargetAlt(300) end)
        self:addBtn(6 + colW + 8, gridY + (rowH + 6)*2, colW, rowH, "[ LOCK CURRENT ALT ]", {130, 90, 20}, {255, 255, 255}, function() FlightCore.lockCurrentAlt() end)
    end,
    renderSYS = function(self, sw, sh, hh)
        local cardW = math.floor((sw - 20) / 2)
        local cardH = math.floor((sh - hh - 50) / 2)
        local startY = hh + 6

        local slots = {
            {slot="FL", col=1, row=1, name="Front-Left Turtle"},
            {slot="FR", col=2, row=1, name="Front-Right Turtle"},
            {slot="BL", col=1, row=2, name="Back-Left Turtle"},
            {slot="BR", col=2, row=2, name="Back-Right Turtle"}
        }

        for _, s in ipairs(slots) do
            local cx = 6 + (s.col - 1) * (cardW + 8)
            local cy = startY + (s.row - 1) * (cardH + 6)
            local node = FlightCore.engines[s.slot]
            local online = node ~= nil
            local bg = online and {20, 35, 30} or {40, 20, 20}
            self.gpu.fillRect(self.displayId, cx, cy, cardW, cardH, bg[1], bg[2], bg[3])

            local tagCol = online and {80, 255, 120} or {255, 80, 80}
            self.gpu.drawText(self.displayId, string.format("[%s] %s", s.slot, s.name), cx + 8, cy + 8, 220, 235, 255, "Arial", 12, "bold")
            self.gpu.drawText(self.displayId, string.format("STATUS: %s", online and "ONLINE (PAIRED)" or "OFFLINE / MISSING"), cx + 8, cy + 26, tagCol[1], tagCol[2], tagCol[3], "Arial", 11, "bold")
            local lbl = node and (node.label or node.name) or "None"
            self.gpu.drawText(self.displayId, string.format("NODE: %s | PWM: %d/15", lbl, FlightCore.engineOutputs[s.slot]), cx + 8, cy + 44, 160, 190, 220, "Arial", 10, "plain")
        end

        local btmY = startY + cardH * 2 + 14
        local btmW = math.floor((sw - 16) / 2)
        self:addBtn(6, btmY, btmW, 28, "RE-SCAN HARDWARE", {25, 100, 190}, {255, 255, 255}, function() FlightCore.scanQuadTurtles(); FlightCore.state.statusMsg="Hardware Re-scanned!" end)
        self:addBtn(6 + btmW + 4, btmY, btmW, 28, "STOP ALL ENGINES", {190, 40, 40}, {255, 255, 255}, function() FlightCore.stopEngines() end)
    end,
    handleEvent = function(self, event, p1, p2, p3, p4)
        local clickX, clickY = nil, nil
        if event == "directgpu_touch" then
            if type(p1) == "number" and type(p2) == "number" then clickX, clickY = p1, p2
            elseif type(p2) == "number" and type(p3) == "number" then clickX, clickY = p2, p3 end
        elseif event == "monitor_touch" then
            if type(p2) == "number" and type(p3) == "number" then
                local mon = peripheral.find("monitor")
                local mw, mh = (mon and mon.getSize()) or term.getSize()
                clickX = math.floor(((p2 - 0.5) / mw) * (self.screenW or 320))
                clickY = math.floor(((p3 - 0.5) / mh) * (self.screenH or 240))
            end
        elseif event == "mouse_click" then
            if type(p1) == "table" and p1.x and p1.y then clickX, clickY = p1.x, p1.y
            elseif type(p2) == "number" and type(p3) == "number" then
                local tw, th = term.getSize()
                clickX = math.floor(((p2 - 0.5) / tw) * (self.screenW or 320))
                clickY = math.floor(((p3 - 0.5) / th) * (self.screenH or 240))
            end
        end

        if clickX and clickY then
            for _, btn in ipairs(self.buttons) do
                if clickX >= btn.x and clickX <= btn.x + btn.w and clickY >= btn.y and clickY <= btn.y + btn.h then
                    pcall(btn.action)
                    self:draw()
                    break
                end
            end
        end
    end
}

-- --------------------------------------------------------
-- 2.2 驅動 B: Tom's Peripherals 全彩點陣向量儀表 (內建 5x7 像素字體)
-- --------------------------------------------------------
local FONT_5X7 = {
    ['0'] = {0x3E, 0x51, 0x49, 0x45, 0x3E},
    ['1'] = {0x00, 0x42, 0x7F, 0x40, 0x00},
    ['2'] = {0x42, 0x61, 0x51, 0x49, 0x46},
    ['3'] = {0x21, 0x41, 0x45, 0x4B, 0x31},
    ['4'] = {0x18, 0x14, 0x12, 0x7F, 0x10},
    ['5'] = {0x27, 0x45, 0x45, 0x45, 0x39},
    ['6'] = {0x3C, 0x4A, 0x49, 0x49, 0x30},
    ['7'] = {0x01, 0x71, 0x09, 0x05, 0x03},
    ['8'] = {0x36, 0x49, 0x49, 0x49, 0x36},
    ['9'] = {0x06, 0x49, 0x49, 0x29, 0x1E},
    ['A'] = {0x7E, 0x11, 0x11, 0x11, 0x7E},
    ['B'] = {0x7F, 0x49, 0x49, 0x49, 0x36},
    ['C'] = {0x3E, 0x41, 0x41, 0x41, 0x22},
    ['D'] = {0x7F, 0x41, 0x41, 0x22, 0x1C},
    ['E'] = {0x7F, 0x49, 0x49, 0x49, 0x41},
    ['F'] = {0x7F, 0x09, 0x09, 0x09, 0x01},
    ['G'] = {0x3E, 0x41, 0x49, 0x49, 0x7A},
    ['H'] = {0x7F, 0x08, 0x08, 0x08, 0x7F},
    ['I'] = {0x00, 0x41, 0x7F, 0x41, 0x00},
    ['J'] = {0x20, 0x40, 0x41, 0x3F, 0x01},
    ['K'] = {0x7F, 0x08, 0x14, 0x22, 0x41},
    ['L'] = {0x7F, 0x40, 0x40, 0x40, 0x40},
    ['M'] = {0x7F, 0x02, 0x0C, 0x02, 0x7F},
    ['N'] = {0x7F, 0x04, 0x08, 0x10, 0x7F},
    ['O'] = {0x3E, 0x41, 0x41, 0x41, 0x3E},
    ['P'] = {0x7F, 0x09, 0x09, 0x09, 0x06},
    ['Q'] = {0x3E, 0x41, 0x51, 0x21, 0x5E},
    ['R'] = {0x7F, 0x09, 0x19, 0x29, 0x46},
    ['S'] = {0x46, 0x49, 0x49, 0x49, 0x31},
    ['T'] = {0x01, 0x01, 0x7F, 0x01, 0x01},
    ['U'] = {0x3F, 0x40, 0x40, 0x40, 0x3F},
    ['V'] = {0x1F, 0x20, 0x40, 0x20, 0x1F},
    ['W'] = {0x7F, 0x20, 0x18, 0x20, 0x7F},
    ['X'] = {0x63, 0x14, 0x08, 0x14, 0x63},
    ['Y'] = {0x07, 0x08, 0x70, 0x08, 0x07},
    ['Z'] = {0x61, 0x51, 0x49, 0x45, 0x43},
    ['+'] = {0x08, 0x08, 0x3E, 0x08, 0x08},
    ['-'] = {0x08, 0x08, 0x08, 0x08, 0x08},
    ['.'] = {0x00, 0x60, 0x60, 0x00, 0x00},
    [':'] = {0x00, 0x36, 0x36, 0x00, 0x00},
    ['/'] = {0x20, 0x10, 0x08, 0x04, 0x02},
    ['*'] = {0x14, 0x08, 0x3E, 0x08, 0x14},
    ['['] = {0x00, 0x7F, 0x41, 0x41, 0x00},
    [']'] = {0x00, 0x41, 0x41, 0x7F, 0x00},
    ['('] = {0x00, 0x1C, 0x22, 0x41, 0x00},
    [')'] = {0x00, 0x41, 0x22, 0x1C, 0x00},
    ['%'] = {0x23, 0x13, 0x08, 0x64, 0x62},
    ['>'] = {0x41, 0x22, 0x14, 0x08, 0x00},
    ['<'] = {0x00, 0x08, 0x14, 0x22, 0x41},
    ['='] = {0x14, 0x14, 0x14, 0x14, 0x14},
    ['!'] = {0x00, 0x00, 0x5F, 0x00, 0x00},
    [' '] = {0x00, 0x00, 0x00, 0x00, 0x00}
}

Drivers.toms = {
    name = "VTOL Airship " .. VERSION .. " (Tom)",
    shortTag = "Tom",
    currentView = "OVERVIEW",
    isMenuOpen = false,
    isAvailable = function()
        return (peripheral.find("tm_gpu") or peripheral.find("gpu")) ~= nil
    end,
    init = function(self)
        self.gpu = peripheral.find("tm_gpu") or peripheral.find("gpu")
        pcall(function() if self.gpu.setSize then self.gpu.setSize(64) end end)
        self.gpu.refreshSize()
        sleep(0.05)
        self.screenW, self.screenH = 320, 240
        self.buttons = {}
        self.currentView = "OVERVIEW"
        self.isMenuOpen = false
    end,
    toARGB = function(self, col)
        if type(col) ~= "number" then return 0xFFFFFFFF end
        if col <= 0x00FFFFFF then return col + 0xFF000000 end
        return col
    end,
    drawText = function(self, startX, startY, text, color, scale)
        scale = scale or 1
        text = tostring(text or "")
        local argb = self:toARGB(color)
        local sw, sh = self.screenW, self.screenH

        if self.gpu.drawText and scale == 1 then
            local ok = pcall(self.gpu.drawText, math.max(1, math.min(sw, startX)), math.max(1, math.min(sh, startY)), text, argb)
            if ok then return end
        end

        text = string.upper(text)
        local curX = startX
        local gpu_fr = self.gpu.filledRectangle

        for i = 1, #text do
            local ch = text:sub(i, i)
            local glyph = FONT_5X7[ch] or FONT_5X7[' ']
            for col = 1, 5 do
                local colBits = glyph[col] or 0
                if colBits ~= 0 then
                    local px = curX + (col - 1) * scale
                    if px >= 1 and px + scale - 1 <= sw then
                        local runStart = nil
                        local runLen = 0
                        for row = 0, 7 do
                            local bit = (row < 7) and (bit32.band(colBits, bit32.lshift(1, row)) ~= 0) or false
                            if bit then
                                if not runStart then runStart = row end
                                runLen = runLen + 1
                            else
                                if runStart then
                                    local py = startY + runStart * scale
                                    local ph = runLen * scale
                                    if py >= 1 and py + ph - 1 <= sh then
                                        pcall(gpu_fr, px, py, scale, ph, argb)
                                    end
                                    runStart = nil
                                    runLen = 0
                                end
                            end
                        end
                    end
                end
            end
            curX = curX + 6 * scale
        end
    end,
    draw = function(self)
        local ok, w, h = pcall(function() return self.gpu.getSize() end)
        if ok and w and h and w > 0 and h > 0 then self.screenW, self.screenH = w, h end
        local sw, sh = self.screenW, self.screenH

        local function toARGB(c) return self:toARGB(c) end
        local function sFill(c) pcall(function() self.gpu.fill(toARGB(c)) end) end
        local function sFR(x, y, bw, bh, c)
            x, y = math.max(1, math.min(sw, x)), math.max(1, math.min(sh, y))
            bw, bh = math.max(1, math.min(sw - x + 1, bw)), math.max(1, math.min(sh - y + 1, bh))
            pcall(function() self.gpu.filledRectangle(x, y, bw, bh, toARGB(c)) end)
        end
        local function sR(x, y, bw, bh, c)
            x, y = math.max(1, math.min(sw, x)), math.max(1, math.min(sh, y))
            bw, bh = math.max(1, math.min(sw - x + 1, bw)), math.max(1, math.min(sh - y + 1, bh))
            pcall(function() self.gpu.rectangle(x, y, bw, bh, toARGB(c)) end)
        end
        local function sL(x1, y1, x2, y2, c)
            pcall(function() self.gpu.line(math.max(1, math.min(sw, x1)), math.max(1, math.min(sh, y1)), math.max(1, math.min(sw, x2)), math.max(1, math.min(sh, y2)), toARGB(c)) end)
        end
        local function sLS(x1, y1, x2, y2, c)
            pcall(function()
                if self.gpu.lineS then self.gpu.lineS(math.max(1, math.min(sw, x1)), math.max(1, math.min(sh, y1)), math.max(1, math.min(sw, x2)), math.max(1, math.min(sh, y2)), toARGB(c))
                else self.gpu.line(math.max(1, math.min(sw, x1)), math.max(1, math.min(sh, y1)), math.max(1, math.min(sw, x2)), math.max(1, math.min(sh, y2)), toARGB(c)) end
            end)
        end
        local function sTxt(x, y, txt, fc, sz)
            self:drawText(x, y, txt, fc, sz or 1)
        end
        local function sArc(cx, cy, r, startD, endD, stepD, c)
            local px, py = nil, nil
            local st = (startD > endD) and -math.abs(stepD or 5) or math.abs(stepD or 5)
            for deg = startD, endD, st do
                local rad = math.rad(deg)
                local nx, ny = cx + r * math.cos(rad), cy - r * math.sin(rad)
                if px and py then sLS(px, py, nx, ny, c) end
                px, py = nx, ny
            end
        end

        local function addBtn(x, y, bw, bh, text, bg, fg, act)
            table.insert(self.buttons, {x=x, y=y, w=bw, h=bh, text=text, bg=bg, fg=fg, action=act})
        end

        sFill(0x0F141E)
        self.buttons = {}

        -- 1. 頂部標題列與右上角 MENU 按鈕
        local headerH = math.max(24, math.floor(sh * 0.08))
        sFR(1, 1, sw, headerH, 0x192841)
        local titleSize = (sw >= 300) and 2 or 1
        local viewTag = self.isMenuOpen and "SELECT VIEW" or self.currentView
        sTxt(12, math.floor((headerH - 7 * titleSize) / 2) + 1, string.format("VTOL Airship %s - [%s]", VERSION, viewTag), 0xF0F5FF, titleSize)

        local menuBtnW = (sw >= 300) and 70 or 50
        local menuBtnH = headerH - 4
        local menuBtnX = sw - menuBtnW - 4
        local menuBtnY = 3
        if self.isMenuOpen then
            addBtn(menuBtnX, menuBtnY, menuBtnW, menuBtnH, "[X CLOSE]", 0x992222, 0xFFFFFF, function() self.isMenuOpen = false end)
        else
            addBtn(menuBtnX, menuBtnY, menuBtnW, menuBtnH, "[= MENU]", 0x224488, 0xFFFFFF, function() self.isMenuOpen = true end)
        end

        -- 2. 視圖分流路由
        if self.isMenuOpen then
            local cardW = math.floor((sw - 24) / 2)
            local cardH = math.floor((sh - headerH - 24) / 3)
            local startY = headerH + 6

            local function addMenuCard(col, row, title, targetView, color)
                local cx = 6 + (col - 1) * (cardW + 8)
                local cy = startY + (row - 1) * (cardH + 6)
                addBtn(cx, cy, cardW, cardH, title, color, 0xFFFFFF, function()
                    self.currentView = targetView
                    self.isMenuOpen = false
                end)
            end

            addMenuCard(1, 1, "1. OVERVIEW", "OVERVIEW", 0x1B3A60)
            addMenuCard(2, 1, "2. PFD (FLIGHT)", "PFD", 0x145A32)
            addMenuCard(1, 2, "3. ECAM (ENGINES)", "ECAM", 0x78281F)
            addMenuCard(2, 2, "4. NAV (PRESETS)", "NAV", 0x117864)

            local btmW = sw - 12
            local cy = startY + 2 * (cardH + 6)
            addBtn(6, cy, btmW, cardH, "5. SYS (DIAGNOSTICS & TURTLES)", 0x512E5F, 0xFFFFFF, function()
                self.currentView = "SYS"
                self.isMenuOpen = false
            end)

        elseif self.currentView == "OVERVIEW" then
            local currAlt = FlightCore.altiSensor and FlightCore.altiSensor.getHeight() or 0
            local currVspeed = FlightCore.altiSensor and FlightCore.altiSensor.getVerticalSpeed() or 0
            local currPitch, currRoll = FlightCore.getGimbalData()

            local instY = headerH + 6
            local instH = math.floor((sh - headerH) * 0.52)
            local leftW = math.max(130, math.floor(sw * 0.35))
            local pfdX = 6

            sFR(pfdX, instY, leftW, instH, 0x141A26)
            sR(pfdX, instY, leftW, instH, 0x283850)
            sTxt(pfdX + 8, instY + 6, "PRIMARY FLIGHT DISPLAY", 0xAAD2E6, 1)

            local gR = math.min(32, math.floor(instH * 0.28))
            local gCX = pfdX + math.floor(leftW * 0.26)
            local gCY = instY + math.floor(instH * 0.54)
            for dy = -gR, gR do
                local dx = math.floor(math.sqrt(math.max(0, gR*gR - dy*dy)) + 0.5)
                sL(gCX - dx, gCY + dy, gCX + dx, gCY + dy, 0x1E2837)
            end
            sArc(gCX, gCY, gR, 0, 360, 10, 0x50AAF0)
            local altText = string.format("%.0f", currAlt)
            local altSize = (gR >= 26) and 2 or 1
            sTxt(gCX - math.floor(#altText * 3 * altSize), gCY - math.floor(3.5 * altSize), altText, 0xFFFFFF, altSize)
            sTxt(gCX - 16, gCY + math.floor(gR * 0.40), "ALT(M)", 0xA0CDF0, 1)

            local textX = pfdX + math.floor(leftW * 0.52)
            local rowSpacing = math.floor((instH - 24) / 5)
            local textSize = (instH >= 130 and sw >= 360) and 2 or 1

            sTxt(textX, instY + 16, string.format("TGT: %.0fm", FlightCore.state.targetAlt), 0x50E6FF, textSize)
            sTxt(textX, instY + 16 + rowSpacing, string.format("V.S: %+.2f", currVspeed), 0xFFCD4B, textSize)
            local mCol = (FlightCore.state.mode == "HOLD_ALT") and 0x50FF78 or (FlightCore.state.mode == "CALIBRATING" and 0x50C8FF or 0xFF5050)
            sTxt(textX, instY + 16 + rowSpacing * 2, string.format("MOD: %s", FlightCore.state.mode:sub(1,7)), mCol, textSize)
            sTxt(textX, instY + 16 + rowSpacing * 3, string.format("P:%+3.0f R:%+3.0f", currPitch, currRoll), 0xB4DCFF, 1)
            sTxt(textX, instY + 16 + rowSpacing * 4, FlightCore.gimbalAvailable and "GIMBAL: OK" or "GIMBAL: N/A", FlightCore.gimbalAvailable and 0x50FF78 or 0xFF5050, 1)

            local ecamX = pfdX + leftW + 6
            local ecamW = sw - ecamX - 6
            sFR(ecamX, instY, ecamW, instH, 0x121822)
            sR(ecamX, instY, ecamW, instH, 0x283850)

            local slotW = math.floor((ecamW - 8) / 4)
            local slots = {"FL", "FR", "BL", "BR"}
            local dialR = math.min(math.floor(slotW * 0.35), math.floor(instH * 0.28))
            local engCenterY = instY + math.floor(instH * 0.44)

            for i, slot in ipairs(slots) do
                local engCenterX = ecamX + 4 + math.floor((i - 0.5) * slotW)
                local node = FlightCore.engines[slot]
                local lbl = node and (node.label or node.name or slot) or slot
                if #lbl > 5 then lbl = lbl:sub(1, 5) end

                sTxt(engCenterX - math.floor(#lbl * 3), engCenterY - dialR - 12, lbl, 0xC8E6FF, 1)
                sArc(engCenterX, engCenterY, dialR, 210, -30, 10, 0x8CA0B4)
                sArc(engCenterX, engCenterY, dialR + 1, 210, -30, 10, 0x5A6E82)
                sArc(engCenterX, engCenterY, dialR, 10, -30, 10, 0xFF3232)

                local ratio = math.min(1.0, math.max(0.0, FlightCore.virtualOutputs[slot] / 15.0))
                local nRad = math.rad(210 - ratio * 240)
                local nx = math.floor(engCenterX + (dialR - 2) * math.cos(nRad) + 0.5)
                local ny = math.floor(engCenterY - (dialR - 2) * math.sin(nRad) + 0.5)
                sLS(engCenterX, engCenterY, nx, ny, 0x32FF64)
                sFR(engCenterX - 2, engCenterY - 2, 5, 5, 0xC8DCF0)

                local boxW = math.max(36, math.floor(dialR * 1.55))
                local boxH = 14
                local boxX = engCenterX - math.floor(boxW / 2)
                local boxY = engCenterY + math.floor(dialR * 0.28)
                sFR(boxX, boxY, boxW, boxH, 0x0C121C)
                sR(boxX, boxY, boxW, boxH, 0x2896C8)

                local valStr = string.format("%4.1f", FlightCore.virtualOutputs[slot])
                sTxt(boxX + math.floor((boxW - #valStr * 6) / 2), boxY + 4, valStr, 0x46FF78, 1)

                local sigStr = string.format("%d/15", FlightCore.engineOutputs[slot] or 0)
                sTxt(engCenterX - math.floor(#sigStr * 3), boxY + boxH + 3, sigStr, 0x96BEE1, 1)
            end

            local statY = instY + instH + 6
            local statH = math.max(18, math.floor(sh * 0.06))
            sFR(pfdX, statY, sw - 12, statH, 0x1E2432)
            sR(pfdX, statY, sw - 12, statH, 0x3C4B64)
            sTxt(pfdX + 8, statY + math.floor((statH - 7) / 2), string.format("STATUS: %s", FlightCore.state.statusMsg), 0x50E6FF, 1)

            local btnAreaY = statY + statH + 6
            local btnAreaH = sh - btnAreaY - 6
            local rowH = math.max(16, math.floor((btnAreaH - 12) / 3))

            local bY1 = btnAreaY
            local bW1 = math.floor((sw - 12 - 16) / 5)
            addBtn(pfdX, bY1, bW1, rowH, "+10m", 0x1B5E20, 0xE8F5E9, function() FlightCore.adjustTargetAlt(10) end)
            addBtn(pfdX + bW1 + 4, bY1, bW1, rowH, "+1m", 0x2E7D32, 0xE8F5E9, function() FlightCore.adjustTargetAlt(1) end)
            addBtn(pfdX + (bW1 + 4)*2, bY1, bW1, rowH, "-1m", 0xE65100, 0xFFF3E0, function() FlightCore.adjustTargetAlt(-1) end)
            addBtn(pfdX + (bW1 + 4)*3, bY1, bW1, rowH, "-10m", 0xC62828, 0xFFEBEE, function() FlightCore.adjustTargetAlt(-10) end)
            addBtn(pfdX + (bW1 + 4)*4, bY1, bW1, rowH, "SET CURR", 0x00838F, 0xE0F7FA, function() FlightCore.lockCurrentAlt() end)

            local bY2 = bY1 + rowH + 6
            local bW2 = math.floor((sw - 12 - 12) / 4)
            addBtn(pfdX, bY2, bW2, rowH, "BASE +", 0x00695C, 0xE0F2F1, function() FlightCore.adjustBaseThrottle(1.0) end)
            addBtn(pfdX + bW2 + 4, bY2, bW2, rowH, "BASE -", 0x37474F, 0xECEFF1, function() FlightCore.adjustBaseThrottle(-1.0) end)
            addBtn(pfdX + (bW2 + 4)*2, bY2, bW2, rowH, "RE-SCAN", 0x1565C0, 0xE3F2FD, function() FlightCore.scanQuadTurtles(); FlightCore.state.statusMsg="Hardware Re-scanned!" end)
            addBtn(pfdX + (bW2 + 4)*3, bY2, bW2, rowH, "CALIBRATE", 0x6A1B9A, 0xF3E5F5, function() FlightCore.startCalibration() end)

            local bY3 = bY2 + rowH + 6
            local bW3 = math.floor((sw - 12 - 6) / 2)
            local bH3 = math.max(rowH, sh - bY3 - 6)
            local holdBg = (FlightCore.state.mode == "HOLD_ALT") and 0x2E7D32 or 0x1B5E20
            addBtn(pfdX, bY3, bW3, bH3, " [ HOLD ALT ] ", holdBg, 0xFFFFFF, function() FlightCore.holdAltitude() end)
            local stopBg = (FlightCore.state.mode == "IDLE") and 0xC62828 or 0xB71C1C
            addBtn(pfdX + bW3 + 6, bY3, bW3, bH3, " [ STOP / IDLE ] ", stopBg, 0xFFFFFF, function() FlightCore.stopEngines() end)

        elseif self.currentView == "PFD" then
            local currAlt = FlightCore.altiSensor and FlightCore.altiSensor.getHeight() or 0
            local currVspeed = FlightCore.altiSensor and FlightCore.altiSensor.getVerticalSpeed() or 0
            local currPitch, currRoll = FlightCore.getGimbalData()

            local mainH = math.floor((sh - headerH) * 0.65)
            local mainY = headerH + 6

            local leftW = math.floor(sw * 0.52)
            sFR(6, mainY, leftW, mainH, 0x141A26)
            sR(6, mainY, leftW, mainH, 0x283850)

            local hCX = 6 + math.floor(leftW / 2)
            local hCY = mainY + math.floor(mainH / 2)
            local hR = math.min(math.floor(leftW * 0.40), math.floor(mainH * 0.40))
            sArc(hCX, hCY, hR, 0, 360, 10, 0x50AAF0)

            local rollRad = math.rad(-currRoll)
            local lx1 = hCX - math.floor(hR * 0.85 * math.cos(rollRad))
            local ly1 = hCY - math.floor(hR * 0.85 * math.sin(rollRad))
            local lx2 = hCX + math.floor(hR * 0.85 * math.cos(rollRad))
            local ly2 = hCY + math.floor(hR * 0.85 * math.sin(rollRad))
            sLS(lx1, ly1, lx2, ly2, 0xFFD700)
            sFR(hCX - 2, hCY - 2, 5, 5, 0xFF5050)
            sTxt(12, mainY + mainH - 14, string.format("P:%+4.1f*  R:%+4.1f*", currPitch, currRoll), 0xC8E6FF, 1)

            local rightX = 6 + leftW + 6
            local rightW = sw - rightX - 6
            sFR(rightX, mainY, rightW, mainH, 0x18202E)
            sR(rightX, mainY, rightW, mainH, 0x283850)

            sTxt(rightX + 8, mainY + 8, "CURRENT ALTITUDE", 0x8CA0B4, 1)
            local altStr = string.format("%.1f m", currAlt)
            sTxt(rightX + 8, mainY + 22, altStr, 0xFFFFFF, 2)

            sTxt(rightX + 8, mainY + 44, string.format("TARGET: %.0f m", FlightCore.state.targetAlt), 0x50E6FF, 1)
            sTxt(rightX + 8, mainY + 58, string.format("V.S: %+.2f m/s", currVspeed), 0xFFCD4B, 1)
            sTxt(rightX + 8, mainY + 72, string.format("STATUS: %s", FlightCore.state.statusMsg:sub(1,18)), 0x50FF78, 1)

            local bY = mainY + mainH + 6
            local bH = sh - bY - 6
            local bW = math.floor((sw - 12 - 20) / 6)
            addBtn(6, bY, bW, bH, "+10m", 0x1B5E20, 0xE8F5E9, function() FlightCore.adjustTargetAlt(10) end)
            addBtn(6 + (bW+4), bY, bW, bH, "+1m", 0x2E7D32, 0xE8F5E9, function() FlightCore.adjustTargetAlt(1) end)
            addBtn(6 + (bW+4)*2, bY, bW, bH, "-1m", 0xE65100, 0xFFF3E0, function() FlightCore.adjustTargetAlt(-1) end)
            addBtn(6 + (bW+4)*3, bY, bW, bH, "-10m", 0xC62828, 0xFFEBEE, function() FlightCore.adjustTargetAlt(-10) end)
            local holdBg = (FlightCore.state.mode == "HOLD_ALT") and 0x2E7D32 or 0x1B5E20
            addBtn(6 + (bW+4)*4, bY, bW, bH, "HOLD", holdBg, 0xFFFFFF, function() FlightCore.holdAltitude() end)
            local stopBg = (FlightCore.state.mode == "IDLE") and 0xC62828 or 0xB71C1C
            addBtn(6 + (bW+4)*5, bY, bW, bH, "STOP", stopBg, 0xFFFFFF, function() FlightCore.stopEngines() end)

        elseif self.currentView == "ECAM" then
            local instY = headerH + 6
            local instH = math.floor((sh - headerH) * 0.60)
            sFR(6, instY, sw - 12, instH, 0x121822)
            sR(6, instY, sw - 12, instH, 0x283850)

            local slotW = math.floor((sw - 20) / 4)
            local slots = {"FL", "FR", "BL", "BR"}
            local dialR = math.min(math.floor(slotW * 0.38), math.floor(instH * 0.32))
            local engCenterY = instY + math.floor(instH * 0.44)

            for i, slot in ipairs(slots) do
                local engCenterX = 8 + math.floor((i - 0.5) * slotW)
                local node = FlightCore.engines[slot]
                local lbl = node and (node.label or node.name or slot) or slot
                if #lbl > 5 then lbl = lbl:sub(1, 5) end

                sTxt(engCenterX - math.floor(#lbl * 3), engCenterY - dialR - 12, lbl, 0xC8E6FF, 1)
                sArc(engCenterX, engCenterY, dialR, 210, -30, 10, 0x8CA0B4)
                sArc(engCenterX, engCenterY, dialR, 10, -30, 10, 0xFF3232)

                local ratio = math.min(1.0, math.max(0.0, FlightCore.virtualOutputs[slot] / 15.0))
                local nRad = math.rad(210 - ratio * 240)
                local nx = math.floor(engCenterX + (dialR - 2) * math.cos(nRad) + 0.5)
                local ny = math.floor(engCenterY - (dialR - 2) * math.sin(nRad) + 0.5)
                sLS(engCenterX, engCenterY, nx, ny, 0x32FF64)

                local boxW = math.max(36, math.floor(dialR * 1.55))
                local boxH = 14
                local boxX = engCenterX - math.floor(boxW / 2)
                local boxY = engCenterY + math.floor(dialR * 0.28)
                sFR(boxX, boxY, boxW, boxH, 0x0C121C)
                sR(boxX, boxY, boxW, boxH, 0x2896C8)

                local valStr = string.format("%4.1f", FlightCore.virtualOutputs[slot])
                sTxt(boxX + math.floor((boxW - #valStr * 6) / 2), boxY + 4, valStr, 0x46FF78, 1)

                local sigStr = string.format("%d/15", FlightCore.engineOutputs[slot] or 0)
                sTxt(engCenterX - math.floor(#sigStr * 3), boxY + boxH + 3, sigStr, 0x96BEE1, 1)
            end

            local statY = instY + instH + 6
            local statH = math.max(18, math.floor(sh * 0.06))
            sFR(6, statY, sw - 12, statH, 0x1E2432)
            sR(6, statY, sw - 12, statH, 0x3C4B64)
            sTxt(12, statY + 4, string.format("BASE: %4.2f/15  |  STATUS: %s", FlightCore.state.baseThrottle, FlightCore.state.statusMsg:sub(1,24)), 0x50E6FF, 1)

            local bY = statY + statH + 6
            local bH = sh - bY - 6
            local bW = math.floor((sw - 12 - 20) / 6)
            addBtn(6, bY, bW, bH, "BASE +", 0x00695C, 0xE0F2F1, function() FlightCore.adjustBaseThrottle(1.0) end)
            addBtn(6 + (bW+4), bY, bW, bH, "BASE -", 0x37474F, 0xECEFF1, function() FlightCore.adjustBaseThrottle(-1.0) end)
            addBtn(6 + (bW+4)*2, bY, bW, bH, "RE-SCAN", 0x1565C0, 0xE3F2FD, function() FlightCore.scanQuadTurtles(); FlightCore.state.statusMsg="Hardware Re-scanned!" end)
            addBtn(6 + (bW+4)*3, bY, bW, bH, "CALIB 200m", 0x6A1B9A, 0xF3E5F5, function() FlightCore.startCalibration() end)
            local holdBg = (FlightCore.state.mode == "HOLD_ALT") and 0x2E7D32 or 0x1B5E20
            addBtn(6 + (bW+4)*4, bY, bW, bH, "HOLD", holdBg, 0xFFFFFF, function() FlightCore.holdAltitude() end)
            local stopBg = (FlightCore.state.mode == "IDLE") and 0xC62828 or 0xB71C1C
            addBtn(6 + (bW+4)*5, bY, bW, bH, "STOP", stopBg, 0xFFFFFF, function() FlightCore.stopEngines() end)

        elseif self.currentView == "NAV" then
            local currAlt = FlightCore.altiSensor and FlightCore.altiSensor.getHeight() or 0
            local currVspeed = FlightCore.altiSensor and FlightCore.altiSensor.getVerticalSpeed() or 0

            local statH = 24
            sFR(6, headerH + 6, sw - 12, statH, 0x141E28)
            sR(6, headerH + 6, sw - 12, statH, 0x284864)
            sTxt(12, headerH + 12, string.format("ALT: %.1fm  ->  TARGET: %.0fm  (V.S: %+.2f)", currAlt, FlightCore.state.targetAlt, currVspeed), 0x50E6FF, 1)

            local gridY = headerH + 6 + statH + 6
            local btnAreaH = sh - gridY - 6
            local rowH = math.floor((btnAreaH - 12) / 3)
            local colW = math.floor((sw - 12 - 8) / 2)

            addBtn(6, gridY, colW, rowH, "[ 0m LANDING ]", 0xA93226, 0xFFFFFF, function() FlightCore.setTargetAlt(0) end)
            addBtn(6 + colW + 8, gridY, colW, rowH, "[ 80m TREETOP ]", 0x1E8449, 0xFFFFFF, function() FlightCore.setTargetAlt(80) end)

            addBtn(6, gridY + rowH + 6, colW, rowH, "[ 150m CRUISE ]", 0x2471A3, 0xFFFFFF, function() FlightCore.setTargetAlt(150) end)
            addBtn(6 + colW + 8, gridY + rowH + 6, colW, rowH, "[ 200m CALIBRATE ]", 0x7D3C98, 0xFFFFFF, function() FlightCore.setTargetAlt(200) end)

            addBtn(6, gridY + (rowH + 6)*2, colW, rowH, "[ 300m HIGH-ALT ]", 0x17A589, 0xFFFFFF, function() FlightCore.setTargetAlt(300) end)
            addBtn(6 + colW + 8, gridY + (rowH + 6)*2, colW, rowH, "[ LOCK CURRENT ALT ]", 0xB7950B, 0xFFFFFF, function() FlightCore.lockCurrentAlt() end)

        elseif self.currentView == "SYS" then
            local cardW = math.floor((sw - 20) / 2)
            local cardH = math.floor((sh - headerH - 45) / 2)
            local startY = headerH + 6

            local slots = {
                {slot="FL", col=1, row=1, name="Front-Left Turtle"},
                {slot="FR", col=2, row=1, name="Front-Right Turtle"},
                {slot="BL", col=1, row=2, name="Back-Left Turtle"},
                {slot="BR", col=2, row=2, name="Back-Right Turtle"}
            }

            for _, s in ipairs(slots) do
                local cx = 6 + (s.col - 1) * (cardW + 8)
                local cy = startY + (s.row - 1) * (cardH + 6)
                local node = FlightCore.engines[s.slot]
                local online = node ~= nil
                sFR(cx, cy, cardW, cardH, online and 0x14281E or 0x281414)
                sR(cx, cy, cardW, cardH, online and 0x28643C or 0x642828)

                sTxt(cx + 6, cy + 6, string.format("[%s] %s", s.slot, s.name), 0xF0F5FF, 1)
                sTxt(cx + 6, cy + 20, string.format("STATUS: %s", online and "ONLINE" or "OFFLINE"), online and 0x50FF78 or 0xFF5050, 1)
                local lbl = node and (node.label or node.name) or "N/A"
                sTxt(cx + 6, cy + 34, string.format("NODE: %s | PWM: %d", lbl:sub(1,7), FlightCore.engineOutputs[s.slot]), 0x8CA0B4, 1)
            end

            local btmY = startY + cardH * 2 + 10
            local btmW = math.floor((sw - 16) / 2)
            addBtn(6, btmY, btmW, 24, "RE-SCAN HARDWARE", 0x1565C0, 0xFFFFFF, function() FlightCore.scanQuadTurtles(); FlightCore.state.statusMsg="Hardware Re-scanned!" end)
            addBtn(6 + btmW + 4, btmY, btmW, 24, "STOP ALL ENGINES", 0xC62828, 0xFFFFFF, function() FlightCore.stopEngines() end)
        end

        -- 繪製所有按鈕
        local btnTextSize = 1
        for _, btn in ipairs(self.buttons) do
            sFR(btn.x, btn.y, btn.w, btn.h, btn.bg)
            sR(btn.x, btn.y, btn.w, btn.h, 0x8CA0B4)
            local tx = btn.x + math.max(2, math.floor((btn.w - #btn.text * 6 * btnTextSize) / 2))
            local ty = btn.y + math.floor((btn.h - 7 * btnTextSize) / 2)
            sTxt(tx, ty, btn.text, btn.fg, btnTextSize)
        end

        pcall(function() if self.gpu.sync then self.gpu.sync() end end)
    end,
    handleEvent = function(self, event, p1, p2, p3, p4)
        local clickX, clickY = nil, nil
        if event == "tm_monitor_touch" then
            if type(p1) == "number" and type(p2) == "number" then clickX, clickY = p1, p2
            elseif type(p2) == "number" and type(p3) == "number" then clickX, clickY = p2, p3 end
        elseif event == "monitor_touch" then
            if type(p2) == "number" and type(p3) == "number" then
                local mon = peripheral.find("monitor")
                local mw, mh = (mon and mon.getSize()) or term.getSize()
                clickX = math.floor(((p2 - 0.5) / mw) * (self.screenW or 320))
                clickY = math.floor(((p3 - 0.5) / mh) * (self.screenH or 240))
            end
        elseif event == "mouse_click" then
            if type(p2) == "number" and type(p3) == "number" then
                local tw, th = term.getSize()
                clickX = math.floor(((p2 - 0.5) / tw) * (self.screenW or 320))
                clickY = math.floor(((p3 - 0.5) / th) * (self.screenH or 240))
            end
        end

        if clickX and clickY then
            for _, btn in ipairs(self.buttons) do
                if clickX >= btn.x and clickX <= btn.x + btn.w and clickY >= btn.y and clickY <= btn.y + btn.h then
                    pcall(btn.action)
                    self:draw()
                    break
                end
            end
        end
    end
}

-- --------------------------------------------------------
-- 2.3 驅動 C: CC: Tweaked 原生螢幕 / 終端機自適應文字儀表
-- --------------------------------------------------------
Drivers.normal = {
    name = "VTOL Airship " .. VERSION .. " (normal)",
    shortTag = "normal",
    currentView = "OVERVIEW",
    isMenuOpen = false,
    isAvailable = function()
        return true
    end,
    init = function(self)
        self.mon = peripheral.find("monitor")
        self.display = self.mon or term.current()
        if self.mon then
            self.mon.setTextScale(1.0)
            local mw, mh = self.mon.getSize()
            if mw < 38 or mh < 14 then self.mon.setTextScale(0.5) end
        end
        if self.display.setPaletteColor then
            self.display.setPaletteColor(colors.black, 0x111622)
            self.display.setPaletteColor(colors.gray, 0x1c2438)
            self.display.setPaletteColor(colors.lightGray, 0x5a6988)
            self.display.setPaletteColor(colors.blue, 0x2266cc)
            self.display.setPaletteColor(colors.cyan, 0x00d2ff)
            self.display.setPaletteColor(colors.lime, 0x2ed573)
            self.display.setPaletteColor(colors.green, 0x1e824c)
            self.display.setPaletteColor(colors.yellow, 0xffa502)
            self.display.setPaletteColor(colors.red, 0xff4757)
            self.display.setPaletteColor(colors.white, 0xf1f2f6)
        end
        self.buttons = {}
        self.currentView = "OVERVIEW"
        self.isMenuOpen = false
    end,
    safeBlit = function(self, x, y, text, fgChar, bgChar)
        text = tostring(text or "")
        local len = #text
        if len == 0 then return end
        self.display.setCursorPos(x, y)
        local fg = tostring(fgChar or "0")
        if #fg == 1 then fg = string.rep(fg, len)
        elseif #fg < len then fg = fg .. string.rep("0", len - #fg)
        elseif #fg > len then fg = fg:sub(1, len) end
        local bg = tostring(bgChar or "f")
        if #bg == 1 then bg = string.rep(bg, len)
        elseif #bg < len then bg = bg .. string.rep(bg:sub(#bg, #bg), len - #bg)
        elseif #bg > len then bg = bg:sub(1, len) end
        self.display.blit(text, fg, bg)
    end,
    addBtn = function(self, x, y, w, h, text, bg, fg, act)
        table.insert(self.buttons, {x=x, y=y, w=w, h=h, text=text, bg=bg, fg=fg, action=act})
    end,
    draw = function(self)
        local w, h = self.display.getSize()
        self.display.setBackgroundColor(colors.black)
        self.display.clear()
        self.buttons = {}

        -- 1. 頂部標題列與右上角 MENU 按鈕
        local viewTag = self.isMenuOpen and "MENU" or self.currentView
        local titleText = string.format(" VTOL %s [%s]", VERSION, viewTag)
        local menuBtnW = (w >= 38) and 8 or 6
        local menuBtnX = w - menuBtnW + 1

        local headerPad = w - #titleText - menuBtnW
        local header = titleText .. string.rep(" ", math.max(0, headerPad))
        self:safeBlit(1, 1, header:sub(1, w - menuBtnW), "0", "b")

        if self.isMenuOpen then
            self:addBtn(menuBtnX, 1, menuBtnW, 1, "[CLOSE]", "e", "0", function() self.isMenuOpen = false end)
        else
            self:addBtn(menuBtnX, 1, menuBtnW, 1, " [MENU] ", "b", "0", function() self.isMenuOpen = true end)
        end

        -- 2. 視圖分流路由
        if self.isMenuOpen then
            local cardW = math.floor((w - 3) / 2)
            local cardH = math.max(2, math.floor((h - 5) / 3))
            local startY = 3

            local function addMenuCard(col, row, title, targetView, bgCol)
                local cx = 2 + (col - 1) * (cardW + 1)
                local cy = startY + (row - 1) * (cardH + 1)
                self:addBtn(cx, cy, cardW, cardH, title, bgCol, "0", function()
                    self.currentView = targetView
                    self.isMenuOpen = false
                end)
            end

            addMenuCard(1, 1, "1. OVERVIEW", "OVERVIEW", "3")
            addMenuCard(2, 1, "2. PFD FLIGHT", "PFD", "5")
            addMenuCard(1, 2, "3. ECAM ENGINES", "ECAM", "e")
            addMenuCard(2, 2, "4. NAV PRESETS", "NAV", "9")

            local btmW = w - 2
            local cy = startY + 2 * (cardH + 1)
            self:addBtn(2, cy, btmW, cardH, "5. SYS DIAGNOSTICS & TURTLES", "a", "0", function()
                self.currentView = "SYS"
                self.isMenuOpen = false
            end)

        elseif self.currentView == "OVERVIEW" then
            local currAlt = FlightCore.altiSensor and FlightCore.altiSensor.getHeight() or 0
            local currVspeed = FlightCore.altiSensor and FlightCore.altiSensor.getVerticalSpeed() or 0
            local currPitch, currRoll = FlightCore.getGimbalData()

            local instY = 3
            local availH = h - instY
            local cardH = math.max(6, math.floor(availH * 0.48))
            local leftW = math.max(20, math.floor(w * 0.35))
            local rightX = leftW + 3
            local rightW = w - rightX

            for y = instY, instY + cardH - 1 do
                self:safeBlit(2, y, string.rep(" ", leftW), "0", "8")
            end
            self:safeBlit(3, instY, "=== PRIMARY FLIGHT ===", "0", "8")
            self:safeBlit(3, instY + 1, string.format("ALT: %6.1f m", currAlt), "0", "8")
            self:safeBlit(3, instY + 2, string.format("TGT: %6.1f m", FlightCore.state.targetAlt), "9", "8")
            self:safeBlit(3, instY + 3, string.format("V.S: %+6.2f m/s", currVspeed), "4", "8")
            local mCol = (FlightCore.state.mode == "HOLD_ALT") and "5" or (FlightCore.state.mode == "CALIBRATING" and "b" or "e")
            self:safeBlit(3, instY + 4, string.format("MOD: %-10s", FlightCore.state.mode:sub(1,10)), mCol, "8")
            if cardH >= 6 then
                self:safeBlit(3, instY + 5, string.format("ATT: P%+3.0f* R%+3.0f*", currPitch, currRoll), "0", "8")
            end

            local dialW = math.max(7, math.floor(rightW / 4))
            local slots = {"FL", "FR", "BL", "BR"}
            for i, slot in ipairs(slots) do
                local dx = rightX + (i - 1) * dialW
                local node = FlightCore.engines[slot]
                local lbl = node and (node.label or node.name or slot) or slot
                local val = FlightCore.virtualOutputs[slot] or 0.0
                local sig = FlightCore.engineOutputs[slot] or 0

                for row = 0, cardH - 1 do
                    self:safeBlit(dx, instY + row, string.rep(" ", dialW - 1), "0", "7")
                end
                self:safeBlit(dx, instY, string.format("[%s]", lbl:sub(1, 3)), "9", "8")
                self:safeBlit(dx, instY + 1, string.format("%4.1f", val), "5", "7")
                self:safeBlit(dx, instY + 2, string.format("%2d/15", sig), "3", "7")
            end

            local statY = instY + cardH + 1
            local statText = string.format(" STATUS: %s", FlightCore.state.statusMsg)
            local statPad = w - #statText
            self:safeBlit(1, statY, (statText .. string.rep(" ", math.max(0, statPad))):sub(1, w), "0", "7")

            local btnAreaY = statY + 2
            local remH = h - btnAreaY + 1
            local bH = math.max(1, math.floor(remH / 3))

            local bY1 = btnAreaY
            local bW1 = math.floor((w - 6) / 5)
            self:addBtn(2, bY1, bW1, bH, "+10m", "5", "0", function() FlightCore.adjustTargetAlt(10) end)
            self:addBtn(3 + bW1, bY1, bW1, bH, "+1m", "5", "0", function() FlightCore.adjustTargetAlt(1) end)
            self:addBtn(4 + bW1*2, bY1, bW1, bH, "-1m", "4", "0", function() FlightCore.adjustTargetAlt(-1) end)
            self:addBtn(5 + bW1*3, bY1, bW1, bH, "-10m", "e", "0", function() FlightCore.adjustTargetAlt(-10) end)
            self:addBtn(6 + bW1*4, bY1, bW1, bH, "SET CURR", "3", "0", function() FlightCore.lockCurrentAlt() end)

            local bY2 = bY1 + bH + 1
            local bW2 = math.floor((w - 5) / 4)
            self:addBtn(2, bY2, bW2, bH, "BASE +", "3", "0", function() FlightCore.adjustBaseThrottle(1.0) end)
            self:addBtn(3 + bW2, bY2, bW2, bH, "BASE -", "9", "0", function() FlightCore.adjustBaseThrottle(-1.0) end)
            self:addBtn(4 + bW2*2, bY2, bW2, bH, "RE-SCAN", "b", "0", function() FlightCore.scanQuadTurtles(); FlightCore.state.statusMsg="Hardware Re-scanned!" end)
            self:addBtn(5 + bW2*3, bY2, bW2, bH, "CALIBRATE", "a", "0", function() FlightCore.startCalibration() end)

            local bY3 = bY2 + bH + 1
            local mainBW = math.floor((w - 3) / 2)
            local mainBH = math.max(bH, h - bY3 + 1)
            local holdBg = (FlightCore.state.mode == "HOLD_ALT") and "5" or "d"
            self:addBtn(2, bY3, mainBW, mainBH, " [ HOLD ALT ] ", holdBg, "0", function() FlightCore.holdAltitude() end)
            local stopBg = (FlightCore.state.mode == "IDLE") and "e" or "c"
            self:addBtn(3 + mainBW, bY3, mainBW, mainBH, " [ STOP / IDLE ] ", stopBg, "0", function() FlightCore.stopEngines() end)

        elseif self.currentView == "PFD" then
            local currAlt = FlightCore.altiSensor and FlightCore.altiSensor.getHeight() or 0
            local currVspeed = FlightCore.altiSensor and FlightCore.altiSensor.getVerticalSpeed() or 0
            local currPitch, currRoll = FlightCore.getGimbalData()

            self:safeBlit(2, 3, string.format("CURRENT ALTITUDE : %7.1f m", currAlt), "0", "5")
            self:safeBlit(2, 5, string.format("TARGET ALTITUDE  : %7.1f m", FlightCore.state.targetAlt), "0", "b")
            self:safeBlit(2, 7, string.format("VERTICAL SPEED   : %+7.2f m/s", currVspeed), "0", "4")
            self:safeBlit(2, 9, string.format("ATTITUDE GYRO    : P%+4.1f*  R%+4.1f*", currPitch, currRoll), "0", "8")
            self:safeBlit(2, 11, string.format("SYSTEM STATUS    : %s", FlightCore.state.statusMsg:sub(1, w - 20)), "0", "7")

            local bY = h - 3
            local bW = math.floor((w - 7) / 6)
            self:addBtn(2, bY, bW, 2, "+10m", "5", "0", function() FlightCore.adjustTargetAlt(10) end)
            self:addBtn(3 + bW, bY, bW, 2, "+1m", "5", "0", function() FlightCore.adjustTargetAlt(1) end)
            self:addBtn(4 + bW*2, bY, bW, 2, "-1m", "4", "0", function() FlightCore.adjustTargetAlt(-1) end)
            self:addBtn(5 + bW*3, bY, bW, 2, "-10m", "e", "0", function() FlightCore.adjustTargetAlt(-10) end)
            self:addBtn(6 + bW*4, bY, bW, 2, "HOLD", "5", "0", function() FlightCore.holdAltitude() end)
            self:addBtn(7 + bW*5, bY, bW, 2, "STOP", "e", "0", function() FlightCore.stopEngines() end)

        elseif self.currentView == "ECAM" then
            local slots = {"FL", "FR", "BL", "BR"}
            local colW = math.floor((w - 5) / 4)
            for i, slot in ipairs(slots) do
                local dx = 2 + (i - 1) * (colW + 1)
                local node = FlightCore.engines[slot]
                local lbl = node and (node.label or node.name or slot) or slot
                local val = FlightCore.virtualOutputs[slot] or 0.0
                local sig = FlightCore.engineOutputs[slot] or 0

                for r = 3, 9 do
                    self:safeBlit(dx, r, string.rep(" ", colW), "0", "7")
                end
                self:safeBlit(dx, 3, string.format(" [%s] ", lbl:sub(1, colW - 4)), "9", "8")
                self:safeBlit(dx, 5, string.format("THR: %4.1f", val), "5", "7")
                self:safeBlit(dx, 7, string.format("PWM: %2d", sig), "3", "7")
            end

            self:safeBlit(2, 11, string.format("BASE THRUST: %4.2f/15  |  STATUS: %s", FlightCore.state.baseThrottle, FlightCore.state.statusMsg:sub(1, w - 30)), "0", "b")

            local bY = h - 3
            local bW = math.floor((w - 7) / 6)
            self:addBtn(2, bY, bW, 2, "BASE +", "3", "0", function() FlightCore.adjustBaseThrottle(1.0) end)
            self:addBtn(3 + bW, bY, bW, 2, "BASE -", "9", "0", function() FlightCore.adjustBaseThrottle(-1.0) end)
            self:addBtn(4 + bW*2, bY, bW, 2, "RE-SCAN", "b", "0", function() FlightCore.scanQuadTurtles(); FlightCore.state.statusMsg="Hardware Re-scanned!" end)
            self:addBtn(5 + bW*3, bY, bW, 2, "CALIB 200", "a", "0", function() FlightCore.startCalibration() end)
            self:addBtn(6 + bW*4, bY, bW, 2, "HOLD", "5", "0", function() FlightCore.holdAltitude() end)
            self:addBtn(7 + bW*5, bY, bW, 2, "STOP", "e", "0", function() FlightCore.stopEngines() end)

        elseif self.currentView == "NAV" then
            local currAlt = FlightCore.altiSensor and FlightCore.altiSensor.getHeight() or 0
            self:safeBlit(2, 3, string.format("ALT: %6.1fm  ->  TARGET: %6.1fm", currAlt, FlightCore.state.targetAlt), "0", "b")

            local gridY = 5
            local colW = math.floor((w - 3) / 2)
            local rowH = math.max(2, math.floor((h - gridY - 2) / 3))

            self:addBtn(2, gridY, colW, rowH, "[ 0m LANDING ]", "e", "0", function() FlightCore.setTargetAlt(0) end)
            self:addBtn(3 + colW, gridY, colW, rowH, "[ 80m TREETOP ]", "5", "0", function() FlightCore.setTargetAlt(80) end)

            self:addBtn(2, gridY + rowH + 1, colW, rowH, "[ 150m CRUISE ]", "3", "0", function() FlightCore.setTargetAlt(150) end)
            self:addBtn(3 + colW, gridY + rowH + 1, colW, rowH, "[ 200m CALIBRATE ]", "a", "0", function() FlightCore.setTargetAlt(200) end)

            self:addBtn(2, gridY + (rowH + 1)*2, colW, rowH, "[ 300m HIGH-ALT ]", "b", "0", function() FlightCore.setTargetAlt(300) end)
            self:addBtn(3 + colW, gridY + (rowH + 1)*2, colW, rowH, "[ LOCK CURRENT ]", "4", "0", function() FlightCore.lockCurrentAlt() end)

        elseif self.currentView == "SYS" then
            local slots = {
                {slot="FL", col=1, row=1, name="Front-Left Turtle"},
                {slot="FR", col=2, row=1, name="Front-Right Turtle"},
                {slot="BL", col=1, row=2, name="Back-Left Turtle"},
                {slot="BR", col=2, row=2, name="Back-Right Turtle"}
            }
            local cardW = math.floor((w - 3) / 2)
            local cardH = 3

            for _, s in ipairs(slots) do
                local cx = 2 + (s.col - 1) * (cardW + 1)
                local cy = 3 + (s.row - 1) * (cardH + 1)
                local node = FlightCore.engines[s.slot]
                local online = node ~= nil
                local bg = online and "5" or "e"

                for r = 0, cardH - 1 do
                    self:safeBlit(cx, cy + r, string.rep(" ", cardW), "0", bg)
                end
                self:safeBlit(cx + 1, cy, string.format("[%s] %s", s.slot, s.name:sub(1, cardW - 6)), "0", bg)
                self:safeBlit(cx + 1, cy + 1, string.format("STATUS: %s", online and "ONLINE" or "OFFLINE"), "0", bg)
            end

            local btmY = h - 2
            local btmW = math.floor((w - 3) / 2)
            self:addBtn(2, btmY, btmW, 2, "RE-SCAN HARDWARE", "3", "0", function() FlightCore.scanQuadTurtles(); FlightCore.state.statusMsg="Hardware Re-scanned!" end)
            self:addBtn(3 + btmW, btmY, btmW, 2, "STOP ALL ENGINES", "e", "0", function() FlightCore.stopEngines() end)
        end

        -- 繪製所有按鈕
        for _, btn in ipairs(self.buttons) do
            for dy = 0, btn.h - 1 do
                self:safeBlit(btn.x, btn.y + dy, string.rep(" ", btn.w), btn.fg, btn.bg)
            end
            local tx = btn.x + math.floor((btn.w - #btn.text) / 2)
            local ty = btn.y + math.floor(btn.h / 2)
            self:safeBlit(tx, ty, btn.text, btn.fg, btn.bg)
        end
    end,
    handleEvent = function(self, event, p1, p2, p3)
        if event == "monitor_touch" or event == "mouse_click" then
            local clickX, clickY = p2, p3
            for _, btn in ipairs(self.buttons) do
                if clickX >= btn.x and clickX <= btn.x + btn.w - 1 and clickY >= btn.y and clickY <= btn.y + btn.h - 1 then
                    pcall(btn.action)
                    self:draw()
                    break
                end
            end
        end
    end
}

-- ========================================================
-- PART 3: 驅動自動探測與統一調度 (DISPATCHER & RUNNER)
-- ========================================================
local activeDriver = nil

if requestedDriver == "directgpu" or requestedDriver == "gpu" or requestedDriver == "direct" then
    activeDriver = Drivers.directgpu
elseif requestedDriver == "tom" or requestedDriver == "toms" or requestedDriver == "tm" then
    activeDriver = Drivers.toms
elseif requestedDriver == "normal" or requestedDriver == "mon" or requestedDriver == "monitor" or requestedDriver == "term" then
    activeDriver = Drivers.normal
else
    -- 自動探測模式 (優先級: DirectGPU -> Tom's GPU -> Monitor/Terminal)
    if Drivers.directgpu:isAvailable() then
        activeDriver = Drivers.directgpu
    elseif Drivers.toms:isAvailable() then
        activeDriver = Drivers.toms
    else
        activeDriver = Drivers.normal
    end
end

print(string.format("[AVIONICS] Active Display Driver: %s", activeDriver.name))

-- 初始化硬體與驅動
FlightCore.scanQuadTurtles()
activeDriver:init()

-- 3.1 核心動力控制線程 (20Hz)
local function flightLoop()
    local scanTicker = 0
    while true do
        scanTicker = scanTicker + 1
        if scanTicker >= 20 then
            scanTicker = 0
            FlightCore.scanQuadTurtles()
        end
        FlightCore.step()
        sleep(0.05)
    end
end

-- 3.2 顯示繪製線程 (約 12.5 FPS，平滑且完全避免 Watchdog 逾時)
local function renderLoop()
    while true do
        activeDriver:draw()
        sleep(0.08)
    end
end

-- 3.3 觸控與點擊監聽線程
local function eventLoop()
    while true do
        local event, p1, p2, p3, p4, p5 = os.pullEvent()
        activeDriver:handleEvent(event, p1, p2, p3, p4, p5)
    end
end

parallel.waitForAll(flightLoop, renderLoop, eventLoop)
