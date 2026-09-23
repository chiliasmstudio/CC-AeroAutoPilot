--[[
    Create: Avionics & CC: Tweaked
    Unified Multi-Engine Avionics Flight Computer (多軸模組化統一飛控大腦)
    Version: v3.7.0 (Adaptive Multi-Screen 3x3 & Compact Edition)
    
    螢幕尺寸自適應分類 (Dual Screen Size Mode):
    - 大型螢幕 (>= 3x3): 啟動全功能高解析度航空玻璃駕駛艙，提供超大 A350 儀表、細緻多引擎遙測與完整控制面板。
    - 小型螢幕 (< 3x3): 啟動緊湊高效儀表佈局，優化字體邊界與按鈕觸控區域，防止任何元素溢出或重疊。

    6 大功能視圖 (6 Major Glass Cockpit Views):
      1. OVERVIEW (綜合駕駛艙: 姿態儀 + 四象限動力 + 精簡控制)
      2. PFD (主飛行儀表: 巨大人工地平線姿態儀 + 數位高度錶 + 升降速度)
      3. ECAM (發動機動力監控: 2x2 四象限直觀排布，支援單側多引擎即時監控)
      4. CTRL (飛行控制畫面: 完整獨立控制面板、高度微調、油門調整、模式切換)
      5. NAV (快速導航與預設高度: 0m/80m/150m/200m/300m/鎖定)
      6. SYS (系統診斷與硬體自檢: 4 象限烏龜即時心跳、延遲與 PWM 反饋)

    使用方式:
      run.lua              (自動檢測最佳顯示硬體: DirectGPU -> Tom's GPU -> Monitor -> Terminal)
      run.lua tom          (強制使用 Tom's Peripherals GPU 驅動，支援多 GPU 聯屏)
      run.lua directgpu    (強制使用 CC-DirectGPU-Mod 驅動)
      run.lua normal       (強制使用 CC: Tweaked 原生螢幕/終端機驅動，支援多螢幕)
--]]

local VERSION = "v3.7.0"
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

-- 支援一側/象限配置多個引擎 (陣列結構)
FlightCore.engines = { FL = {}, FR = {}, BL = {}, BR = {} }
FlightCore.engineOutputs  = { FL = 0, FR = 0, BL = 0, BR = 0 }
FlightCore.virtualOutputs = { FL = 0.0, FR = 0.0, BL = 0.0, BR = 0.0 }

-- 烏龜心跳與遙測健康度資料庫 (按 Turtle ID 索引)
FlightCore.turtles = {}

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
    FlightCore.engines.FL = {}
    FlightCore.engines.FR = {}
    FlightCore.engines.BL = {}
    FlightCore.engines.BR = {}

    local unmapped = {}
    for _, name in ipairs(peripheral.getNames()) do
        local pType = peripheral.getType(name)
        if pType == "turtle" or pType == "computer" then
            local wrapped = peripheral.wrap(name)
            local label = ""
            if wrapped and wrapped.getLabel then
                pcall(function() label = wrapped.getLabel() or "" end)
            end
            if label == "" then label = name end

            local mapped = false
            if matchPrefix(label, "FL") then
                table.insert(FlightCore.engines.FL, wrapped)
                mapped = true
            elseif matchPrefix(label, "FR") then
                table.insert(FlightCore.engines.FR, wrapped)
                mapped = true
            elseif matchPrefix(label, "BL") then
                table.insert(FlightCore.engines.BL, wrapped)
                mapped = true
            elseif matchPrefix(label, "BR") then
                table.insert(FlightCore.engines.BR, wrapped)
                mapped = true
            end

            if not mapped then
                table.insert(unmapped, wrapped)
            end
        end
    end

    if #FlightCore.engines.FL == 0 and #FlightCore.engines.FR == 0 and #FlightCore.engines.BL == 0 and #FlightCore.engines.BR == 0 then
        if #unmapped >= 4 then
            table.insert(FlightCore.engines.FL, unmapped[1])
            table.insert(FlightCore.engines.FR, unmapped[2])
            table.insert(FlightCore.engines.BL, unmapped[3])
            table.insert(FlightCore.engines.BR, unmapped[4])
        end
    end
end

function FlightCore.getQuadHealth(role)
    local directCount = #(FlightCore.engines[role] or {})
    local onlineCount = 0
    local now = os.epoch("utc")
    local quadTurtles = {}

    for id, t in pairs(FlightCore.turtles) do
        if t.role == role then
            local isOnline = (now - t.lastSeen) < 3500
            table.insert(quadTurtles, {
                id = id,
                label = t.label or ("T#" .. id),
                online = isOnline,
                sig = t.sig or 0,
                age = (now - t.lastSeen) / 1000
            })
            if isOnline then onlineCount = onlineCount + 1 end
        end
    end

    local totalCount = math.max(directCount, #quadTurtles)
    if directCount > 0 and onlineCount == 0 then
        onlineCount = directCount
    end

    return {
        total = totalCount,
        online = onlineCount,
        nodes = quadTurtles
    }
end

function FlightCore.processModemMessages()
    if not FlightCore.masterModem then return end
    while true do
        local event, side, ch, replyCh, msg, dist = os.pullEventRaw("modem_message")
        if event == "modem_message" and ch == 101 and type(msg) == "table" then
            if msg.type == "HEARTBEAT" and msg.role and msg.id then
                FlightCore.turtles[msg.id] = {
                    role = msg.role,
                    label = msg.label or ("Turtle-" .. msg.id),
                    sig = msg.sig or 0,
                    ver = msg.ver or "unknown",
                    lastSeen = os.epoch("utc")
                }
            end
        else
            break
        end
    end
end

function FlightCore.outputToEngines(sigFL, sigFR, sigBL, sigBR)
    local targets = { FL = sigFL, FR = sigFR, BL = sigBR, BR = sigBR }

    if FlightCore.masterModem then
        pcall(function()
            FlightCore.masterModem.transmit(100, 101, {
                type = "FLIGHT_SYNC",
                FL = sigFL,
                FR = sigFR,
                BL = sigBL,
                BR = sigBR,
                timestamp = os.epoch("utc")
            })
        end)
    end

    for role, sig in pairs(targets) do
        local engList = FlightCore.engines[role] or {}
        for _, dev in ipairs(engList) do
            for _, s in ipairs(FlightCore.outputSides) do
                pcall(function() dev.setOutput(s, sig > 0) end)
                pcall(function() dev.setAnalogOutput(s, sig) end)
            end
        end
    end
end

function FlightCore.setTargetAlt(alt)
    FlightCore.state.targetAlt = math.max(0, math.min(1000, alt))
    FlightCore.state.mode = "HOLD_ALT"
    FlightCore.state.statusMsg = string.format("Target: %.1fm", FlightCore.state.targetAlt)
    FlightCore.altPID:reset()
end

function FlightCore.adjustTargetAlt(delta)
    FlightCore.setTargetAlt(FlightCore.state.targetAlt + delta)
end

function FlightCore.lockCurrentAlt()
    if FlightCore.altiSensor then
        local currentAlt = FlightCore.altiSensor.getHeight() or 0
        FlightCore.setTargetAlt(currentAlt)
    end
end

function FlightCore.adjustBaseThrottle(delta)
    FlightCore.state.baseThrottle = math.max(0.01, math.min(15.0, FlightCore.state.baseThrottle + delta))
    FlightCore.state.statusMsg = string.format("Base Throttle: %.2f", FlightCore.state.baseThrottle)
end

function FlightCore.stopEngines()
    FlightCore.state.mode = "IDLE"
    FlightCore.state.statusMsg = "Engines Stopped (IDLE)"
    FlightCore.engineOutputs = { FL = 0, FR = 0, BL = 0, BR = 0 }
    FlightCore.virtualOutputs = { FL = 0.0, FR = 0.0, BL = 0.0, BR = 0.0 }
    FlightCore.outputToEngines(0, 0, 0, 0)
end

function FlightCore.holdAltitude()
    if FlightCore.state.mode ~= "HOLD_ALT" then
        if FlightCore.altiSensor then
            local currentAlt = FlightCore.altiSensor.getHeight() or 0
            FlightCore.state.targetAlt = currentAlt
            FlightCore.state.virtualAlt = currentAlt
        end
        FlightCore.state.mode = "HOLD_ALT"
        FlightCore.state.statusMsg = "Altitude Hold Active"
        FlightCore.altPID:reset()
    end
end

function FlightCore.startCalibration()
    if not FlightCore.altiSensor then
        FlightCore.state.statusMsg = "ERR: No Altitude Sensor!"
        return
    end
    FlightCore.state.mode = "CALIBRATING"
    FlightCore.state.calibPhase = "GROUND_SEARCH"
    FlightCore.state.calibStartAlt = FlightCore.altiSensor.getHeight() or 0
    FlightCore.state.calibTargetAlt = 200.0
    FlightCore.state.calibThrottle = 0.0
    FlightCore.state.rampRate = 0.0001
    FlightCore.state.stableTimer = 0
    FlightCore.state.statusMsg = "Calibrating: Ground Search..."
    FlightCore.altPID:reset()
end

function FlightCore.updateFlightLogic()
    FlightCore.pwmTick = (FlightCore.pwmTick + 1) % 10

    if FlightCore.state.mode == "IDLE" then
        FlightCore.engineOutputs = { FL = 0, FR = 0, BL = 0, BR = 0 }
        FlightCore.virtualOutputs = { FL = 0.0, FR = 0.0, BL = 0.0, BR = 0.0 }
        FlightCore.outputToEngines(0, 0, 0, 0)
        return
    end

    if not FlightCore.altiSensor then
        FlightCore.altiSensor = peripheral.find("altitude_sensor")
    end

    local currentAlt = FlightCore.altiSensor and FlightCore.altiSensor.getHeight() or 0
    local currentVspeed = FlightCore.altiSensor and FlightCore.altiSensor.getVerticalSpeed() or 0
    local currPitch, currRoll = FlightCore.getGimbalData()

    if FlightCore.state.mode == "CALIBRATING" then
        if FlightCore.state.calibPhase == "GROUND_SEARCH" then
            FlightCore.state.calibThrottle = FlightCore.state.calibThrottle + FlightCore.state.rampRate
            FlightCore.state.rampRate = FlightCore.state.rampRate * 1.05
            FlightCore.state.baseThrottle = FlightCore.state.calibThrottle

            local altDelta = currentAlt - FlightCore.state.calibStartAlt
            if altDelta > 1.5 or currentVspeed > 0.5 then
                FlightCore.state.calibPhase = "CLIMB_TO_200"
                FlightCore.state.targetAlt = 200.0
                FlightCore.state.virtualAlt = currentAlt
                FlightCore.state.statusMsg = string.format("Lift Found (%.2f)! Climbing to 200m...", FlightCore.state.baseThrottle)
            else
                FlightCore.state.statusMsg = string.format("Searching Lift: %.3f (Alt: %.1f)", FlightCore.state.baseThrottle, currentAlt)
            end

            FlightCore.virtualOutputs.FL = FlightCore.state.baseThrottle
            FlightCore.virtualOutputs.FR = FlightCore.state.baseThrottle
            FlightCore.virtualOutputs.BL = FlightCore.state.baseThrottle
            FlightCore.virtualOutputs.BR = FlightCore.state.baseThrottle

        elseif FlightCore.state.calibPhase == "CLIMB_TO_200" then
            local speedLimit = 2.5
            if currentAlt < FlightCore.state.targetAlt then
                FlightCore.state.virtualAlt = math.min(FlightCore.state.targetAlt, FlightCore.state.virtualAlt + speedLimit * 0.05)
            else
                FlightCore.state.virtualAlt = math.max(FlightCore.state.targetAlt, FlightCore.state.virtualAlt - speedLimit * 0.05)
            end

            local altError = FlightCore.state.virtualAlt - currentAlt
            local pidAdj = FlightCore.altPID:update(altError)
            local finalThrust = math.max(0, math.min(15, FlightCore.state.baseThrottle + pidAdj))

            FlightCore.virtualOutputs.FL = finalThrust
            FlightCore.virtualOutputs.FR = finalThrust
            FlightCore.virtualOutputs.BL = finalThrust
            FlightCore.virtualOutputs.BR = finalThrust

            if math.abs(currentAlt - 200.0) < 1.0 and math.abs(currentVspeed) < 0.2 then
                FlightCore.state.calibPhase = "STABILIZE_200"
                FlightCore.state.stableTimer = 0
                FlightCore.state.statusMsg = "Stabilizing at 200m..."
            else
                FlightCore.state.statusMsg = string.format("Climbing: %.1fm -> 200m (Base: %.2f)", currentAlt, FlightCore.state.baseThrottle)
            end

        elseif FlightCore.state.calibPhase == "STABILIZE_200" then
            local altError = 200.0 - currentAlt
            local pidAdj = FlightCore.altPID:update(altError)

            if currentVspeed > 0.05 then
                FlightCore.state.baseThrottle = math.max(0.1, FlightCore.state.baseThrottle - 0.001)
            elseif currentVspeed < -0.05 then
                FlightCore.state.baseThrottle = math.min(15.0, FlightCore.state.baseThrottle + 0.001)
            end

            local finalThrust = math.max(0, math.min(15, FlightCore.state.baseThrottle + pidAdj))
            FlightCore.virtualOutputs.FL = finalThrust
            FlightCore.virtualOutputs.FR = finalThrust
            FlightCore.virtualOutputs.BL = finalThrust
            FlightCore.virtualOutputs.BR = finalThrust

            if math.abs(currentAlt - 200.0) < 0.5 and math.abs(currentVspeed) < 0.08 then
                FlightCore.state.stableTimer = FlightCore.state.stableTimer + 0.05
                FlightCore.state.statusMsg = string.format("Calibrating: Stable %.1fs/5.0s (Base: %.2f)", FlightCore.state.stableTimer, FlightCore.state.baseThrottle)
                if FlightCore.state.stableTimer >= 5.0 then
                    FlightCore.state.mode = "HOLD_ALT"
                    FlightCore.state.targetAlt = 200.0
                    FlightCore.state.statusMsg = string.format("Calib Complete! Hover Base: %.2f", FlightCore.state.baseThrottle)
                end
            else
                FlightCore.state.stableTimer = 0
                FlightCore.state.statusMsg = string.format("Stabilizing: %.1fm (V.S: %+.2f)", currentAlt, currentVspeed)
            end
        end

    elseif FlightCore.state.mode == "HOLD_ALT" then
        local altDiff = FlightCore.state.targetAlt - FlightCore.state.virtualAlt
        local step = math.max(-2.5 * 0.05, math.min(2.5 * 0.05, altDiff))
        FlightCore.state.virtualAlt = FlightCore.state.virtualAlt + step

        local altError = FlightCore.state.virtualAlt - currentAlt
        local pidAdj = FlightCore.altPID:update(altError)

        local pitchCorr = -currPitch * 0.05
        local rollCorr = currRoll * 0.05

        FlightCore.virtualOutputs.FL = math.max(0, math.min(15, FlightCore.state.baseThrottle + pidAdj + pitchCorr - rollCorr))
        FlightCore.virtualOutputs.FR = math.max(0, math.min(15, FlightCore.state.baseThrottle + pidAdj + pitchCorr + rollCorr))
        FlightCore.virtualOutputs.BL = math.max(0, math.min(15, FlightCore.state.baseThrottle + pidAdj - pitchCorr - rollCorr))
        FlightCore.virtualOutputs.BR = math.max(0, math.min(15, FlightCore.state.baseThrottle + pidAdj - pitchCorr + rollCorr))
    end

    local function calcPWM(val)
        local intPart = math.floor(val)
        local fracPart = val - intPart
        local threshold = math.floor(fracPart * 10 + 0.5)
        if FlightCore.pwmTick < threshold then
            return math.min(15, intPart + 1)
        else
            return intPart
        end
    end

    FlightCore.engineOutputs.FL = calcPWM(FlightCore.virtualOutputs.FL)
    FlightCore.engineOutputs.FR = calcPWM(FlightCore.virtualOutputs.FR)
    FlightCore.engineOutputs.BL = calcPWM(FlightCore.virtualOutputs.BL)
    FlightCore.engineOutputs.BR = calcPWM(FlightCore.virtualOutputs.BR)

    FlightCore.outputToEngines(
        FlightCore.engineOutputs.FL,
        FlightCore.engineOutputs.FR,
        FlightCore.engineOutputs.BL,
        FlightCore.engineOutputs.BR
    )
end

-- ========================================================
-- PART 2: 多前端顯示驅動層 (FRONTEND DISPLAY DRIVERS)
-- ========================================================
local VIEW_TITLES = {
    OVERVIEW = "FLIGHT MONITOR",
    PFD      = "PRIMARY FLIGHT",
    ECAM     = "ECAM 2x2",
    CTRL     = "FLIGHT CONTROLS",
    NAV      = "NAV PRESETS",
    SYS      = "SYSTEM STATUS"
}

local Drivers = {}

-- --------------------------------------------------------
-- 2.1 驅動 A: CC-DirectGPU-Mod (高解析硬體加速)
-- --------------------------------------------------------
Drivers.direct = {
    screens = {},
    init = function(self)
        self.screens = {}
        for _, name in ipairs(peripheral.getNames()) do
            if peripheral.getType(name) == "directgpu" then
                local gpu = peripheral.wrap(name)
                if gpu then
                    local count = gpu.getDisplayCount and gpu.getDisplayCount() or 1
                    for dId = 0, count - 1 do
                        local info = gpu.getDisplayInfo and gpu.getDisplayInfo(dId) or {pixelWidth=320, pixelHeight=240}
                        table.insert(self.screens, {
                            gpu = gpu,
                            displayId = dId,
                            screenW = info.pixelWidth or 320,
                            screenH = info.pixelHeight or 240,
                            currentView = "OVERVIEW",
                            isMenuOpen = false,
                            buttons = {}
                        })
                    end
                end
            end
        end
        return #self.screens > 0
    end,
    drawA350Dial = function(self, scr, cx, cy, r, val, maxVal, label, sig, slot)
        local gpu = scr.gpu
        local dispId = scr.displayId
        local labelSize = math.max(9, math.floor(r * 0.44))
        local lblOffset = math.floor(#label * (labelSize * 0.32))
        gpu.drawText(dispId, label, cx - lblOffset, cy - r - math.floor(labelSize * 1.1), 200, 230, 255, "Arial", labelSize, "bold")

        local arcPts, arcPtsOuter = {}, {}
        for deg = 210, -30, -10 do
            local rad = math.rad(deg)
            table.insert(arcPts, { math.floor(cx + r * math.cos(rad) + 0.5), math.floor(cy - r * math.sin(rad) + 0.5) })
            table.insert(arcPtsOuter, { math.floor(cx + (r + 1) * math.cos(rad) + 0.5), math.floor(cy - (r + 1) * math.sin(rad) + 0.5) })
        end
        gpu.drawPolylines(dispId, arcPts, 160, 180, 205)
        gpu.drawPolylines(dispId, arcPtsOuter, 120, 140, 165)

        for _, deg in ipairs({210, 90, -30}) do
            local rad = math.rad(deg)
            gpu.drawLine(dispId, math.floor(cx + (r - 3) * math.cos(rad) + 0.5), math.floor(cy - (r - 3) * math.sin(rad) + 0.5),
                                           math.floor(cx + (r + 5) * math.cos(rad) + 0.5), math.floor(cy - (r + 5) * math.sin(rad) + 0.5), 180, 200, 225)
        end

        local redPts = {}
        for deg = 10, -30, -8 do
            local rad = math.rad(deg)
            table.insert(redPts, { math.floor(cx + r * math.cos(rad) + 0.5), math.floor(cy - r * math.sin(rad) + 0.5) })
        end
        gpu.drawPolylines(dispId, redPts, 255, 55, 55)

        local ratio = math.min(1.0, math.max(0.0, val / (maxVal or 15.0)))
        local nRad = math.rad(210 - ratio * 240)
        local nx = math.floor(cx + (r - 3) * math.cos(nRad) + 0.5)
        local ny = math.floor(cy - (r - 3) * math.sin(nRad) + 0.5)
        gpu.drawLine(dispId, cx, cy, nx, ny, 50, 255, 100)
        gpu.drawCircle(dispId, cx, cy, math.max(2, math.floor(r * 0.14)), 200, 220, 240, true)

        local boxW = math.max(28, math.floor(r * 1.50))
        local boxH = math.max(12, math.floor(r * 0.58))
        local boxX = cx - math.floor(boxW / 2)
        local boxY = cy + math.floor(r * 0.28)
        gpu.fillRect(dispId, boxX, boxY, boxW, boxH, 12, 18, 28)
        gpu.drawPolylines(dispId, {{boxX, boxY}, {boxX+boxW, boxY}, {boxX+boxW, boxY+boxH}, {boxX, boxY+boxH}, {boxX, boxY}}, 40, 150, 200)

        local valStr = string.format("%4.1f", val)
        local numFontSize = math.max(9, math.floor(boxH * 0.70))
        gpu.drawText(dispId, valStr, boxX + 3, boxY + 2, 70, 255, 120, "Arial", numFontSize, "bold")

        local sigStr = string.format("PWM:%d", sig or 0)
        local sigFontSize = math.max(8, math.floor(boxH * 0.50))
        gpu.drawText(dispId, sigStr, cx - math.floor(#sigStr * 2.8), boxY + boxH + 2, 150, 190, 225, "Arial", sigFontSize, "plain")
    end,
    draw = function(self)
        for _, scr in ipairs(self.screens) do
            self:drawScreen(scr)
        end
    end,
    drawScreen = function(self, scr)
        local gpu = scr.gpu
        local dispId = scr.displayId
        local dispInfo = gpu.getDisplayInfo(dispId)
        local sw = dispInfo.pixelWidth or 320
        local sh = dispInfo.pixelHeight or 240
        local isWide = (sw >= 350)
        local isLarge = (sw >= 220 and sh >= 180)

        gpu.clear(dispId, 15, 20, 30)
        scr.buttons = {}

        local function addBtn(x, y, w, h, text, bg, fg, act)
            table.insert(scr.buttons, {x=x, y=y, w=w, h=h, text=text, bg=bg, fg=fg, action=act})
        end

        local headerH = isLarge and math.max(24, math.floor(sh * 0.08)) or 18
        gpu.fillRect(dispId, 0, 0, sw, headerH, 25, 40, 65)

        local menuBtnW = (sw >= 240) and 50 or 26
        local menuBtnH = headerH - 4
        local menuBtnX = sw - menuBtnW - 3
        local menuBtnY = 2
        if scr.isMenuOpen then
            addBtn(menuBtnX, menuBtnY, menuBtnW, menuBtnH, "[X]", {190, 45, 45}, {255, 255, 255}, function() scr.isMenuOpen = false end)
        else
            addBtn(menuBtnX, menuBtnY, menuBtnW, menuBtnH, (sw >= 240) and "[MENU]" or "[=]", {35, 80, 150}, {255, 255, 255}, function() scr.isMenuOpen = true end)
        end

        local titleFontSize = isLarge and math.max(10, math.min(14, math.floor(headerH * 0.48))) or 9
        local viewTitle = scr.isMenuOpen and "SELECT VIEW" or (isLarge and (VIEW_TITLES[scr.currentView] or scr.currentView) or scr.currentView)
        gpu.drawText(dispId, viewTitle, 8, math.floor((headerH - titleFontSize) / 2), 240, 245, 255, "Arial", titleFontSize, "bold")

        if scr.isMenuOpen then
            local cardW = math.floor((sw - 20) / 2)
            local cardH = math.floor((sh - headerH - 20) / 3)
            local startY = headerH + 5

            local function addMenuCard(col, row, title, targetView, color)
                local cx = 6 + (col - 1) * (cardW + 8)
                local cy = startY + (row - 1) * (cardH + 5)
                addBtn(cx, cy, cardW, cardH, title, color, {255, 255, 255}, function()
                    scr.currentView = targetView
                    scr.isMenuOpen = false
                end)
            end

            addMenuCard(1, 1, isLarge and "1. OVERVIEW (ALL)" or "1. OVERVIEW", "OVERVIEW", {25, 60, 95})
            addMenuCard(2, 1, isLarge and "2. PFD (FLIGHT)" or "2. PFD", "PFD", {20, 90, 50})
            addMenuCard(1, 2, isLarge and "3. ECAM (QUAD ENG)" or "3. ECAM", "ECAM", {120, 40, 30})
            addMenuCard(2, 2, isLarge and "4. CTRL (CONTROLS)" or "4. CTRL", "CTRL", {18, 120, 100})
            addMenuCard(1, 3, isLarge and "5. NAV (PRESETS)" or "5. NAV", "NAV", {35, 115, 165})
            addMenuCard(2, 3, isLarge and "6. SYS (DIAGNOSE)" or "6. SYS", "SYS", {80, 45, 95})

        elseif scr.currentView == "OVERVIEW" then
            local currAlt = FlightCore.altiSensor and FlightCore.altiSensor.getHeight() or 0
            local currVspeed = FlightCore.altiSensor and FlightCore.altiSensor.getVerticalSpeed() or 0
            local currPitch, currRoll = FlightCore.getGimbalData()

            if isWide then
                local instY = headerH + 6
                local instH = math.floor((sh - headerH) * 0.52)
                local leftW = math.max(120, math.floor(sw * 0.35))
                local pfdX = 6

                gpu.fillRect(dispId, pfdX, instY, leftW, instH, 20, 26, 38)
                gpu.drawText(dispId, "PRIMARY FLIGHT", pfdX + 6, instY + 4, 170, 210, 230, "Arial", 10, "bold")

                local gR = math.min(30, math.floor(instH * 0.28))
                local gCX = pfdX + math.floor(leftW * 0.26)
                local gCY = instY + math.floor(instH * 0.54)
                gpu.drawCircle(dispId, gCX, gCY, gR, 80, 170, 240, true)
                gpu.drawText(dispId, string.format("%.0f", currAlt), gCX - 8, gCY - 5, 255, 255, 255, "Arial", 11, "bold")

                local textX = pfdX + math.floor(leftW * 0.52)
                local rowSpacing = math.floor((instH - 20) / 4)
                gpu.drawText(dispId, string.format("TGT:%.0fm", FlightCore.state.targetAlt), textX, instY + 14, 80, 230, 255, "Arial", 10, "bold")
                gpu.drawText(dispId, string.format("V.S:%+.2f", currVspeed), textX, instY + 14 + rowSpacing, 255, 205, 75, "Arial", 10, "bold")
                local mCol = (FlightCore.state.mode == "HOLD_ALT") and {80, 255, 120} or {255, 80, 80}
                gpu.drawText(dispId, string.format("MOD:%s", FlightCore.state.mode:sub(1,6)), textX, instY + 14 + rowSpacing * 2, mCol[1], mCol[2], mCol[3], "Arial", 10, "bold")
                gpu.drawText(dispId, string.format("P:%+2.0f R:%+2.0f", currPitch, currRoll), textX, instY + 14 + rowSpacing * 3, 180, 220, 255, "Arial", 9, "plain")

                local ecamX = pfdX + leftW + 6
                local ecamW = sw - ecamX - 6
                gpu.fillRect(dispId, ecamX, instY, ecamW, instH, 18, 24, 34)

                local slotW = math.floor((ecamW - 6) / 4)
                local slots = {"FL", "FR", "BL", "BR"}
                local dialR = math.min(math.floor(slotW * 0.33), math.floor(instH * 0.28))
                local engCenterY = instY + math.floor(instH * 0.52)

                for i, slot in ipairs(slots) do
                    local engCenterX = ecamX + 3 + math.floor((i - 0.5) * slotW)
                    local qH = FlightCore.getQuadHealth(slot)
                    local lbl = string.format("%s(%d)", slot, qH.total)
                    self:drawA350Dial(scr, engCenterX, engCenterY, dialR, FlightCore.virtualOutputs[slot], 15.0, lbl, FlightCore.engineOutputs[slot], slot)
                end

                local statY = instY + instH + 5
                local statH = math.max(18, math.floor(sh * 0.07))
                gpu.fillRect(dispId, pfdX, statY, sw - 12, statH, 30, 36, 50)
                gpu.drawText(dispId, string.format("STATUS: %s", FlightCore.state.statusMsg:sub(1,40)), pfdX + 8, statY + 4, 80, 230, 255, "Arial", 10, "bold")

                local btnAreaY = statY + statH + 5
                local btnAreaH = sh - btnAreaY - 5
                local rowH = math.floor((btnAreaH - 8) / 3)
                local bY1 = btnAreaY
                local bW1 = math.floor((sw - 12 - 16) / 5)
                addBtn(pfdX, bY1, bW1, rowH, "+10m", {27, 94, 32}, {232, 245, 233}, function() FlightCore.adjustTargetAlt(10) end)
                addBtn(pfdX + bW1 + 4, bY1, bW1, rowH, "+1m", {46, 125, 50}, {232, 245, 233}, function() FlightCore.adjustTargetAlt(1) end)
                addBtn(pfdX + (bW1 + 4)*2, bY1, bW1, rowH, "-1m", {230, 81, 0}, {255, 243, 224}, function() FlightCore.adjustTargetAlt(-1) end)
                addBtn(pfdX + (bW1 + 4)*3, bY1, bW1, rowH, "-10m", {198, 40, 40}, {255, 235, 238}, function() FlightCore.adjustTargetAlt(-10) end)
                addBtn(pfdX + (bW1 + 4)*4, bY1, bW1, rowH, "LOCK", {0, 131, 143}, {224, 247, 250}, function() FlightCore.lockCurrentAlt() end)

                local bY2 = bY1 + rowH + 4
                local bW2 = math.floor((sw - 12 - 12) / 4)
                addBtn(pfdX, bY2, bW2, rowH, "BASE +", {0, 105, 92}, {224, 242, 241}, function() FlightCore.adjustBaseThrottle(1.0) end)
                addBtn(pfdX + bW2 + 4, bY2, bW2, rowH, "BASE -", {55, 71, 79}, {236, 239, 241}, function() FlightCore.adjustBaseThrottle(-1.0) end)
                addBtn(pfdX + (bW2 + 4)*2, bY2, bW2, rowH, "RE-SCAN", {21, 101, 192}, {227, 242, 253}, function() FlightCore.scanQuadTurtles(); FlightCore.state.statusMsg="Re-scanned!" end)
                addBtn(pfdX + (bW2 + 4)*3, bY2, bW2, rowH, "CALIB", {106, 27, 154}, {243, 229, 245}, function() FlightCore.startCalibration() end)

                local bY3 = bY2 + rowH + 4
                local bW3 = math.floor((sw - 12 - 4) / 2)
                local holdBg = (FlightCore.state.mode == "HOLD_ALT") and {46, 125, 50} or {27, 94, 32}
                addBtn(pfdX, bY3, bW3, rowH, "[ HOLD ALT ]", holdBg, {255, 255, 255}, function() FlightCore.holdAltitude() end)
                local stopBg = (FlightCore.state.mode == "IDLE") and {198, 40, 40} or {183, 28, 28}
                addBtn(pfdX + bW3 + 4, bY3, bW3, rowH, "[ STOP / IDLE ]", stopBg, {255, 255, 255}, function() FlightCore.stopEngines() end)
            else
                -- 緊湊/直立螢幕版 (2x2 Quad Engine 矩陣)
                local instY = headerH + 4
                local instH = math.floor((sh - headerH) * 0.54)
                local colW = math.floor((sw - 14) / 2)
                local pfdX = 4
                local ecamX = pfdX + colW + 4

                -- 左側: PFD / 姿態
                gpu.fillRect(dispId, pfdX, instY, colW, instH, 20, 26, 38)
                gpu.drawText(dispId, "ALT/ATT", pfdX + 4, instY + 3, 170, 210, 230, "Arial", 9, "bold")

                local gR = math.min(16, math.floor(instH * 0.26))
                local gCX = pfdX + 18
                local gCY = instY + math.floor(instH * 0.60)
                gpu.drawCircle(dispId, gCX, gCY, gR, 80, 170, 240, true)
                gpu.drawText(dispId, string.format("%.0f", currAlt), gCX - 7, gCY - 4, 255, 255, 255, "Arial", 9, "bold")

                local textX = pfdX + 38
                local rowSpacing = math.floor((instH - 16) / 4)
                gpu.drawText(dispId, string.format("T:%.0f", FlightCore.state.targetAlt), textX, instY + 12, 80, 230, 255, "Arial", 9, "bold")
                gpu.drawText(dispId, string.format("V:%+.1f", currVspeed), textX, instY + 12 + rowSpacing, 255, 205, 75, "Arial", 9, "bold")
                local mCol = (FlightCore.state.mode == "HOLD_ALT") and {80, 255, 120} or {255, 80, 80}
                gpu.drawText(dispId, string.format("M:%s", FlightCore.state.mode:sub(1,4)), textX, instY + 12 + rowSpacing * 2, mCol[1], mCol[2], mCol[3], "Arial", 9, "bold")
                gpu.drawText(dispId, string.format("P:%+.0f", currPitch), textX, instY + 12 + rowSpacing * 3, 180, 220, 255, "Arial", 9, "plain")

                -- 右側: 2x2 Mini Quad Engines
                gpu.fillRect(dispId, ecamX, instY, colW, instH, 18, 24, 34)
                local subW = math.floor((colW - 6) / 2)
                local subH = math.floor((instH - 8) / 2)
                local miniSlots = {
                    {slot="FL", col=1, row=1},
                    {slot="FR", col=2, row=1},
                    {slot="BL", col=1, row=2},
                    {slot="BR", col=2, row=2}
                }
                for _, ms in ipairs(miniSlots) do
                    local sx = ecamX + 2 + (ms.col - 1) * (subW + 2)
                    local sy = instY + 2 + (ms.row - 1) * (subH + 2)
                    local qH = FlightCore.getQuadHealth(ms.slot)
                    local val = FlightCore.virtualOutputs[ms.slot]
                    gpu.fillRect(dispId, sx, sy, subW, subH, 24, 32, 46)
                    gpu.drawText(dispId, string.format("%s %dE", ms.slot, qH.total), sx + 2, sy + 2, 200, 230, 255, "Arial", 8, "bold")
                    gpu.drawText(dispId, string.format("%4.1f", val), sx + 2, sy + math.max(10, subH - 9), 70, 255, 120, "Arial", 8, "plain")
                end

                -- 底部狀態
                local statY = instY + instH + 4
                local statH = 15
                gpu.fillRect(dispId, pfdX, statY, sw - 8, statH, 30, 36, 50)
                local maxStatChars = math.max(6, math.floor((sw - 20) / 6))
                gpu.drawText(dispId, string.format("STATUS: %s", FlightCore.state.statusMsg:sub(1, maxStatChars)), pfdX + 4, statY + 3, 80, 230, 255, "Arial", 9, "bold")

                -- 底部按鈕
                local btnAreaY = statY + statH + 4
                local btnAreaH = sh - btnAreaY - 3
                local rowH = math.floor((btnAreaH - 3) / 2)
                local bW4 = math.floor((sw - 8 - 9) / 4)
                addBtn(pfdX, btnAreaY, bW4, rowH, "+10", {27, 94, 32}, {255, 255, 255}, function() FlightCore.adjustTargetAlt(10) end)
                addBtn(pfdX + bW4 + 3, btnAreaY, bW4, rowH, "-10", {198, 40, 40}, {255, 255, 255}, function() FlightCore.adjustTargetAlt(-10) end)
                addBtn(pfdX + (bW4 + 3)*2, btnAreaY, bW4, rowH, "B+", {0, 105, 92}, {255, 255, 255}, function() FlightCore.adjustBaseThrottle(1.0) end)
                addBtn(pfdX + (bW4 + 3)*3, btnAreaY, bW4, rowH, "B-", {55, 71, 79}, {255, 255, 255}, function() FlightCore.adjustBaseThrottle(-1.0) end)

                local bY2 = btnAreaY + rowH + 2
                local bW2 = math.floor((sw - 8 - 3) / 2)
                local holdBg = (FlightCore.state.mode == "HOLD_ALT") and {46, 125, 50} or {27, 94, 32}
                addBtn(pfdX, bY2, bW2, rowH, "HOLD", holdBg, {255, 255, 255}, function() FlightCore.holdAltitude() end)
                local stopBg = (FlightCore.state.mode == "IDLE") and {198, 40, 40} or {183, 28, 28}
                addBtn(pfdX + bW2 + 3, bY2, bW2, rowH, "STOP", stopBg, {255, 255, 255}, function() FlightCore.stopEngines() end)
            end

        elseif scr.currentView == "PFD" then
            local currAlt = FlightCore.altiSensor and FlightCore.altiSensor.getHeight() or 0
            local currVspeed = FlightCore.altiSensor and FlightCore.altiSensor.getVerticalSpeed() or 0
            local currPitch, currRoll = FlightCore.getGimbalData()

            local mainH = math.floor((sh - headerH) * (isLarge and 0.70 or 0.62))
            local mainY = headerH + 5

            local leftW = math.floor(sw * 0.52)
            gpu.fillRect(dispId, 6, mainY, leftW, mainH, 20, 26, 38)

            local hCX = 6 + math.floor(leftW / 2)
            local hCY = mainY + math.floor(mainH / 2)
            local hR = math.min(math.floor(leftW * 0.38), math.floor(mainH * 0.40))
            gpu.drawCircle(dispId, hCX, hCY, hR, 80, 170, 240, false)

            local rollRad = math.rad(-currRoll)
            local lx1 = math.floor(hCX - hR * 0.85 * math.cos(rollRad) + 0.5)
            local ly1 = math.floor(hCY - hR * 0.85 * math.sin(rollRad) + 0.5)
            local lx2 = math.floor(hCX + hR * 0.85 * math.cos(rollRad) + 0.5)
            local ly2 = math.floor(hCY + hR * 0.85 * math.sin(rollRad) + 0.5)
            gpu.drawLine(dispId, lx1, ly1, lx2, ly2, 255, 215, 0)
            gpu.drawCircle(dispId, hCX, hCY, 3, 255, 80, 80, true)
            gpu.drawText(dispId, string.format("P:%+4.1f* R:%+4.1f*", currPitch, currRoll), 10, mainY + mainH - 12, 200, 230, 255, "Arial", 9, "bold")

            local rightX = 6 + leftW + 5
            local rightW = sw - rightX - 6
            gpu.fillRect(dispId, rightX, mainY, rightW, mainH, 24, 32, 46)

            gpu.drawText(dispId, "CURRENT ALT", rightX + 6, mainY + 6, 140, 160, 180, "Arial", 9, "plain")
            local altStr = string.format("%.1fm", currAlt)
            gpu.drawText(dispId, altStr, rightX + 6, mainY + 18, 255, 255, 255, "Arial", isLarge and 14 or 11, "bold")

            local midY = mainY + (isLarge and 38 or 30)
            gpu.drawText(dispId, string.format("TGT: %.0fm", FlightCore.state.targetAlt), rightX + 6, midY, 80, 230, 255, "Arial", 9, "bold")
            gpu.drawText(dispId, string.format("V.S: %+.2f", currVspeed), rightX + 6, midY + 13, 255, 205, 75, "Arial", 9, "bold")

            local bY = mainY + mainH + 5
            local bH = sh - bY - 5
            local bW = math.floor((sw - 12 - 12) / 4)
            addBtn(6, bY, bW, bH, "+10m", {30, 100, 45}, {255, 255, 255}, function() FlightCore.adjustTargetAlt(10) end)
            addBtn(6 + (bW+4), bY, bW, bH, "-10m", {190, 40, 40}, {255, 255, 255}, function() FlightCore.adjustTargetAlt(-10) end)
            local holdBg = (FlightCore.state.mode == "HOLD_ALT") and {45, 140, 60} or {30, 90, 40}
            addBtn(6 + (bW+4)*2, bY, bW, bH, "HOLD", holdBg, {255, 255, 255}, function() FlightCore.holdAltitude() end)
            local stopBg = (FlightCore.state.mode == "IDLE") and {200, 45, 45} or {160, 30, 30}
            addBtn(6 + (bW+4)*3, bY, bW, bH, "STOP", stopBg, {255, 255, 255}, function() FlightCore.stopEngines() end)

        elseif scr.currentView == "ECAM" then
            local topY = headerH + 3
            local btmH = isLarge and 22 or 14
            local areaH = sh - topY - btmH - 4
            local quadW = math.floor((sw - 12) / 2)
            local quadH = math.floor((areaH - 3) / 2)

            local quadDefs = {
                {slot="FL", col=1, row=1, title="[FL] FRONT-LEFT"},
                {slot="FR", col=2, row=1, title="[FR] FRONT-RIGHT"},
                {slot="BL", col=1, row=2, title="[BL] BACK-LEFT"},
                {slot="BR", col=2, row=2, title="[BR] BACK-RIGHT"}
            }

            for _, q in ipairs(quadDefs) do
                local qx = 4 + (q.col - 1) * (quadW + 4)
                local qy = topY + (q.row - 1) * (quadH + 3)
                gpu.fillRect(dispId, qx, qy, quadW, quadH, 18, 24, 34)

                local qH = FlightCore.getQuadHealth(q.slot)
                local online = qH.online > 0

                if isLarge and (quadW >= 110) then
                    local titleCol = online and {180, 230, 255} or {255, 120, 120}
                    gpu.drawText(dispId, q.title, qx + 6, qy + 4, titleCol[1], titleCol[2], titleCol[3], "Arial", 10, "bold")

                    local engCountStr = string.format("ENG:%d", qH.total)
                    gpu.drawText(dispId, engCountStr, qx + quadW - math.floor(#engCountStr * 6) - 6, qy + 4, 150, 190, 230, "Arial", 9, "plain")

                    local dialR = math.min(math.floor(quadW * 0.28), math.floor(quadH * 0.28))
                    local engCenterX = qx + math.floor(quadW / 2)
                    local engCenterY = qy + math.floor(quadH * 0.52)
                    self:drawA350Dial(scr, engCenterX, engCenterY, dialR, FlightCore.virtualOutputs[q.slot], 15.0, q.slot, FlightCore.engineOutputs[q.slot], q.slot)

                    local statText = string.format("ACT:%d/%d", qH.online, qH.total)
                    if qH.total == 0 then statText = "NO ENG" end
                    local statCol = (qH.online > 0) and {80, 255, 120} or {255, 80, 80}
                    gpu.drawText(dispId, statText, qx + 6, qy + quadH - 12, statCol[1], statCol[2], statCol[3], "Arial", 9, "bold")
                else
                    -- 緊湊/直立螢幕版
                    local qTitle = string.format("[%s] %dE", q.slot, qH.total)
                    local titleCol = online and {180, 230, 255} or {255, 120, 120}
                    gpu.drawText(dispId, qTitle, qx + 3, qy + 3, titleCol[1], titleCol[2], titleCol[3], "Arial", 8, "bold")

                    local dialR = math.min(10, math.floor(quadH * 0.22))
                    local engCenterX = qx + math.floor(quadW / 2)
                    local engCenterY = qy + 11 + dialR

                    local valStr = string.format("%4.1f", FlightCore.virtualOutputs[q.slot])
                    gpu.drawText(dispId, valStr, engCenterX - 10, engCenterY - 4, 70, 255, 120, "Arial", 8, "bold")

                    local btmLineY = qy + quadH - 9
                    gpu.drawText(dispId, string.format("P:%d", FlightCore.engineOutputs[q.slot] or 0), qx + 3, btmLineY, 150, 190, 225, "Arial", 8, "plain")
                    local actStr = string.format("%d/%d", qH.online, qH.total)
                    local actCol = (qH.online > 0) and {80, 255, 120} or {255, 80, 80}
                    gpu.drawText(dispId, actStr, qx + quadW - math.floor(#actStr * 5) - 3, btmLineY, actCol[1], actCol[2], actCol[3], "Arial", 8, "bold")
                end
            end

            local statY = topY + areaH + 2
            gpu.fillRect(dispId, 4, statY, sw - 8, btmH, 25, 32, 45)
            if isLarge then
                gpu.drawText(dispId, string.format("BASE: %4.2f/15 | BALANCED | STATUS: %s", FlightCore.state.baseThrottle, FlightCore.state.statusMsg:sub(1, 26)), 8, statY + 4, 80, 230, 255, "Arial", 9, "bold")
            else
                local statChars = math.max(6, math.floor((sw - 60) / 6))
                gpu.drawText(dispId, string.format("B:%4.2f | %s", FlightCore.state.baseThrottle, FlightCore.state.statusMsg:sub(1, statChars)), 6, statY + 3, 80, 230, 255, "Arial", 8, "bold")
            end

        elseif scr.currentView == "CTRL" then
            local topY = headerH + 5
            local topH = isLarge and 32 or 20
            gpu.fillRect(dispId, 6, topY, sw - 12, topH, 20, 28, 42)
            local mCol = (FlightCore.state.mode == "HOLD_ALT") and {80, 255, 120} or {255, 80, 80}
            gpu.drawText(dispId, string.format("[%s]", FlightCore.state.mode:sub(1,6)), 10, topY + 5, mCol[1], mCol[2], mCol[3], "Arial", 10, "bold")
            local currAlt = FlightCore.altiSensor and FlightCore.altiSensor.getHeight() or 0
            gpu.drawText(dispId, string.format("ALT:%.0f->%.0f | B:%.2f", currAlt, FlightCore.state.targetAlt, FlightCore.state.baseThrottle), 75, topY + 5, 200, 230, 255, "Arial", 9, "plain")

            local gridY = topY + topH + 5
            local btnAreaH = sh - gridY - 5
            local rowH = math.floor((btnAreaH - 8) / 3)

            -- Row 1: Target Altitude
            local bW1 = math.floor((sw - 12 - 12) / 4)
            addBtn(6, gridY, bW1, rowH, "+10m", {30, 100, 45}, {240, 255, 240}, function() FlightCore.adjustTargetAlt(10) end)
            addBtn(6 + (bW1+4), gridY, bW1, rowH, "+1m", {45, 130, 55}, {240, 255, 240}, function() FlightCore.adjustTargetAlt(1) end)
            addBtn(6 + (bW1+4)*2, gridY, bW1, rowH, "-1m", {210, 95, 20}, {255, 245, 230}, function() FlightCore.adjustTargetAlt(-1) end)
            addBtn(6 + (bW1+4)*3, gridY, bW1, rowH, "-10m", {190, 40, 40}, {255, 235, 235}, function() FlightCore.adjustTargetAlt(-10) end)

            -- Row 2: Base Throttle
            local r2Y = gridY + rowH + 4
            local bW2 = math.floor((sw - 12 - 12) / 4)
            addBtn(6, r2Y, bW2, rowH, "B +1", {0, 110, 95}, {230, 250, 245}, function() FlightCore.adjustBaseThrottle(1.0) end)
            addBtn(6 + (bW2+4), r2Y, bW2, rowH, "B -1", {55, 70, 80}, {235, 240, 245}, function() FlightCore.adjustBaseThrottle(-1.0) end)
            addBtn(6 + (bW2+4)*2, r2Y, bW2, rowH, "B +.1", {20, 120, 110}, {230, 255, 250}, function() FlightCore.adjustBaseThrottle(0.1) end)
            addBtn(6 + (bW2+4)*3, r2Y, bW2, rowH, "B -.1", {70, 80, 90}, {235, 240, 245}, function() FlightCore.adjustBaseThrottle(-0.1) end)

            -- Row 3: Flight Ops
            local r3Y = r2Y + rowH + 4
            local bW3 = math.floor((sw - 12 - 12) / 4)
            local holdBg = (FlightCore.state.mode == "HOLD_ALT") and {45, 140, 60} or {30, 90, 40}
            addBtn(6, r3Y, bW3, rowH, "HOLD", holdBg, {255, 255, 255}, function() FlightCore.holdAltitude() end)
            addBtn(6 + (bW3+4), r3Y, bW3, rowH, "CALIB", {110, 30, 155}, {250, 230, 255}, function() FlightCore.startCalibration() end)
            addBtn(6 + (bW3+4)*2, r3Y, bW3, rowH, "RE-SCAN", {25, 100, 190}, {230, 245, 255}, function() FlightCore.scanQuadTurtles(); FlightCore.state.statusMsg="Re-scanned!" end)
            local stopBg = (FlightCore.state.mode == "IDLE") and {200, 45, 45} or {160, 30, 30}
            addBtn(6 + (bW3+4)*3, r3Y, bW3, rowH, "STOP", stopBg, {255, 255, 255}, function() FlightCore.stopEngines() end)

        elseif scr.currentView == "NAV" then
            local currAlt = FlightCore.altiSensor and FlightCore.altiSensor.getHeight() or 0
            local currVspeed = FlightCore.altiSensor and FlightCore.altiSensor.getVerticalSpeed() or 0

            local statH = isLarge and 26 or 18
            gpu.fillRect(dispId, 6, headerH + 5, sw - 12, statH, 20, 28, 40)
            gpu.drawText(dispId, string.format("ALT: %.0fm -> TGT: %.0fm (V.S: %+.1f)", currAlt, FlightCore.state.targetAlt, currVspeed), 10, headerH + 7, 80, 230, 255, "Arial", 9, "bold")

            local gridY = headerH + 5 + statH + 5
            local btnAreaH = sh - gridY - 5
            local rowH = math.floor((btnAreaH - 8) / 3)
            local colW = math.floor((sw - 12 - 6) / 2)

            addBtn(6, gridY, colW, rowH, isLarge and "[ 0m LANDING ]" or "0m LAND", {160, 50, 50}, {255, 255, 255}, function() FlightCore.setTargetAlt(0) end)
            addBtn(6 + colW + 6, gridY, colW, rowH, isLarge and "[ 80m TREETOP ]" or "80m TREE", {35, 110, 60}, {255, 255, 255}, function() FlightCore.setTargetAlt(80) end)

            addBtn(6, gridY + rowH + 4, colW, rowH, isLarge and "[ 150m CRUISE ]" or "150m CRZ", {25, 90, 140}, {255, 255, 255}, function() FlightCore.setTargetAlt(150) end)
            addBtn(6 + colW + 6, gridY + rowH + 4, colW, rowH, isLarge and "[ 200m CALIBRATE ]" or "200m CAL", {100, 40, 140}, {255, 255, 255}, function() FlightCore.setTargetAlt(200) end)

            addBtn(6, gridY + (rowH + 4)*2, colW, rowH, isLarge and "[ 300m HIGH-ALT ]" or "300m HIGH", {20, 110, 150}, {255, 255, 255}, function() FlightCore.setTargetAlt(300) end)
            addBtn(6 + colW + 6, gridY + (rowH + 4)*2, colW, rowH, isLarge and "[ LOCK CURRENT ]" or "LOCK CURR", {130, 90, 20}, {255, 255, 255}, function() FlightCore.lockCurrentAlt() end)

        elseif scr.currentView == "SYS" then
            local cardW = math.floor((sw - 18) / 2)
            local cardH = math.floor((sh - headerH - 38) / 2)
            local startY = headerH + 5

            local slots = {
                {slot="FL", col=1, row=1, name="FL Quad"},
                {slot="FR", col=2, row=1, name="FR Quad"},
                {slot="BL", col=1, row=2, name="BL Quad"},
                {slot="BR", col=2, row=2, name="BR Quad"}
            }

            for _, s in ipairs(slots) do
                local cx = 6 + (s.col - 1) * (cardW + 6)
                local cy = startY + (s.row - 1) * (cardH + 4)
                local qH = FlightCore.getQuadHealth(s.slot)
                local online = qH.online > 0
                local bg = online and {20, 35, 30} or {40, 20, 20}
                gpu.fillRect(dispId, cx, cy, cardW, cardH, bg[1], bg[2], bg[3])

                local tagCol = online and {80, 255, 120} or {255, 80, 80}
                gpu.drawText(dispId, string.format("[%s] %d ENG", s.slot, qH.total), cx + 4, cy + 4, 220, 235, 255, "Arial", 9, "bold")
                gpu.drawText(dispId, string.format("ACT:%d/%d | P:%d", qH.online, qH.total, FlightCore.engineOutputs[s.slot]), cx + 4, cy + 16, tagCol[1], tagCol[2], tagCol[3], "Arial", 8, "bold")
            end

            local btmY = startY + cardH * 2 + 6
            local btmW = math.floor((sw - 16) / 2)
            addBtn(6, btmY, btmW, 22, "RE-SCAN HW", {25, 100, 190}, {255, 255, 255}, function() FlightCore.scanQuadTurtles(); FlightCore.state.statusMsg="Re-scanned!" end)
            addBtn(6 + btmW + 4, btmY, btmW, 22, "STOP ALL", {190, 40, 40}, {255, 255, 255}, function() FlightCore.stopEngines() end)
        end

        for _, btn in ipairs(scr.buttons) do
            gpu.fillRect(dispId, btn.x, btn.y, btn.w, btn.h, btn.bg[1], btn.bg[2], btn.bg[3])
            local btnFontSize = math.max(9, math.floor(btn.h * 0.44))
            local tx = btn.x + math.max(2, math.floor((btn.w - #btn.text * (btnFontSize * 0.58)) / 2))
            local ty = btn.y + math.floor((btn.h - btnFontSize) / 2)
            gpu.drawText(dispId, btn.text, tx, ty, btn.fg[1], btn.fg[2], btn.fg[3], "Arial", btnFontSize, "bold")
        end

        gpu.updateDisplay(dispId)
    end,
    handleEvent = function(self, event, p1, p2, p3, p4)
        for _, scr in ipairs(self.screens) do
            local clickX, clickY = nil, nil
            if event == "directgpu_touch" then
                if type(p1) == "number" and type(p2) == "number" then clickX, clickY = p1, p2
                elseif type(p2) == "number" and type(p3) == "number" then clickX, clickY = p2, p3 end
            elseif event == "monitor_touch" then
                if type(p2) == "number" and type(p3) == "number" then
                    local mon = peripheral.find("monitor")
                    local mw, mh = (mon and mon.getSize()) or term.getSize()
                    clickX = math.floor(((p2 - 0.5) / mw) * 320)
                    clickY = math.floor(((p3 - 0.5) / mh) * 240)
                end
            elseif event == "mouse_click" then
                if type(p1) == "table" and p1.x and p1.y then clickX, clickY = p1.x, p1.y
                elseif type(p2) == "number" and type(p3) == "number" then
                    local tw, th = term.getSize()
                    clickX = math.floor(((p2 - 0.5) / tw) * 320)
                    clickY = math.floor(((p3 - 0.5) / th) * 240)
                end
            end

            if clickX and clickY then
                for _, btn in ipairs(scr.buttons) do
                    if clickX >= btn.x and clickX <= btn.x + btn.w and clickY >= btn.y and clickY <= btn.y + btn.h then
                        pcall(btn.action)
                        self:drawScreen(scr)
                        break
                    end
                end
            end
        end
    end
}

-- --------------------------------------------------------
-- 2.2 驅動 B: Tom's Peripherals 全彩點陣向量儀表 (支援多 GPU 螢幕聯動)
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
    ['A'] = {0x7C, 0x12, 0x11, 0x12, 0x7C},
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
    ['a'] = {0x20, 0x54, 0x54, 0x54, 0x78},
    ['b'] = {0x7F, 0x48, 0x44, 0x44, 0x38},
    ['c'] = {0x38, 0x44, 0x44, 0x44, 0x20},
    ['d'] = {0x38, 0x44, 0x44, 0x48, 0x7F},
    ['e'] = {0x38, 0x54, 0x54, 0x54, 0x18},
    ['f'] = {0x08, 0x7E, 0x09, 0x01, 0x02},
    ['g'] = {0x18, 0xA4, 0xA4, 0xA4, 0x7C},
    ['h'] = {0x7F, 0x08, 0x04, 0x04, 0x78},
    ['i'] = {0x00, 0x44, 0x7D, 0x40, 0x00},
    ['j'] = {0x40, 0x80, 0x84, 0x7D, 0x00},
    ['k'] = {0x7F, 0x10, 0x28, 0x44, 0x00},
    ['l'] = {0x00, 0x41, 0x7F, 0x40, 0x00},
    ['m'] = {0x7C, 0x04, 0x18, 0x04, 0x78},
    ['n'] = {0x7C, 0x08, 0x04, 0x04, 0x78},
    ['o'] = {0x38, 0x44, 0x44, 0x44, 0x38},
    ['p'] = {0xFC, 0x24, 0x24, 0x24, 0x18},
    ['q'] = {0x18, 0x24, 0x24, 0x18, 0xFC},
    ['r'] = {0x7C, 0x08, 0x04, 0x04, 0x08},
    ['s'] = {0x48, 0x54, 0x54, 0x54, 0x20},
    ['t'] = {0x04, 0x3F, 0x44, 0x40, 0x20},
    ['u'] = {0x3C, 0x40, 0x40, 0x20, 0x7C},
    ['v'] = {0x1C, 0x20, 0x40, 0x20, 0x1C},
    ['w'] = {0x3C, 0x40, 0x30, 0x40, 0x3C},
    ['x'] = {0x44, 0x28, 0x10, 0x28, 0x44},
    ['y'] = {0x1C, 0xA0, 0xA0, 0xA0, 0x7C},
    ['z'] = {0x44, 0x64, 0x54, 0x4C, 0x44},
    ['.'] = {0x00, 0x60, 0x60, 0x00, 0x00},
    [':'] = {0x00, 0x36, 0x36, 0x00, 0x00},
    ['-'] = {0x08, 0x08, 0x08, 0x08, 0x08},
    ['+'] = {0x08, 0x08, 0x3E, 0x08, 0x08},
    ['/'] = {0x00, 0x60, 0x18, 0x06, 0x00},
    ['%'] = {0x63, 0x33, 0x18, 0x0C, 0x66},
    ['['] = {0x00, 0x7F, 0x41, 0x41, 0x00},
    [']'] = {0x00, 0x41, 0x41, 0x7F, 0x00},
    ['('] = {0x00, 0x1C, 0x22, 0x41, 0x00},
    [')'] = {0x00, 0x41, 0x22, 0x1C, 0x00},
    ['<'] = {0x08, 0x14, 0x22, 0x41, 0x00},
    ['>'] = {0x00, 0x41, 0x22, 0x14, 0x08},
    ['='] = {0x14, 0x14, 0x14, 0x14, 0x14},
    ['|'] = {0x00, 0x00, 0x7F, 0x00, 0x00},
    ['_'] = {0x40, 0x40, 0x40, 0x40, 0x40},
    ['*'] = {0x14, 0x08, 0x3E, 0x08, 0x14},
    ['!'] = {0x00, 0x00, 0x5F, 0x00, 0x00},
    ['?'] = {0x02, 0x01, 0x51, 0x09, 0x06},
    ['#'] = {0x14, 0x7F, 0x14, 0x7F, 0x14},
    [' '] = {0x00, 0x00, 0x00, 0x00, 0x00}
}

Drivers.tom = {
    screens = {},
    init = function(self)
        self.screens = {}
        for _, name in ipairs(peripheral.getNames()) do
            local pType = peripheral.getType(name)
            if pType == "tm_gpu" or pType == "toms_gpu" or pType == "gpu" then
                local gpu = peripheral.wrap(name)
                if gpu and gpu.fill and gpu.rectangle then
                    local w, h = 320, 240
                    local ok, gw, gh = pcall(function() return gpu.getSize() end)
                    if ok and gw and gh and gw > 0 and gh > 0 then w, h = gw, gh end
                    table.insert(self.screens, {
                        id = name,
                        gpu = gpu,
                        screenW = w,
                        screenH = h,
                        currentView = "OVERVIEW",
                        isMenuOpen = false,
                        buttons = {}
                    })
                end
            end
        end
        return #self.screens > 0
    end,
    toARGB = function(self, hexColor)
        if type(hexColor) == "table" then
            local r = hexColor[1] or 0
            local g = hexColor[2] or 0
            local b = hexColor[3] or 0
            local a = hexColor[4] or 255
            local val = a * 16777216 + r * 65536 + g * 256 + b
            if val >= 2147483648 then val = val - 4294967296 end
            return val
        elseif type(hexColor) == "number" then
            local a = 255
            local r = math.floor(hexColor / 65536) % 256
            local g = math.floor(hexColor / 256) % 256
            local b = hexColor % 256
            local val = a * 16777216 + r * 65536 + g * 256 + b
            if val >= 2147483648 then val = val - 4294967296 end
            return val
        end
        return -1
    end,
    drawText = function(self, scr, x, y, text, color, scale)
        scale = scale or 1
        local curX = x
        local argb = self:toARGB(color)
        local sw, sh = scr.screenW, scr.screenH
        local POW2 = { [0]=1, [1]=2, [2]=4, [3]=8, [4]=16, [5]=32, [6]=64, [7]=128 }

        for i = 1, #text do
            local ch = text:sub(i, i)
            local bitmap = FONT_5X7[ch] or FONT_5X7['?']

            if bitmap then
                for col = 1, 5 do
                    local byte = bitmap[col]
                    local px = curX + (col - 1) * scale
                    if px >= 1 and px <= sw then
                        local runStart = nil
                        local runLen = 0
                        for row = 0, 6 do
                            if (math.floor(byte / POW2[row]) % 2) == 1 then
                                if not runStart then runStart = row end
                                runLen = runLen + 1
                            else
                                if runStart then
                                    local py = y + runStart * scale
                                    local ph = runLen * scale
                                    if py <= sh then
                                        pcall(function() scr.gpu.filledRectangle(px, py, scale, ph, argb) end)
                                    end
                                    runStart = nil
                                    runLen = 0
                                end
                            end
                        end
                        if runStart then
                            local py = y + runStart * scale
                            local ph = runLen * scale
                            if py <= sh then
                                pcall(function() scr.gpu.filledRectangle(px, py, scale, ph, argb) end)
                            end
                        end
                    end
                end
            end
            curX = curX + 6 * scale
        end
    end,
    draw = function(self)
        for _, scr in ipairs(self.screens) do
            self:drawScreen(scr)
        end
    end,
    drawScreen = function(self, scr)
        local ok, w, h = pcall(function() return scr.gpu.getSize() end)
        if ok and w and h and w > 0 and h > 0 then scr.screenW, scr.screenH = w, h end
        local sw, sh = scr.screenW, scr.screenH
        local isWide = (sw >= 350)
        local isLarge = (sw >= 220 and sh >= 180)

        local function toARGB(c) return self:toARGB(c) end
        local function sFill(c) pcall(function() scr.gpu.fill(toARGB(c)) end) end
        local function sFR(x, y, bw, bh, c)
            x, y = math.max(1, math.min(sw, x)), math.max(1, math.min(sh, y))
            bw, bh = math.max(1, math.min(sw - x + 1, bw)), math.max(1, math.min(sh - y + 1, bh))
            pcall(function() scr.gpu.filledRectangle(x, y, bw, bh, toARGB(c)) end)
        end
        local function sR(x, y, bw, bh, c)
            x, y = math.max(1, math.min(sw, x)), math.max(1, math.min(sh, y))
            bw, bh = math.max(1, math.min(sw - x + 1, bw)), math.max(1, math.min(sh - y + 1, bh))
            pcall(function() scr.gpu.rectangle(x, y, bw, bh, toARGB(c)) end)
        end
        local function sL(x1, y1, x2, y2, c)
            pcall(function() scr.gpu.line(math.max(1, math.min(sw, x1)), math.max(1, math.min(sh, y1)), math.max(1, math.min(sw, x2)), math.max(1, math.min(sh, y2)), toARGB(c)) end)
        end
        local function sLS(x1, y1, x2, y2, c)
            pcall(function()
                if scr.gpu.lineS then scr.gpu.lineS(math.max(1, math.min(sw, x1)), math.max(1, math.min(sh, y1)), math.max(1, math.min(sw, x2)), math.max(1, math.min(sh, y2)), toARGB(c))
                else scr.gpu.line(math.max(1, math.min(sw, x1)), math.max(1, math.min(sh, y1)), math.max(1, math.min(sw, x2)), math.max(1, math.min(sh, y2)), toARGB(c)) end
            end)
        end
        local function sTxt(x, y, txt, fc, sz)
            self:drawText(scr, x, y, txt, fc, sz or 1)
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
            table.insert(scr.buttons, {x=x, y=y, w=bw, h=bh, text=text, bg=bg, fg=fg, action=act})
        end

        sFill(0x0F141E)
        scr.buttons = {}

        -- 1. 頂部狀態列
        local headerH = isLarge and math.max(22, math.floor(sh * 0.08)) or 16
        sFR(1, 1, sw, headerH, 0x192841)

        local menuBtnW = (sw >= 240) and 50 or 26
        local menuBtnH = headerH - 3
        local menuBtnX = sw - menuBtnW - 3
        local menuBtnY = 2
        if scr.isMenuOpen then
            addBtn(menuBtnX, menuBtnY, menuBtnW, menuBtnH, "[X]", 0x992222, 0xFFFFFF, function() scr.isMenuOpen = false end)
        else
            addBtn(menuBtnX, menuBtnY, menuBtnW, menuBtnH, (sw >= 240) and "[MENU]" or "[=]", 0x224488, 0xFFFFFF, function() scr.isMenuOpen = true end)
        end

        local maxTitleW = menuBtnX - 12
        local viewTitle = scr.isMenuOpen and "SELECT VIEW" or (isLarge and (VIEW_TITLES[scr.currentView] or scr.currentView) or scr.currentView)
        local titleSize = 1
        if isLarge and (sw >= 300) and (#viewTitle * 6 * 2 <= maxTitleW) then
            titleSize = 2
        end
        sTxt(6, math.floor((headerH - 7 * titleSize) / 2) + 1, viewTitle, 0xF0F5FF, titleSize)

        -- 2. 視圖路由
        if scr.isMenuOpen then
            local cardW = math.floor((sw - 16) / 2)
            local cardH = math.floor((sh - headerH - 16) / 3)
            local startY = headerH + 4

            local function addMenuCard(col, row, title, targetView, color)
                local cx = 4 + (col - 1) * (cardW + 6)
                local cy = startY + (row - 1) * (cardH + 4)
                addBtn(cx, cy, cardW, cardH, title, color, 0xFFFFFF, function()
                    scr.currentView = targetView
                    scr.isMenuOpen = false
                end)
            end

            addMenuCard(1, 1, isLarge and "1. OVERVIEW (ALL)" or "1. OVERVIEW", "OVERVIEW", 0x1B3A60)
            addMenuCard(2, 1, isLarge and "2. PFD (FLIGHT)" or "2. PFD", "PFD", 0x145A32)
            addMenuCard(1, 2, isLarge and "3. ECAM (QUAD ENG)" or "3. ECAM", "ECAM", 0x78281F)
            addMenuCard(2, 2, isLarge and "4. CTRL (CONTROLS)" or "4. CTRL", "CTRL", 0x117864)
            addMenuCard(1, 3, isLarge and "5. NAV (PRESETS)" or "5. NAV", "NAV", 0x2471A3)
            addMenuCard(2, 3, isLarge and "6. SYS (DIAGNOSE)" or "6. SYS", "SYS", 0x512E5F)

        elseif scr.currentView == "OVERVIEW" then
            local currAlt = FlightCore.altiSensor and FlightCore.altiSensor.getHeight() or 0
            local currVspeed = FlightCore.altiSensor and FlightCore.altiSensor.getVerticalSpeed() or 0
            local currPitch, currRoll = FlightCore.getGimbalData()

            if isWide then
                -- 超寬大型螢幕模式 (橫排 4 儀表)
                local instY = headerH + 5
                local instH = math.floor((sh - headerH) * 0.52)
                local leftW = math.max(120, math.floor(sw * 0.35))
                local pfdX = 6

                sFR(pfdX, instY, leftW, instH, 0x141A26)
                sR(pfdX, instY, leftW, instH, 0x283850)
                sTxt(pfdX + 6, instY + 4, "PRIMARY FLIGHT", 0xAAD2E6, 1)

                local gR = math.min(30, math.floor(instH * 0.28))
                local gCX = pfdX + math.floor(leftW * 0.26)
                local gCY = instY + math.floor(instH * 0.54)
                for dy = -gR, gR do
                    local dx = math.floor(math.sqrt(math.max(0, gR*gR - dy*dy)) + 0.5)
                    sL(gCX - dx, gCY + dy, gCX + dx, gCY + dy, 0x1E2837)
                end
                sArc(gCX, gCY, gR, 0, 360, 10, 0x50AAF0)
                local altText = string.format("%.0f", currAlt)
                local altSize = (gR >= 26 and isLarge) and 2 or 1
                sTxt(gCX - math.floor(#altText * 3 * altSize), gCY - math.floor(3.5 * altSize), altText, 0xFFFFFF, altSize)

                local textX = pfdX + math.floor(leftW * 0.52)
                local rowSpacing = math.floor((instH - 20) / 4)
                sTxt(textX, instY + 14, string.format("TGT:%.0fm", FlightCore.state.targetAlt), 0x50E6FF, 1)
                sTxt(textX, instY + 14 + rowSpacing, string.format("V.S:%+.2f", currVspeed), 0xFFCD4B, 1)
                local mCol = (FlightCore.state.mode == "HOLD_ALT") and 0x50FF78 or 0xFF5050
                sTxt(textX, instY + 14 + rowSpacing * 2, string.format("MOD:%s", FlightCore.state.mode:sub(1,6)), mCol, 1)
                sTxt(textX, instY + 14 + rowSpacing * 3, string.format("P:%+2.0f R:%+2.0f", currPitch, currRoll), 0xB4DCFF, 1)

                local ecamX = pfdX + leftW + 5
                local ecamW = sw - ecamX - 6
                sFR(ecamX, instY, ecamW, instH, 0x121822)
                sR(ecamX, instY, ecamW, instH, 0x283850)

                local slotW = math.floor((ecamW - 6) / 4)
                local slots = {"FL", "FR", "BL", "BR"}
                local dialR = math.min(math.floor(slotW * 0.33), math.floor(instH * 0.28))
                local engCenterY = instY + math.floor(instH * 0.44)

                for i, slot in ipairs(slots) do
                    local engCenterX = ecamX + 3 + math.floor((i - 0.5) * slotW)
                    local qH = FlightCore.getQuadHealth(slot)
                    local lbl = string.format("%s(%d)", slot, qH.total)

                    sTxt(engCenterX - math.floor(#lbl * 3), engCenterY - dialR - 10, lbl, 0xC8E6FF, 1)
                    sArc(engCenterX, engCenterY, dialR, 210, -30, 10, 0x8CA0B4)
                    sArc(engCenterX, engCenterY, dialR, 10, -30, 10, 0xFF3232)

                    local ratio = math.min(1.0, math.max(0.0, FlightCore.virtualOutputs[slot] / 15.0))
                    local nRad = math.rad(210 - ratio * 240)
                    local nx = math.floor(engCenterX + (dialR - 2) * math.cos(nRad) + 0.5)
                    local ny = math.floor(engCenterY - (dialR - 2) * math.sin(nRad) + 0.5)
                    sLS(engCenterX, engCenterY, nx, ny, 0x32FF64)

                    local boxW = math.max(30, math.floor(dialR * 1.50))
                    local boxH = 12
                    local boxX = engCenterX - math.floor(boxW / 2)
                    local boxY = engCenterY + math.floor(dialR * 0.28)
                    sFR(boxX, boxY, boxW, boxH, 0x0C121C)
                    sR(boxX, boxY, boxW, boxH, 0x2896C8)

                    local valStr = string.format("%4.1f", FlightCore.virtualOutputs[slot])
                    sTxt(boxX + math.floor((boxW - #valStr * 6) / 2), boxY + 3, valStr, 0x46FF78, 1)
                end

                local statY = instY + instH + 4
                local statH = math.max(16, math.floor(sh * 0.065))
                sFR(pfdX, statY, sw - 12, statH, 0x1E2432)
                sR(pfdX, statY, sw - 12, statH, 0x3C4B64)
                sTxt(pfdX + 6, statY + 3, string.format("STATUS: %s", FlightCore.state.statusMsg:sub(1,35)), 0x50E6FF, 1)

                local btnAreaY = statY + statH + 4
                local btnAreaH = sh - btnAreaY - 4
                local rowH = math.floor((btnAreaH - 8) / 3)
                local bY1 = btnAreaY
                local bW1 = math.floor((sw - 12 - 16) / 5)
                addBtn(pfdX, bY1, bW1, rowH, "+10m", 0x1B5E20, 0xE8F5E9, function() FlightCore.adjustTargetAlt(10) end)
                addBtn(pfdX + bW1 + 4, bY1, bW1, rowH, "+1m", 0x2E7D32, 0xE8F5E9, function() FlightCore.adjustTargetAlt(1) end)
                addBtn(pfdX + (bW1 + 4)*2, bY1, bW1, rowH, "-1m", 0xE65100, 0xFFF3E0, function() FlightCore.adjustTargetAlt(-1) end)
                addBtn(pfdX + (bW1 + 4)*3, bY1, bW1, rowH, "-10m", 0xC62828, 0xFFEBEE, function() FlightCore.adjustTargetAlt(-10) end)
                addBtn(pfdX + (bW1 + 4)*4, bY1, bW1, rowH, "LOCK", 0x00838F, 0xE0F7FA, function() FlightCore.lockCurrentAlt() end)

                local bY2 = bY1 + rowH + 4
                local bW2 = math.floor((sw - 12 - 12) / 4)
                addBtn(pfdX, bY2, bW2, rowH, "BASE +", 0x00695C, 0xE0F2F1, function() FlightCore.adjustBaseThrottle(1.0) end)
                addBtn(pfdX + bW2 + 4, bY2, bW2, rowH, "BASE -", 0x37474F, 0xECEFF1, function() FlightCore.adjustBaseThrottle(-1.0) end)
                addBtn(pfdX + (bW2 + 4)*2, bY2, bW2, rowH, "RE-SCAN", 0x1565C0, 0xE3F2FD, function() FlightCore.scanQuadTurtles(); FlightCore.state.statusMsg="Re-scanned!" end)
                addBtn(pfdX + (bW2 + 4)*3, bY2, bW2, rowH, "CALIB", 0x6A1B9A, 0xF3E5F5, function() FlightCore.startCalibration() end)

                local bY3 = bY2 + rowH + 4
                local bW3 = math.floor((sw - 12 - 4) / 2)
                local holdBg = (FlightCore.state.mode == "HOLD_ALT") and 0x2E7D32 or 0x1B5E20
                addBtn(pfdX, bY3, bW3, rowH, "[ HOLD ALT ]", holdBg, 0xFFFFFF, function() FlightCore.holdAltitude() end)
                local stopBg = (FlightCore.state.mode == "IDLE") and 0xC62828 or 0xB71C1C
                addBtn(pfdX + bW3 + 4, bY3, bW3, rowH, "[ STOP / IDLE ]", stopBg, 0xFFFFFF, function() FlightCore.stopEngines() end)
            else
                -- 緊湊/直式多螢幕自適應模式 (2x2 Quad Engine 矩陣排布，徹底杜絕文字儀表擠壓)
                local instY = headerH + 4
                local instH = math.floor((sh - headerH) * 0.54)
                local colW = math.floor((sw - 14) / 2)
                local pfdX = 4
                local ecamX = pfdX + colW + 4

                -- 左側: PFD / 姿態
                sFR(pfdX, instY, colW, instH, 0x141A26)
                sR(pfdX, instY, colW, instH, 0x283850)
                sTxt(pfdX + 4, instY + 3, "ALT/ATT", 0xAAD2E6, 1)

                local gR = math.min(18, math.floor(instH * 0.26))
                local gCX = pfdX + 20
                local gCY = instY + math.floor(instH * 0.60)
                for dy = -gR, gR do
                    local dx = math.floor(math.sqrt(math.max(0, gR*gR - dy*dy)) + 0.5)
                    sL(gCX - dx, gCY + dy, gCX + dx, gCY + dy, 0x1E2837)
                end
                sArc(gCX, gCY, gR, 0, 360, 15, 0x50AAF0)
                local altText = string.format("%.0f", currAlt)
                sTxt(gCX - math.floor(#altText * 3), gCY - 3, altText, 0xFFFFFF, 1)

                local textX = pfdX + 42
                local rowSpacing = math.floor((instH - 16) / 4)
                sTxt(textX, instY + 12, string.format("T:%.0f", FlightCore.state.targetAlt), 0x50E6FF, 1)
                sTxt(textX, instY + 12 + rowSpacing, string.format("V:%+.1f", currVspeed), 0xFFCD4B, 1)
                local mCol = (FlightCore.state.mode == "HOLD_ALT") and 0x50FF78 or 0xFF5050
                sTxt(textX, instY + 12 + rowSpacing * 2, string.format("M:%s", FlightCore.state.mode:sub(1,4)), mCol, 1)
                sTxt(textX, instY + 12 + rowSpacing * 3, string.format("P:%+.0f", currPitch), 0xB4DCFF, 1)

                -- 右側: 2x2 Mini Quad Engines (直觀象限四區塊)
                sFR(ecamX, instY, colW, instH, 0x121822)
                sR(ecamX, instY, colW, instH, 0x283850)

                local subW = math.floor((colW - 6) / 2)
                local subH = math.floor((instH - 8) / 2)
                local miniSlots = {
                    {slot="FL", col=1, row=1},
                    {slot="FR", col=2, row=1},
                    {slot="BL", col=1, row=2},
                    {slot="BR", col=2, row=2}
                }
                for _, ms in ipairs(miniSlots) do
                    local sx = ecamX + 2 + (ms.col - 1) * (subW + 2)
                    local sy = instY + 2 + (ms.row - 1) * (subH + 2)
                    local qH = FlightCore.getQuadHealth(ms.slot)
                    local val = FlightCore.virtualOutputs[ms.slot]
                    sFR(sx, sy, subW, subH, 0x18202E)
                    sR(sx, sy, subW, subH, (qH.online > 0) and 0x284864 or 0x642828)
                    sTxt(sx + 2, sy + 2, string.format("%s %dE", ms.slot, qH.total), 0xC8E6FF, 1)
                    sTxt(sx + 2, sy + math.max(10, subH - 9), string.format("%4.1f", val), 0x46FF78, 1)
                end

                -- 底部狀態列
                local statY = instY + instH + 4
                local statH = 15
                sFR(pfdX, statY, sw - 8, statH, 0x1E2432)
                sR(pfdX, statY, sw - 8, statH, 0x3C4B64)
                local maxStatChars = math.max(6, math.floor((sw - 20) / 6))
                sTxt(pfdX + 4, statY + 4, string.format("STATUS: %s", FlightCore.state.statusMsg:sub(1, maxStatChars)), 0x50E6FF, 1)

                -- 底部按鈕群
                local btnAreaY = statY + statH + 4
                local btnAreaH = sh - btnAreaY - 3
                local rowH = math.floor((btnAreaH - 3) / 2)
                local bW4 = math.floor((sw - 8 - 9) / 4)
                addBtn(pfdX, btnAreaY, bW4, rowH, "+10", 0x1B5E20, 0xFFFFFF, function() FlightCore.adjustTargetAlt(10) end)
                addBtn(pfdX + bW4 + 3, btnAreaY, bW4, rowH, "-10", 0xC62828, 0xFFFFFF, function() FlightCore.adjustTargetAlt(-10) end)
                addBtn(pfdX + (bW4 + 3)*2, btnAreaY, bW4, rowH, "B+", 0x00695C, 0xFFFFFF, function() FlightCore.adjustBaseThrottle(1.0) end)
                addBtn(pfdX + (bW4 + 3)*3, btnAreaY, bW4, rowH, "B-", 0x37474F, 0xFFFFFF, function() FlightCore.adjustBaseThrottle(-1.0) end)

                local bY2 = btnAreaY + rowH + 2
                local bW2 = math.floor((sw - 8 - 3) / 2)
                local holdBg = (FlightCore.state.mode == "HOLD_ALT") and 0x2E7D32 or 0x1B5E20
                addBtn(pfdX, bY2, bW2, rowH, "HOLD", holdBg, 0xFFFFFF, function() FlightCore.holdAltitude() end)
                local stopBg = (FlightCore.state.mode == "IDLE") and 0xC62828 or 0xB71C1C
                addBtn(pfdX + bW2 + 3, bY2, bW2, rowH, "STOP", stopBg, 0xFFFFFF, function() FlightCore.stopEngines() end)
            end

        elseif scr.currentView == "PFD" then
            local currAlt = FlightCore.altiSensor and FlightCore.altiSensor.getHeight() or 0
            local currVspeed = FlightCore.altiSensor and FlightCore.altiSensor.getVerticalSpeed() or 0
            local currPitch, currRoll = FlightCore.getGimbalData()

            local mainH = math.floor((sh - headerH) * (isLarge and 0.70 or 0.62))
            local mainY = headerH + 5

            local leftW = math.floor(sw * 0.52)
            sFR(6, mainY, leftW, mainH, 0x141A26)
            sR(6, mainY, leftW, mainH, 0x283850)

            local hCX = 6 + math.floor(leftW / 2)
            local hCY = mainY + math.floor(mainH / 2)
            local hR = math.min(math.floor(leftW * 0.38), math.floor(mainH * 0.40))
            sArc(hCX, hCY, hR, 0, 360, 10, 0x50AAF0)

            local rollRad = math.rad(-currRoll)
            local lx1 = hCX - math.floor(hR * 0.85 * math.cos(rollRad))
            local ly1 = hCY - math.floor(hR * 0.85 * math.sin(rollRad))
            local lx2 = hCX + math.floor(hR * 0.85 * math.cos(rollRad))
            local ly2 = hCY + math.floor(hR * 0.85 * math.sin(rollRad))
            sLS(lx1, ly1, lx2, ly2, 0xFFD700)
            sFR(hCX - 2, hCY - 2, 5, 5, 0xFF5050)
            sTxt(10, mainY + mainH - 12, string.format("P:%+4.1f* R:%+4.1f*", currPitch, currRoll), 0xC8E6FF, 1)

            local rightX = 6 + leftW + 5
            local rightW = sw - rightX - 6
            sFR(rightX, mainY, rightW, mainH, 0x18202E)
            sR(rightX, mainY, rightW, mainH, 0x283850)

            sTxt(rightX + 6, mainY + 6, "CURRENT ALT", 0x8CA0B4, 1)
            local altStr = string.format("%.1fm", currAlt)
            sTxt(rightX + 6, mainY + 18, altStr, 0xFFFFFF, isLarge and 2 or 1)

            local midY = mainY + (isLarge and 38 or 30)
            sTxt(rightX + 6, midY, string.format("TGT: %.0fm", FlightCore.state.targetAlt), 0x50E6FF, 1)
            sTxt(rightX + 6, midY + 13, string.format("V.S: %+.2f", currVspeed), 0xFFCD4B, 1)

            local bY = mainY + mainH + 5
            local bH = sh - bY - 5
            local bW = math.floor((sw - 12 - 12) / 4)
            addBtn(6, bY, bW, bH, "+10m", 0x1B5E20, 0xFFFFFF, function() FlightCore.adjustTargetAlt(10) end)
            addBtn(6 + (bW+4), bY, bW, bH, "-10m", 0xC62828, 0xFFFFFF, function() FlightCore.adjustTargetAlt(-10) end)
            local holdBg = (FlightCore.state.mode == "HOLD_ALT") and 0x2E7D32 or 0x1B5E20
            addBtn(6 + (bW+4)*2, bY, bW, bH, "HOLD", holdBg, 0xFFFFFF, function() FlightCore.holdAltitude() end)
            local stopBg = (FlightCore.state.mode == "IDLE") and 0xC62828 or 0xB71C1C
            addBtn(6 + (bW+4)*3, bY, bW, bH, "STOP", stopBg, 0xFFFFFF, function() FlightCore.stopEngines() end)

        elseif scr.currentView == "ECAM" then
            -- 2x2 四象限發動機直觀監控
            local topY = headerH + 3
            local btmH = isLarge and 22 or 14
            local areaH = sh - topY - btmH - 4
            local quadW = math.floor((sw - 12) / 2)
            local quadH = math.floor((areaH - 3) / 2)

            local quadDefs = {
                {slot="FL", col=1, row=1, title="[FL] FRONT-LEFT"},
                {slot="FR", col=2, row=1, title="[FR] FRONT-RIGHT"},
                {slot="BL", col=1, row=2, title="[BL] BACK-LEFT"},
                {slot="BR", col=2, row=2, title="[BR] BACK-RIGHT"}
            }

            for _, q in ipairs(quadDefs) do
                local qx = 4 + (q.col - 1) * (quadW + 4)
                local qy = topY + (q.row - 1) * (quadH + 3)
                sFR(qx, qy, quadW, quadH, 0x141C28)
                sR(qx, qy, quadW, quadH, 0x283850)

                local qH = FlightCore.getQuadHealth(q.slot)
                local online = qH.online > 0

                if isLarge and (quadW >= 110) then
                    sTxt(qx + 6, qy + 4, q.title, online and 0xC8E6FF or 0xFF7878, 1)

                    local engCountStr = string.format("ENG:%d", qH.total)
                    sTxt(qx + quadW - math.floor(#engCountStr * 6) - 6, qy + 4, engCountStr, 0x96BEE1, 1)

                    local dialR = math.min(math.floor(quadW * 0.28), math.floor(quadH * 0.28))
                    local engCenterX = qx + math.floor(quadW / 2)
                    local engCenterY = qy + math.floor(quadH * 0.52)

                    sArc(engCenterX, engCenterY, dialR, 210, -30, 10, 0x8CA0B4)
                    sArc(engCenterX, engCenterY, dialR, 10, -30, 10, 0xFF3232)

                    local ratio = math.min(1.0, math.max(0.0, FlightCore.virtualOutputs[q.slot] / 15.0))
                    local nRad = math.rad(210 - ratio * 240)
                    local nx = math.floor(engCenterX + (dialR - 2) * math.cos(nRad) + 0.5)
                    local ny = math.floor(engCenterY - (dialR - 2) * math.sin(nRad) + 0.5)
                    sLS(engCenterX, engCenterY, nx, ny, 0x32FF64)

                    local boxW = math.max(30, math.floor(dialR * 1.50))
                    local boxH = 12
                    local boxX = engCenterX - math.floor(boxW / 2)
                    local boxY = engCenterY + math.floor(dialR * 0.28)
                    sFR(boxX, boxY, boxW, boxH, 0x0C121C)
                    sR(boxX, boxY, boxW, boxH, 0x2896C8)

                    local valStr = string.format("%4.1f", FlightCore.virtualOutputs[q.slot])
                    sTxt(boxX + math.floor((boxW - #valStr * 6) / 2), boxY + 3, valStr, 0x46FF78, 1)

                    local sigStr = string.format("PWM:%d", FlightCore.engineOutputs[q.slot] or 0)
                    sTxt(engCenterX - math.floor(#sigStr * 3), boxY + boxH + 2, sigStr, 0x96BEE1, 1)

                    local statText = string.format("ACT:%d/%d", qH.online, qH.total)
                    if qH.total == 0 then statText = "NO ENG" end
                    sTxt(qx + 6, qy + quadH - 10, statText, (qH.online > 0) and 0x50FF78 or 0xFF5050, 1)
                else
                    -- 緊湊/直立螢幕版
                    local qTitle = string.format("[%s] %dE", q.slot, qH.total)
                    sTxt(qx + 3, qy + 3, qTitle, online and 0xC8E6FF or 0xFF7878, 1)

                    local dialR = math.min(10, math.floor(quadH * 0.22))
                    local engCenterX = qx + math.floor(quadW / 2)
                    local engCenterY = qy + 11 + dialR

                    sArc(engCenterX, engCenterY, dialR, 210, -30, 15, 0x8CA0B4)
                    sArc(engCenterX, engCenterY, dialR, 10, -30, 15, 0xFF3232)

                    local ratio = math.min(1.0, math.max(0.0, FlightCore.virtualOutputs[q.slot] / 15.0))
                    local nRad = math.rad(210 - ratio * 240)
                    local nx = math.floor(engCenterX + (dialR - 1) * math.cos(nRad) + 0.5)
                    local ny = math.floor(engCenterY - (dialR - 1) * math.sin(nRad) + 0.5)
                    sLS(engCenterX, engCenterY, nx, ny, 0x32FF64)

                    local boxW = 24
                    local boxH = 9
                    local boxX = engCenterX - math.floor(boxW / 2)
                    local boxY = engCenterY + math.floor(dialR * 0.25)
                    sFR(boxX, boxY, boxW, boxH, 0x0C121C)
                    sR(boxX, boxY, boxW, boxH, 0x2896C8)

                    local valStr = string.format("%4.1f", FlightCore.virtualOutputs[q.slot])
                    sTxt(boxX + 1, boxY + 1, valStr, 0x46FF78, 1)

                    local btmLineY = qy + quadH - 9
                    sTxt(qx + 3, btmLineY, string.format("P:%d", FlightCore.engineOutputs[q.slot] or 0), 0x96BEE1, 1)
                    local actStr = string.format("%d/%d", qH.online, qH.total)
                    sTxt(qx + quadW - math.floor(#actStr * 6) - 3, btmLineY, actStr, (qH.online > 0) and 0x50FF78 or 0xFF5050, 1)
                end
            end

            local statY = topY + areaH + 2
            sFR(4, statY, sw - 8, btmH, 0x1E2432)
            sR(4, statY, sw - 8, btmH, 0x3C4B64)
            if isLarge then
                sTxt(6, statY + 4, string.format("BASE:%4.2f | PID:BALANCED | %s", FlightCore.state.baseThrottle, FlightCore.state.statusMsg:sub(1, 26)), 0x50E6FF, 1)
            else
                local statChars = math.max(6, math.floor((sw - 60) / 6))
                sTxt(6, statY + 3, string.format("B:%4.2f | %s", FlightCore.state.baseThrottle, FlightCore.state.statusMsg:sub(1, statChars)), 0x50E6FF, 1)
            end

        elseif scr.currentView == "CTRL" then
            local topY = headerH + 5
            local topH = isLarge and 24 or 18
            sFR(6, topY, sw - 12, topH, 0x141E2B)
            sR(6, topY, sw - 12, topH, 0x284864)

            local mCol = (FlightCore.state.mode == "HOLD_ALT") and 0x50FF78 or 0xFF5050
            sTxt(8, topY + 5, string.format("[%s]", FlightCore.state.mode:sub(1,6)), mCol, 1)
            local currAlt = FlightCore.altiSensor and FlightCore.altiSensor.getHeight() or 0
            sTxt(70, topY + 5, string.format("ALT:%.0f->%.0f | B:%.2f", currAlt, FlightCore.state.targetAlt, FlightCore.state.baseThrottle), 0xB4DCFF, 1)

            local gridY = topY + topH + 5
            local btnAreaH = sh - gridY - 5
            local rowH = math.floor((btnAreaH - 8) / 3)

            -- Row 1: Target Altitude
            local bW1 = math.floor((sw - 12 - 12) / 4)
            addBtn(6, gridY, bW1, rowH, "+10m", 0x1B5E20, 0xE8F5E9, function() FlightCore.adjustTargetAlt(10) end)
            addBtn(6 + (bW1+4), gridY, bW1, rowH, "+1m", 0x2E7D32, 0xE8F5E9, function() FlightCore.adjustTargetAlt(1) end)
            addBtn(6 + (bW1+4)*2, gridY, bW1, rowH, "-1m", 0xE65100, 0xFFF3E0, function() FlightCore.adjustTargetAlt(-1) end)
            addBtn(6 + (bW1+4)*3, gridY, bW1, rowH, "-10m", 0xC62828, 0xFFEBEE, function() FlightCore.adjustTargetAlt(-10) end)

            -- Row 2: Base Throttle
            local r2Y = gridY + rowH + 4
            local bW2 = math.floor((sw - 12 - 12) / 4)
            addBtn(6, r2Y, bW2, rowH, "B +1", 0x00695C, 0xE0F2F1, function() FlightCore.adjustBaseThrottle(1.0) end)
            addBtn(6 + (bW2+4), r2Y, bW2, rowH, "B -1", 0x37474F, 0xECEFF1, function() FlightCore.adjustBaseThrottle(-1.0) end)
            addBtn(6 + (bW2+4)*2, r2Y, bW2, rowH, "B +.1", 0x00897B, 0xE0F2F1, function() FlightCore.adjustBaseThrottle(0.1) end)
            addBtn(6 + (bW2+4)*3, r2Y, bW2, rowH, "B -.1", 0x455A64, 0xECEFF1, function() FlightCore.adjustBaseThrottle(-0.1) end)

            -- Row 3: Flight Ops
            local r3Y = r2Y + rowH + 4
            local bW3 = math.floor((sw - 12 - 12) / 4)
            local holdBg = (FlightCore.state.mode == "HOLD_ALT") and 0x2E7D32 or 0x1B5E20
            addBtn(6, r3Y, bW3, rowH, "HOLD", holdBg, 0xFFFFFF, function() FlightCore.holdAltitude() end)
            addBtn(6 + (bW3+4), r3Y, bW3, rowH, "CALIB", 0x6A1B9A, 0xF3E5F5, function() FlightCore.startCalibration() end)
            addBtn(6 + (bW3+4)*2, r3Y, bW3, rowH, "RE-SCAN", 0x1565C0, 0xE3F2FD, function() FlightCore.scanQuadTurtles(); FlightCore.state.statusMsg="Re-scanned!" end)
            local stopBg = (FlightCore.state.mode == "IDLE") and 0xC62828 or 0xB71C1C
            addBtn(6 + (bW3+4)*3, r3Y, bW3, rowH, "STOP", stopBg, 0xFFFFFF, function() FlightCore.stopEngines() end)

        elseif scr.currentView == "NAV" then
            local currAlt = FlightCore.altiSensor and FlightCore.altiSensor.getHeight() or 0
            local currVspeed = FlightCore.altiSensor and FlightCore.altiSensor.getVerticalSpeed() or 0

            local statH = isLarge and 24 or 18
            sFR(6, headerH + 5, sw - 12, statH, 0x141E28)
            sR(6, headerH + 5, sw - 12, statH, 0x284864)
            sTxt(8, headerH + 8, string.format("ALT:%.0fm -> TGT:%.0fm (V.S:%+.1f)", currAlt, FlightCore.state.targetAlt, currVspeed), 0x50E6FF, 1)

            local gridY = headerH + 5 + statH + 5
            local btnAreaH = sh - gridY - 5
            local rowH = math.floor((btnAreaH - 8) / 3)
            local colW = math.floor((sw - 12 - 6) / 2)

            addBtn(6, gridY, colW, rowH, isLarge and "[ 0m LANDING ]" or "0m LAND", 0xA93226, 0xFFFFFF, function() FlightCore.setTargetAlt(0) end)
            addBtn(6 + colW + 6, gridY, colW, rowH, isLarge and "[ 80m TREETOP ]" or "80m TREE", 0x1E8449, 0xFFFFFF, function() FlightCore.setTargetAlt(80) end)

            addBtn(6, gridY + rowH + 4, colW, rowH, isLarge and "[ 150m CRUISE ]" or "150m CRZ", 0x2471A3, 0xFFFFFF, function() FlightCore.setTargetAlt(150) end)
            addBtn(6 + colW + 6, gridY + rowH + 4, colW, rowH, isLarge and "[ 200m CALIBRATE ]" or "200m CAL", 0x7D3C98, 0xFFFFFF, function() FlightCore.setTargetAlt(200) end)

            addBtn(6, gridY + (rowH + 4)*2, colW, rowH, isLarge and "[ 300m HIGH-ALT ]" or "300m HIGH", 0x17A589, 0xFFFFFF, function() FlightCore.setTargetAlt(300) end)
            addBtn(6 + colW + 6, gridY + (rowH + 4)*2, colW, rowH, isLarge and "[ LOCK CURRENT ]" or "LOCK CURR", 0xB7950B, 0xFFFFFF, function() FlightCore.lockCurrentAlt() end)

        elseif scr.currentView == "SYS" then
            local cardW = math.floor((sw - 18) / 2)
            local cardH = math.floor((sh - headerH - 38) / 2)
            local startY = headerH + 5

            local slots = {
                {slot="FL", col=1, row=1, name="FL Quad"},
                {slot="FR", col=2, row=1, name="FR Quad"},
                {slot="BL", col=1, row=2, name="BL Quad"},
                {slot="BR", col=2, row=2, name="BR Quad"}
            }

            for _, s in ipairs(slots) do
                local cx = 6 + (s.col - 1) * (cardW + 6)
                local cy = startY + (s.row - 1) * (cardH + 4)
                local qH = FlightCore.getQuadHealth(s.slot)
                local online = qH.online > 0
                sFR(cx, cy, cardW, cardH, online and 0x14281E or 0x281414)
                sR(cx, cy, cardW, cardH, online and 0x28643C or 0x642828)

                sTxt(cx + 4, cy + 4, string.format("[%s] %d ENG", s.slot, qH.total), 0xF0F5FF, 1)
                sTxt(cx + 4, cy + 16, string.format("ACT: %d/%d | P:%d", qH.online, qH.total, FlightCore.engineOutputs[s.slot]), online and 0x50FF78 or 0xFF5050, 1)
            end

            local btmY = startY + cardH * 2 + 6
            local btmW = math.floor((sw - 16) / 2)
            addBtn(6, btmY, btmW, 22, "RE-SCAN HW", 0x1565C0, 0xFFFFFF, function() FlightCore.scanQuadTurtles(); FlightCore.state.statusMsg="Re-scanned!" end)
            addBtn(6 + btmW + 4, btmY, btmW, 22, "STOP ALL", 0xC62828, 0xFFFFFF, function() FlightCore.stopEngines() end)
        end

        local btnTextSize = 1
        for _, btn in ipairs(scr.buttons) do
            sFR(btn.x, btn.y, btn.w, btn.h, btn.bg)
            sR(btn.x, btn.y, btn.w, btn.h, 0x8CA0B4)
            local tx = btn.x + math.max(2, math.floor((btn.w - #btn.text * 6 * btnTextSize) / 2))
            local ty = btn.y + math.floor((btn.h - 7 * btnTextSize) / 2)
            sTxt(tx, ty, btn.text, btn.fg, btnTextSize)
        end

        pcall(function() if scr.gpu.sync then scr.gpu.sync() end end)
    end,
    handleEvent = function(self, event, p1, p2, p3, p4)
        local targetScreen = nil
        local clickX, clickY = nil, nil

        if event == "tm_monitor_touch" then
            for _, scr in ipairs(self.screens) do
                if scr.id == p1 then targetScreen = scr; break end
            end
            if not targetScreen and #self.screens > 0 then targetScreen = self.screens[1] end

            if type(p1) == "number" and type(p2) == "number" then clickX, clickY = p1, p2
            elseif type(p2) == "number" and type(p3) == "number" then clickX, clickY = p2, p3 end

        elseif event == "monitor_touch" then
            for _, scr in ipairs(self.screens) do
                if scr.id == p1 then targetScreen = scr; break end
            end
            if not targetScreen and #self.screens > 0 then targetScreen = self.screens[1] end

            if type(p2) == "number" and type(p3) == "number" and targetScreen then
                local mon = peripheral.find("monitor")
                local mw, mh = (mon and mon.getSize()) or term.getSize()
                clickX = math.floor(((p2 - 0.5) / mw) * (targetScreen.screenW or 320))
                clickY = math.floor(((p3 - 0.5) / mh) * (targetScreen.screenH or 240))
            end

        elseif event == "mouse_click" then
            targetScreen = self.screens[1]
            if type(p2) == "number" and type(p3) == "number" and targetScreen then
                local tw, th = term.getSize()
                clickX = math.floor(((p2 - 0.5) / tw) * (targetScreen.screenW or 320))
                clickY = math.floor(((p3 - 0.5) / th) * (targetScreen.screenH or 240))
            end
        end

        if targetScreen and clickX and clickY then
            for _, btn in ipairs(targetScreen.buttons) do
                if clickX >= btn.x and clickX <= btn.x + btn.w and clickY >= btn.y and clickY <= btn.y + btn.h then
                    pcall(btn.action)
                    self:drawScreen(targetScreen)
                    break
                end
            end
        end
    end
}

-- --------------------------------------------------------
-- 2.3 驅動 C: CC: Tweaked 原生螢幕/終端機 (文字終端與原生 Monitor)
-- --------------------------------------------------------
Drivers.normal = {
    screens = {},
    init = function(self)
        self.screens = {}
        for _, name in ipairs(peripheral.getNames()) do
            if peripheral.getType(name) == "monitor" then
                local mon = peripheral.wrap(name)
                if mon then
                    mon.setTextScale(0.5)
                    local mw, mh = mon.getSize()
                    table.insert(self.screens, {
                        device = mon,
                        isMonitor = true,
                        screenW = mw,
                        screenH = mh,
                        currentView = "OVERVIEW",
                        isMenuOpen = false,
                        buttons = {}
                    })
                end
            end
        end

        if #self.screens == 0 then
            local tw, th = term.getSize()
            table.insert(self.screens, {
                device = term.current(),
                isMonitor = false,
                screenW = tw,
                screenH = th,
                currentView = "OVERVIEW",
                isMenuOpen = false,
                buttons = {}
            })
        end
        return true
    end,
    safeBlit = function(self, scr, x, y, text, fg, bg)
        local t = scr.device
        local w, h = scr.screenW, scr.screenH
        if y < 1 or y > h or x > w then return end
        if x < 1 then
            local cut = 1 - x
            if cut >= #text then return end
            text = text:sub(cut + 1)
            fg = fg:sub(cut + 1)
            bg = bg:sub(cut + 1)
            x = 1
        end
        if x + #text - 1 > w then
            local maxLen = w - x + 1
            text = text:sub(1, maxLen)
            fg = fg:sub(1, maxLen)
            bg = bg:sub(1, maxLen)
        end
        if #text > 0 then
            t.setCursorPos(x, y)
            t.blit(text, fg, bg)
        end
    end,
    draw = function(self)
        for _, scr in ipairs(self.screens) do
            self:drawScreen(scr)
        end
    end,
    drawScreen = function(self, scr)
        local t = scr.device
        local w, h = t.getSize()
        scr.screenW, scr.screenH = w, h
        local isLarge = (w >= 36 and h >= 18)

        t.setBackgroundColor(colors.black)
        t.clear()
        scr.buttons = {}

        local function addBtn(x, y, bw, bh, text, fg, bg, act)
            table.insert(scr.buttons, {x=x, y=y, w=bw, h=bh, text=text, fg=fg, bg=bg, action=act})
        end

        -- 1. 頂部導航列
        local menuBtnW = isLarge and 8 or 5
        local menuBtnX = w - menuBtnW + 1
        self:safeBlit(scr, 1, 1, string.rep(" ", w), "0", "b")

        local viewTitle = scr.isMenuOpen and "SELECT VIEW" or (isLarge and (VIEW_TITLES[scr.currentView] or scr.currentView) or scr.currentView)
        self:safeBlit(scr, 2, 1, viewTitle, "0", "b")

        if scr.isMenuOpen then
            addBtn(menuBtnX, 1, menuBtnW, 1, isLarge and "[CLOSE]" or "[X]", "0", "e", function() scr.isMenuOpen = false end)
        else
            addBtn(menuBtnX, 1, menuBtnW, 1, isLarge and "[ MENU ]" or "[=]", "0", "9", function() scr.isMenuOpen = true end)
        end

        -- 2. 視圖分流
        if scr.isMenuOpen then
            local startY = 3
            local cardW = math.floor((w - 3) / 2)
            local cardH = math.max(2, math.floor((h - startY - 2) / 3))

            local function addMenuCard(col, row, title, targetView, fg, bg)
                local cx = 2 + (col - 1) * (cardW + 1)
                local cy = startY + (row - 1) * (cardH + 1)
                addBtn(cx, cy, cardW, cardH, title, fg, bg, function()
                    scr.currentView = targetView
                    scr.isMenuOpen = false
                end)
            end

            addMenuCard(1, 1, isLarge and "1. OVERVIEW (ALL)" or "1. OVERVIEW", "OVERVIEW", "0", "9")
            addMenuCard(2, 1, isLarge and "2. PFD (FLIGHT)" or "2. PFD", "PFD", "0", "5")
            addMenuCard(1, 2, isLarge and "3. ECAM (QUAD ENG)" or "3. ECAM", "ECAM", "0", "e")
            addMenuCard(2, 2, isLarge and "4. CTRL (CONTROLS)" or "4. CTRL", "CTRL", "0", "3")
            addMenuCard(1, 3, isLarge and "5. NAV (PRESETS)" or "5. NAV", "NAV", "0", "b")
            addMenuCard(2, 3, isLarge and "6. SYS (DIAGNOSE)" or "6. SYS", "SYS", "0", "a")

        elseif scr.currentView == "OVERVIEW" then
            local currAlt = FlightCore.altiSensor and FlightCore.altiSensor.getHeight() or 0
            local currVspeed = FlightCore.altiSensor and FlightCore.altiSensor.getVerticalSpeed() or 0
            local currPitch, currRoll = FlightCore.getGimbalData()

            self:safeBlit(scr, 2, 3, string.format("ALT: %6.1fm | TGT: %4.0fm | V.S: %+5.2f", currAlt, FlightCore.state.targetAlt, currVspeed), "0", "b")
            self:safeBlit(scr, 2, 4, string.format("MODE: [%s] | PITCH: %+2.0f* | ROLL: %+2.0f*", FlightCore.state.mode, currPitch, currRoll), "0", "8")

            local gridY = 6
            local quadW = math.floor((w - 3) / 2)
            local quadH = math.max(2, math.floor((h - gridY - 5) / 2))

            local quadDefs = {
                {slot="FL", col=1, row=1, title="[FL] FRONT-LEFT"},
                {slot="FR", col=2, row=1, title="[FR] FRONT-RIGHT"},
                {slot="BL", col=1, row=2, title="[BL] BACK-LEFT"},
                {slot="BR", col=2, row=2, title="[BR] BACK-RIGHT"}
            }

            for _, q in ipairs(quadDefs) do
                local qx = 2 + (q.col - 1) * (quadW + 1)
                local qy = gridY + (q.row - 1) * (quadH + 1)
                local qH = FlightCore.getQuadHealth(q.slot)
                local val = FlightCore.virtualOutputs[q.slot]

                for r = 0, quadH - 1 do
                    self:safeBlit(scr, qx, qy + r, string.rep(" ", quadW), "0", "7")
                end
                self:safeBlit(scr, qx + 1, qy, string.format("%s (%d)", q.slot, qH.total), "9", "8")
                self:safeBlit(scr, qx + 1, qy + 1, string.format("THR: %4.1f | P:%d", val, FlightCore.engineOutputs[q.slot] or 0), "5", "7")
            end

            local btmY = h - 3
            self:safeBlit(scr, 2, btmY, string.format("BASE: %4.2f/15 | %s", FlightCore.state.baseThrottle, FlightCore.state.statusMsg:sub(1, w - 20)), "0", "b")

            local btnY = h - 1
            local btnW = math.floor((w - 5) / 4)
            addBtn(2, btnY, btnW, 2, "+10m", "0", "5", function() FlightCore.adjustTargetAlt(10) end)
            addBtn(3 + btnW, btnY, btnW, 2, "-10m", "0", "e", function() FlightCore.adjustTargetAlt(-10) end)
            addBtn(4 + btnW*2, btnY, btnW, 2, "HOLD", "0", "3", function() FlightCore.holdAltitude() end)
            addBtn(5 + btnW*3, btnY, btnW, 2, "STOP", "0", "e", function() FlightCore.stopEngines() end)

        elseif scr.currentView == "PFD" then
            local currAlt = FlightCore.altiSensor and FlightCore.altiSensor.getHeight() or 0
            local currVspeed = FlightCore.altiSensor and FlightCore.altiSensor.getVerticalSpeed() or 0
            local currPitch, currRoll = FlightCore.getGimbalData()

            self:safeBlit(scr, 2, 3, string.format("ALT: %6.1fm | TARGET: %6.1fm", currAlt, FlightCore.state.targetAlt), "0", "b")
            self:safeBlit(scr, 2, 4, string.format("VERTICAL SPEED: %+5.2f m/s", currVspeed), "0", "8")
            self:safeBlit(scr, 2, 5, string.format("PITCH: %+4.1f* | ROLL: %+4.1f*", currPitch, currRoll), "0", "8")

            local btmY = h - 1
            local btnW = math.floor((w - 5) / 4)
            addBtn(2, btmY, btnW, 2, "+10m", "0", "5", function() FlightCore.adjustTargetAlt(10) end)
            addBtn(3 + btnW, btmY, btnW, 2, "-10m", "0", "e", function() FlightCore.adjustTargetAlt(-10) end)
            addBtn(4 + btnW*2, btmY, btnW, 2, "HOLD", "0", "3", function() FlightCore.holdAltitude() end)
            addBtn(5 + btnW*3, btnY, btnW, 2, "STOP", "0", "e", function() FlightCore.stopEngines() end)

        elseif scr.currentView == "ECAM" then
            local gridY = 3
            local quadW = math.floor((w - 3) / 2)
            local quadH = math.max(3, math.floor((h - gridY - 2) / 2))

            local quadDefs = {
                {slot="FL", col=1, row=1, title="[FL] FRONT-LEFT"},
                {slot="FR", col=2, row=1, title="[FR] FRONT-RIGHT"},
                {slot="BL", col=1, row=2, title="[BL] BACK-LEFT"},
                {slot="BR", col=2, row=2, title="[BR] BACK-RIGHT"}
            }

            for _, q in ipairs(quadDefs) do
                local qx = 2 + (q.col - 1) * (quadW + 1)
                local qy = gridY + (q.row - 1) * (quadH + 1)
                local qH = FlightCore.getQuadHealth(q.slot)
                local val = FlightCore.virtualOutputs[q.slot]
                local sig = FlightCore.engineOutputs[q.slot]

                for r = 0, quadH - 1 do
                    self:safeBlit(scr, qx, qy + r, string.rep(" ", quadW), "0", "7")
                end
                self:safeBlit(scr, qx + 1, qy, string.format("%s (%d)", q.slot, qH.total), "9", "8")
                self:safeBlit(scr, qx + 1, qy + 1, string.format("THR: %4.1f", val), "5", "7")
                self:safeBlit(scr, qx + 1, qy + 2, string.format("PWM: %2d/15", sig), "3", "7")
                if quadH >= 4 then
                    self:safeBlit(scr, qx + 1, qy + 3, string.format("ACT: %d/%d", qH.online, qH.total), (qH.online > 0) and "5" or "e", "7")
                end
            end

            self:safeBlit(scr, 2, h, string.format("BASE: %4.2f/15 | %s", FlightCore.state.baseThrottle, FlightCore.state.statusMsg:sub(1, w - 20)), "0", "b")

        elseif scr.currentView == "CTRL" then
            local currAlt = FlightCore.altiSensor and FlightCore.altiSensor.getHeight() or 0
            self:safeBlit(scr, 2, 3, string.format("[%s] ALT:%.0f->%.0f | B:%.2f", FlightCore.state.mode:sub(1,6)), currAlt, FlightCore.state.targetAlt, FlightCore.state.baseThrottle), "0", "b")

            local rowH = math.max(2, math.floor((h - 7) / 3))

            -- Row 1: Target Alt
            local bY1 = 5
            local bW1 = math.floor((w - 5) / 4)
            addBtn(2, bY1, bW1, rowH, "+10m", "0", "5", function() FlightCore.adjustTargetAlt(10) end)
            addBtn(3 + bW1, bY1, bW1, rowH, "+1m", "0", "5", function() FlightCore.adjustTargetAlt(1) end)
            addBtn(4 + bW1*2, bY1, bW1, rowH, "-1m", "0", "e", function() FlightCore.adjustTargetAlt(-1) end)
            addBtn(5 + bW1*3, bY1, bW1, rowH, "-10m", "0", "e", function() FlightCore.adjustTargetAlt(-10) end)

            -- Row 2: Throttle
            local bY2 = bY1 + rowH + 1
            local bW2 = math.floor((w - 5) / 4)
            addBtn(2, bY2, bW2, rowH, "B +1", "3", "0", function() FlightCore.adjustBaseThrottle(1.0) end)
            addBtn(3 + bW2, bY2, bW2, rowH, "B -1", "9", "0", function() FlightCore.adjustBaseThrottle(-1.0) end)
            addBtn(4 + bW2*2, bY2, bW2, rowH, "B +.1", "b", "0", function() FlightCore.adjustBaseThrottle(0.1) end)
            addBtn(5 + bW2*3, bY2, bW2, rowH, "B -.1", "7", "0", function() FlightCore.adjustBaseThrottle(-0.1) end)

            -- Row 3: Flight Ops
            local bY3 = bY2 + rowH + 1
            local bW3 = math.floor((w - 5) / 4)
            addBtn(2, bY3, bW3, rowH, "HOLD", "5", "0", function() FlightCore.holdAltitude() end)
            addBtn(3 + bW3, bY3, bW3, rowH, "CALIB", "a", "0", function() FlightCore.startCalibration() end)
            addBtn(4 + bW3*2, bY3, bW3, rowH, "RE-SCAN", "b", "0", function() FlightCore.scanQuadTurtles(); FlightCore.state.statusMsg="Re-scanned!" end)
            addBtn(5 + bW3*3, bY3, bW3, rowH, "STOP", "e", "0", function() FlightCore.stopEngines() end)

        elseif scr.currentView == "NAV" then
            local currAlt = FlightCore.altiSensor and FlightCore.altiSensor.getHeight() or 0
            self:safeBlit(scr, 2, 3, string.format("ALT: %6.1fm -> TGT: %6.1fm", currAlt, FlightCore.state.targetAlt), "0", "b")

            local gridY = 5
            local colW = math.floor((w - 3) / 2)
            local rowH = math.max(2, math.floor((h - gridY - 2) / 3))

            addBtn(2, gridY, colW, rowH, isLarge and "[ 0m LANDING ]" or "0m LAND", "e", "0", function() FlightCore.setTargetAlt(0) end)
            addBtn(3 + colW, gridY, colW, rowH, isLarge and "[ 80m TREETOP ]" or "80m TREE", "5", "0", function() FlightCore.setTargetAlt(80) end)

            addBtn(2, gridY + rowH + 1, colW, rowH, isLarge and "[ 150m CRUISE ]" or "150m CRZ", "3", "0", function() FlightCore.setTargetAlt(150) end)
            addBtn(3 + colW, gridY + rowH + 1, colW, rowH, isLarge and "[ 200m CALIBRATE ]" or "200m CAL", "a", "0", function() FlightCore.setTargetAlt(200) end)

            addBtn(2, gridY + (rowH + 1)*2, colW, rowH, isLarge and "[ 300m HIGH-ALT ]" or "300m HIGH", "b", "0", function() FlightCore.setTargetAlt(300) end)
            addBtn(3 + colW, gridY + (rowH + 1)*2, colW, rowH, isLarge and "[ LOCK CURRENT ]" or "LOCK CURR", "4", "0", function() FlightCore.lockCurrentAlt() end)

        elseif scr.currentView == "SYS" then
            local slots = {
                {slot="FL", col=1, row=1, name="FL Quad"},
                {slot="FR", col=2, row=1, name="FR Quad"},
                {slot="BL", col=1, row=2, name="BL Quad"},
                {slot="BR", col=2, row=2, name="BR Quad"}
            }
            local cardW = math.floor((w - 3) / 2)
            local cardH = 3
            local startY = 4

            for _, s in ipairs(slots) do
                local cx = 2 + (s.col - 1) * (cardW + 1)
                local cy = startY + (s.row - 1) * (cardH + 1)
                local qH = FlightCore.getQuadHealth(s.slot)
                local bg = (qH.online > 0) and "5" or "e"

                for r = 0, cardH - 1 do
                    self:safeBlit(scr, cx, cy + r, string.rep(" ", cardW), "0", bg)
                end
                self:safeBlit(scr, cx + 1, cy, string.format("[%s] %d ENG", s.slot, qH.total), "0", bg)
                self:safeBlit(scr, cx + 1, cy + 1, string.format("ACT: %d/%d", qH.online, qH.total), "0", bg)
            end

            local btmY = h - 2
            local btmW = math.floor((w - 3) / 2)
            addBtn(2, btmY, btmW, 2, "RE-SCAN HW", "3", "0", function() FlightCore.scanQuadTurtles(); FlightCore.state.statusMsg="Re-scanned!" end)
            addBtn(3 + btmW, btmY, btmW, 2, "STOP ALL", "e", "0", function() FlightCore.stopEngines() end)
        end

        for _, btn in ipairs(scr.buttons) do
            for dy = 0, btn.h - 1 do
                self:safeBlit(scr, btn.x, btn.y + dy, string.rep(" ", btn.w), btn.fg, btn.bg)
            end
            local textX = btn.x + math.max(0, math.floor((btn.w - #btn.text) / 2))
            local textY = btn.y + math.floor((btn.h - 1) / 2)
            self:safeBlit(scr, textX, textY, btn.text, btn.fg, btn.bg)
        end
    end,
    handleEvent = function(self, event, p1, p2, p3, p4)
        for _, scr in ipairs(self.screens) do
            local clickX, clickY = nil, nil
            if event == "monitor_touch" then
                if scr.isMonitor and peripheral.getName(scr.device) == p1 then
                    clickX, clickY = p2, p3
                end
            elseif event == "mouse_click" then
                if not scr.isMonitor then
                    clickX, clickY = p2, p3
                end
            end

            if clickX and clickY then
                for _, btn in ipairs(scr.buttons) do
                    if clickX >= btn.x and clickX <= btn.x + btn.w - 1 and clickY >= btn.y and clickY <= btn.y + btn.h - 1 then
                        pcall(btn.action)
                        self:drawScreen(scr)
                        break
                    end
                end
            end
        end
    end
}

-- ========================================================
-- PART 3: 主事件循環與初始化 (SYSTEM ORCHESTRATOR)
-- ========================================================
local function selectBestDriver()
    if requestedDriver == "directgpu" then
        if Drivers.direct:init() then return Drivers.direct, "CC-DirectGPU-Mod (Forced)" end
    elseif requestedDriver == "tom" or requestedDriver == "toms" then
        if Drivers.tom:init() then return Drivers.tom, "Tom's Peripherals GPU (Forced)" end
    elseif requestedDriver == "normal" or requestedDriver == "monitor" then
        Drivers.normal:init()
        return Drivers.normal, "CC Native Monitor/Terminal (Forced)"
    end

    if Drivers.direct:init() then return Drivers.direct, "CC-DirectGPU-Mod (Hardware Fast)" end
    if Drivers.tom:init() then return Drivers.tom, "Tom's Peripherals GPU (Full Color)" end
    Drivers.normal:init()
    return Drivers.normal, "CC Native Monitor/Terminal (Standard)"
end

local activeDriver, driverName = selectBestDriver()

term.clear()
term.setCursorPos(1, 1)
print(string.format("== VTOL Airship Flight Computer [%s] ==", VERSION))
print("Driver Activated: " .. driverName)
print("Screens Attached: " .. tostring(#activeDriver.screens))
print("---------------------------------------------")

FlightCore.scanQuadTurtles()
print(string.format("FL Engines: %d node(s)", #FlightCore.engines.FL))
print(string.format("FR Engines: %d node(s)", #FlightCore.engines.FR))
print(string.format("BL Engines: %d node(s)", #FlightCore.engines.BL))
print(string.format("BR Engines: %d node(s)", #FlightCore.engines.BR))
print("---------------------------------------------")
print("Flight Computer Running. Press Ctrl+T to Terminate.")

local function flightLoop()
    while true do
        FlightCore.processModemMessages()
        FlightCore.updateFlightLogic()
        sleep(0.05)
    end
end

local function renderLoop()
    while true do
        activeDriver:draw()
        sleep(0.05)
    end
end

local function eventLoop()
    while true do
        local eventData = {os.pullEvent()}
        local event = eventData[1]
        activeDriver:handleEvent(table.unpack(eventData))
        if event == "terminate" then
            FlightCore.stopEngines()
            break
        end
    end
end

parallel.waitForAny(flightLoop, renderLoop, eventLoop)
