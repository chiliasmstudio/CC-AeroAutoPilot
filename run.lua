--[[
    Create: Avionics & CC: Tweaked
    Unified Multi-Engine Avionics Flight Computer (多軸模組化統一飛控大腦)
    Version: v3.8.2 Modular Bundle
    
    螢幕尺寸自適應分類 (Dual Screen Size Mode):
    - 完整顯示螢幕 (>= 5x5): 啟動超大 A350 儀表、細緻多引擎遙測與 3 排完整控制面板。
    - 精簡版本螢幕 (3x3 ~ 4x5, 如 3x3, 4x4, 5x4, 4x5): 啟動 2x2 四象限精簡佈局，高度防重疊排版與雙排按鈕。

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

local VERSION = "v3.8.2"
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
FlightCore.modems = {}
FlightCore.outputSides = {"bottom", "top", "left", "right", "back"}
FlightCore.pwmTick = 0

local function matchPrefix(lbl, prefix)
    local u = string.upper(lbl or "")
    return u == prefix or u:sub(1, #prefix + 1) == (prefix .. "_") or u:sub(1, #prefix + 1) == (prefix .. "-") or u:sub(1, #prefix + 1) == (prefix .. " ")
end

function FlightCore.initSensorsAndModem()
    FlightCore.altiSensor = peripheral.find("altitude_sensor")
    FlightCore.gimbalSensor = peripheral.find("gimbal_sensor")
    FlightCore.modems = {}
    for _, name in ipairs(peripheral.getNames()) do
        if peripheral.getType(name) == "modem" then
            local m = peripheral.wrap(name)
            if m then
                pcall(function() m.open(101) end)
                table.insert(FlightCore.modems, m)
            end
        end
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

-- 非阻塞數據機訊息處理器 (由主事件循環呼叫，絕不在 flightLoop 中調用阻塞 pullEvent)
function FlightCore.handleModemMessage(side, ch, replyCh, msg, dist)
    if ch == 101 and type(msg) == "table" then
        if msg.type == "HEARTBEAT" and msg.role and msg.id then
            FlightCore.turtles[msg.id] = {
                role = msg.role,
                label = msg.label or ("Turtle-" .. msg.id),
                sig = msg.sig or 0,
                ver = msg.ver or "unknown",
                lastSeen = os.epoch("utc")
            }
        end
    end
end

function FlightCore.outputToEngines(sigFL, sigFR, sigBL, sigBR)
    local targets = { FL = sigFL, FR = sigFR, BL = sigBL, BR = sigBR }

    -- 1. 廣播至所有已連接之 Wired/Wireless Modem (Channel 100)
    local payload = {
        type = "FLIGHT_SYNC",
        FL = sigFL,
        FR = sigFR,
        BL = sigBL,
        BR = sigBR,
        timestamp = os.epoch("utc")
    }
    for _, m in ipairs(FlightCore.modems) do
        pcall(function() m.transmit(100, 101, payload) end)
    end

    -- 2. 直接輸出電腦本體所有側面的類比紅石訊號 (以防本體直連紅石)
    local maxSig = math.max(sigFL, sigFR, sigBL, sigBR)
    for _, s in ipairs({"top", "bottom", "left", "right", "back", "front"}) do
        pcall(function() redstone.setAnalogOutput(s, maxSig) end)
    end

    -- 3. 透過有線網路週邊直接控制被包裝的烏龜或周邊裝置
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
    refreshScreens = function(self)
        local existing = {}
        for _, scr in ipairs(self.screens) do
            if scr.key then existing[scr.key] = scr end
        end
        local newScreens = {}
        for _, name in ipairs(peripheral.getNames()) do
            if peripheral.getType(name) == "directgpu" then
                local gpu = peripheral.wrap(name)
                if gpu then
                    local count = 1
                    pcall(function() if gpu.getDisplayCount then count = gpu.getDisplayCount() end end)
                    for dId = 0, count - 1 do
                        local key = name .. "#" .. tostring(dId)
                        local info = {pixelWidth=320, pixelHeight=240}
                        pcall(function() if gpu.getDisplayInfo then info = gpu.getDisplayInfo(dId) or info end end)
                        local prev = existing[key]
                        table.insert(newScreens, {
                            key = key,
                            name = name,
                            gpu = gpu,
                            displayId = dId,
                            screenW = info.pixelWidth or 320,
                            screenH = info.pixelHeight or 240,
                            currentView = prev and prev.currentView or "OVERVIEW",
                            isMenuOpen = prev and prev.isMenuOpen or false,
                            buttons = prev and prev.buttons or {}
                        })
                    end
                end
            end
        end
        self.screens = newScreens
        return #self.screens > 0
    end,
    init = function(self)
        return self:refreshScreens()
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
            local ok, err = pcall(function() self:drawScreen(scr) end)
            if not ok then
                pcall(function() self:refreshScreens() end)
                break
            end
        end
    end,
    drawScreen = function(self, scr)
        local gpu = scr.gpu
        local dispId = scr.displayId
        local ok, dispInfo = pcall(function() return gpu.getDisplayInfo(dispId) end)
        if ok and dispInfo and dispInfo.pixelWidth and dispInfo.pixelHeight and dispInfo.pixelWidth > 0 and dispInfo.pixelHeight > 0 then
            scr.screenW = dispInfo.pixelWidth
            scr.screenH = dispInfo.pixelHeight
        end
        local sw = scr.screenW or 320
        local sh = scr.screenH or 240
        local isFull = (sw >= 750 and sh >= 750) -- 5x5 (820x820) 或以上為完整顯示，3x3/4x4/5x4/4x5 (<=656) 為精簡版

        gpu.clear(dispId, 15, 20, 30)
        scr.buttons = {}

        local function addBtn(x, y, w, h, text, bg, fg, act)
            table.insert(scr.buttons, {x=x, y=y, w=w, h=h, text=text, bg=bg, fg=fg, action=act})
        end

        local headerH = isFull and math.max(26, math.min(36, math.floor(sh * 0.06))) or (sh < 220 and 14 or 20)
        gpu.fillRect(dispId, 0, 0, sw, headerH, 25, 40, 65)

        local menuBtnW = isFull and 54 or (sw < 220 and 18 or 24)
        local menuBtnH = headerH - 4
        local menuBtnX = sw - menuBtnW - 3
        local menuBtnY = 2
        if scr.isMenuOpen then
            addBtn(menuBtnX, menuBtnY, menuBtnW, menuBtnH, "[X]", {190, 45, 45}, {255, 255, 255}, function() scr.isMenuOpen = false end)
        else
            addBtn(menuBtnX, menuBtnY, menuBtnW, menuBtnH, isFull and "[MENU]" or "[=]", {35, 80, 150}, {255, 255, 255}, function() scr.isMenuOpen = true end)
        end

        local titleFontSize = isFull and math.max(10, math.min(14, math.floor(headerH * 0.48))) or (sh < 220 and 8 or 9)
        local viewTitle = scr.isMenuOpen and "SELECT VIEW" or (isFull and (VIEW_TITLES[scr.currentView] or scr.currentView) or scr.currentView)
        gpu.drawText(dispId, viewTitle, 8, math.floor((headerH - titleFontSize) / 2), 240, 245, 255, "Arial", titleFontSize, "bold")

        -- 佈局高度自適應分配器
        local numBtnRows = isFull and 3 or 2
        local btnRowH = isFull and math.max(24, math.min(32, math.floor(sh * 0.08))) or (sh < 220 and 12 or (sh < 350 and 16 or 22))
        local btnSpacing = (sh < 220) and 2 or 3
        local totalBtnH = numBtnRows * btnRowH + (numBtnRows - 1) * btnSpacing
        local statH = isFull and 20 or (sh < 220 and 11 or 15)

        local topMargin = headerH + ((sh < 220) and 2 or 4)
        local statY = sh - totalBtnH - statH - ((sh < 220) and 3 or 5)
        local instY = topMargin
        local instH = statY - instY - ((sh < 220) and 2 or 4)
        local btnAreaY = statY + statH + ((sh < 220) and 2 or 3)

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

            local shortTitle = cardW < 65
            addMenuCard(1, 1, isFull and "1. OVERVIEW (ALL)" or (shortTitle and "1. OVR" or "1. OVERVIEW"), "OVERVIEW", {25, 60, 95})
            addMenuCard(2, 1, isFull and "2. PFD (FLIGHT)" or (shortTitle and "2. PFD" or "2. PFD"), "PFD", {20, 90, 50})
            addMenuCard(1, 2, isFull and "3. ECAM (QUAD ENG)" or (shortTitle and "3. ECAM" or "3. ECAM"), "ECAM", {120, 40, 30})
            addMenuCard(2, 2, isFull and "4. CTRL (CONTROLS)" or (shortTitle and "4. CTRL" or "4. CTRL"), "CTRL", {18, 120, 100})
            addMenuCard(1, 3, isFull and "5. NAV (PRESETS)" or (shortTitle and "5. NAV" or "5. NAV"), "NAV", {35, 115, 165})
            addMenuCard(2, 3, isFull and "6. SYS (DIAGNOSE)" or (shortTitle and "6. SYS" or "6. SYS"), "SYS", {80, 45, 95})

        elseif scr.currentView == "OVERVIEW" then
            local currAlt = FlightCore.altiSensor and FlightCore.altiSensor.getHeight() or 0
            local currVspeed = FlightCore.altiSensor and FlightCore.altiSensor.getVerticalSpeed() or 0
            local currPitch, currRoll = FlightCore.getGimbalData()

            if isFull then
                -- 5x5 或以上：超大高解析度完整儀表 (Full Avionics Dashboard)
                local leftW = math.max(140, math.floor(sw * 0.38))
                local pfdX = 6
                local ecamX = pfdX + leftW + 6
                local ecamW = sw - ecamX - 6

                -- 左側卡片: 主飛行儀表 (PFD)
                gpu.fillRect(dispId, pfdX, instY, leftW, instH, 20, 26, 38)
                gpu.drawText(dispId, "PRIMARY FLIGHT", pfdX + 6, instY + 4, 170, 210, 230, "Arial", 11, "bold")

                local gR = math.max(16, math.min(36, math.floor(instH * 0.26), math.floor(leftW * 0.24)))
                local gCX = pfdX + math.floor(leftW * 0.25)
                local gCY = instY + math.floor(instH * 0.54)
                gpu.drawCircle(dispId, gCX, gCY, gR, 80, 170, 240, true)
                local altStr = string.format("%.0f", currAlt)
                local altSz = (gR >= 22) and 13 or 10
                gpu.drawText(dispId, altStr, gCX - math.floor(#altStr * 4), gCY - 6, 255, 255, 255, "Arial", altSz, "bold")

                local textX = pfdX + math.floor(leftW * 0.52)
                local rowSpacing = math.max(12, math.floor((instH - 24) / 4))
                gpu.drawText(dispId, string.format("TGT: %.0fm", FlightCore.state.targetAlt), textX, instY + 14, 80, 230, 255, "Arial", 10, "bold")
                gpu.drawText(dispId, string.format("V.S: %+.2f", currVspeed), textX, instY + 14 + rowSpacing, 255, 205, 75, "Arial", 10, "bold")
                local mCol = (FlightCore.state.mode == "HOLD_ALT") and {80, 255, 120} or {255, 80, 80}
                gpu.drawText(dispId, string.format("MOD: %s", FlightCore.state.mode:sub(1,6)), textX, instY + 14 + rowSpacing * 2, mCol[1], mCol[2], mCol[3], "Arial", 10, "bold")
                gpu.drawText(dispId, string.format("P:%+2.0f R:%+2.0f", currPitch, currRoll), textX, instY + 14 + rowSpacing * 3, 180, 220, 255, "Arial", 9, "plain")

                -- 右側卡片: 2x2 四象限發動機動力監控
                gpu.fillRect(dispId, ecamX, instY, ecamW, instH, 18, 24, 34)

                local subW = math.floor((ecamW - 8) / 2)
                local subH = math.floor((instH - 8) / 2)
                local miniSlots = {
                    {slot="FL", col=1, row=1, name="FRONT-LEFT"},
                    {slot="FR", col=2, row=1, name="FRONT-RIGHT"},
                    {slot="BL", col=1, row=2, name="BACK-LEFT"},
                    {slot="BR", col=2, row=2, name="BACK-RIGHT"}
                }

                for _, ms in ipairs(miniSlots) do
                    local sx = ecamX + 3 + (ms.col - 1) * (subW + 2)
                    local sy = instY + 3 + (ms.row - 1) * (subH + 2)
                    local qH = FlightCore.getQuadHealth(ms.slot)
                    local val = FlightCore.virtualOutputs[ms.slot]
                    local online = qH.online > 0

                    gpu.fillRect(dispId, sx, sy, subW, subH, 24, 32, 46)
                    local slotTitle = (subW < 100) and string.format("[%s]", ms.slot) or string.format("[%s] %d ENG", ms.slot, qH.total)
                    gpu.drawText(dispId, slotTitle, sx + 4, sy + 3, 200, 230, 255, "Arial", 9, "bold")
                    local onCol = online and {80, 255, 120} or {255, 80, 80}
                    local onText = (subW < 80) and string.format("%d/%d", qH.online, qH.total) or string.format("%d/%d ON", qH.online, qH.total)
                    gpu.drawText(dispId, onText, sx + subW - math.floor(#onText * 6) - 4, sy + 3, onCol[1], onCol[2], onCol[3], "Arial", 9, "bold")

                    -- 動態油門條
                    local barY = sy + math.floor(subH * 0.38)
                    local barW = subW - 8
                    local barH = math.max(6, math.min(14, math.floor(subH * 0.20)))
                    gpu.fillRect(dispId, sx + 4, barY, barW, barH, 10, 18, 28)
                    local fillW = math.floor(barW * math.min(1.0, math.max(0.0, val / 15.0)))
                    local barCol = (val > 12) and {255, 69, 0} or ((val > 8) and {255, 215, 0} or {50, 205, 50})
                    if fillW > 0 then gpu.fillRect(dispId, sx + 4, barY, fillW, barH, barCol[1], barCol[2], barCol[3]) end

                    -- 數值指示
                    local thrText = (subW < 90) and string.format("%.1f", val) or string.format("THR: %.1f", val)
                    gpu.drawText(dispId, thrText, sx + 4, sy + subH - 11, 70, 255, 120, "Arial", 9, "bold")
                    local pwmText = string.format("PWM:%d", FlightCore.engineOutputs[ms.slot] or 0)
                    gpu.drawText(dispId, pwmText, sx + subW - math.floor(#pwmText * 6) - 4, sy + subH - 11, 150, 190, 225, "Arial", 8, "plain")
                end

                -- 中段狀態列
                gpu.fillRect(dispId, pfdX, statY, sw - 12, statH, 30, 36, 50)
                gpu.drawText(dispId, string.format("STATUS: %s", FlightCore.state.statusMsg:sub(1,45)), pfdX + 8, statY + 4, 80, 230, 255, "Arial", 10, "bold")

                -- 底部 3 排控制按鈕群
                local bY1 = btnAreaY
                local bW1 = math.floor((sw - 12 - 20) / 6)
                addBtn(pfdX, bY1, bW1, btnRowH, "+50m", {20, 90, 50}, {255, 255, 255}, function() FlightCore.adjustTargetAlt(50) end)
                addBtn(pfdX + (bW1 + 4), bY1, bW1, btnRowH, "+10m", {27, 94, 32}, {255, 255, 255}, function() FlightCore.adjustTargetAlt(10) end)
                addBtn(pfdX + (bW1 + 4)*2, bY1, bW1, btnRowH, "+1m", {46, 125, 50}, {255, 255, 255}, function() FlightCore.adjustTargetAlt(1) end)
                addBtn(pfdX + (bW1 + 4)*3, bY1, bW1, btnRowH, "-1m", {230, 81, 0}, {255, 255, 255}, function() FlightCore.adjustTargetAlt(-1) end)
                addBtn(pfdX + (bW1 + 4)*4, bY1, bW1, btnRowH, "-10m", {198, 40, 40}, {255, 255, 255}, function() FlightCore.adjustTargetAlt(-10) end)
                addBtn(pfdX + (bW1 + 4)*5, bY1, bW1, btnRowH, "LOCK", {0, 131, 143}, {255, 255, 255}, function() FlightCore.lockCurrentAlt() end)

                local bY2 = bY1 + btnRowH + btnSpacing
                local bW2 = math.floor((sw - 12 - 12) / 4)
                addBtn(pfdX, bY2, bW2, btnRowH, "BASE +1", {0, 105, 92}, {255, 255, 255}, function() FlightCore.adjustBaseThrottle(1.0) end)
                addBtn(pfdX + bW2 + 4, bY2, bW2, btnRowH, "BASE -1", {55, 71, 79}, {255, 255, 255}, function() FlightCore.adjustBaseThrottle(-1.0) end)
                addBtn(pfdX + (bW2 + 4)*2, bY2, bW2, btnRowH, "RE-SCAN", {21, 101, 192}, {255, 255, 255}, function() FlightCore.scanQuadTurtles(); FlightCore.state.statusMsg="Re-scanned!" end)
                addBtn(pfdX + (bW2 + 4)*3, bY2, bW2, btnRowH, "CALIB", {106, 27, 154}, {255, 255, 255}, function() FlightCore.startCalibration() end)

                local bY3 = bY2 + btnRowH + btnSpacing
                local bW3 = math.floor((sw - 12 - 4) / 2)
                local holdBg = (FlightCore.state.mode == "HOLD_ALT") and {46, 125, 50} or {27, 94, 32}
                addBtn(pfdX, bY3, bW3, btnRowH, "[ HOLD ALTITUDE ]", holdBg, {255, 255, 255}, function() FlightCore.holdAltitude() end)
                local stopBg = (FlightCore.state.mode == "IDLE") and {198, 40, 40} or {183, 28, 28}
                addBtn(pfdX + bW3 + 4, bY3, bW3, btnRowH, "[ STOP / IDLE ]", stopBg, {255, 255, 255}, function() FlightCore.stopEngines() end)
            else
                -- 3x3 ~ 4x5 精簡模式 (<5x5 Compact Mode, 適配 384x384 到 512x640 及小螢幕)
                local colW = math.floor((sw - 12) / 2)
                local pfdX = 4
                local ecamX = pfdX + colW + 4

                if colW >= 85 then
                    -- 寬度充足精簡版 (4x4, 5x4): 圓形高度錶 + 2x2 四象限進度條
                    gpu.fillRect(dispId, pfdX, instY, colW, instH, 20, 26, 38)
                    gpu.drawText(dispId, "ALT/ATT", pfdX + 4, instY + 3, 170, 210, 230, "Arial", 9, "bold")

                    local gR = math.max(8, math.min(22, math.floor(colW * 0.18), math.floor(instH * 0.22)))
                    local gCX = pfdX + math.max(gR + 4, math.floor(colW * 0.24))
                    local gCY = instY + math.floor(instH * 0.54)
                    gpu.drawCircle(dispId, gCX, gCY, gR, 80, 170, 240, true)
                    local altStr = string.format("%.0f", currAlt)
                    gpu.drawText(dispId, altStr, gCX - math.floor(#altStr * 4), gCY - 4, 255, 255, 255, "Arial", 9, "bold")

                    local textX = gCX + gR + 6
                    local rowSpacing = math.max(8, math.min(18, math.floor((instH - 18) / 4)))
                    gpu.drawText(dispId, string.format("T:%.0f", FlightCore.state.targetAlt), textX, instY + 10, 80, 230, 255, "Arial", 9, "bold")
                    gpu.drawText(dispId, string.format("V:%+.1f", currVspeed), textX, instY + 10 + rowSpacing, 255, 205, 75, "Arial", 9, "bold")
                    local mCol = (FlightCore.state.mode == "HOLD_ALT") and {80, 255, 120} or {255, 80, 80}
                    gpu.drawText(dispId, string.format("M:%s", FlightCore.state.mode:sub(1,4)), textX, instY + 10 + rowSpacing * 2, mCol[1], mCol[2], mCol[3], "Arial", 9, "bold")
                    gpu.drawText(dispId, string.format("P:%+.0f", currPitch), textX, instY + 10 + rowSpacing * 3, 180, 220, 255, "Arial", 8, "plain")

                    -- 右側: 2x2 Mini Quad Engines
                    gpu.fillRect(dispId, ecamX, instY, colW, instH, 18, 24, 34)
                    local subW = math.floor((colW - 6) / 2)
                    local subH = math.floor((instH - 6) / 2)
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
                        local online = qH.online > 0

                        gpu.fillRect(dispId, sx, sy, subW, subH, 24, 32, 46)
                        gpu.drawText(dispId, string.format("[%s] %dE", ms.slot, qH.total), sx + 2, sy + 2, 200, 230, 255, "Arial", 8, "bold")

                        -- 動態微型進度條 (帶清晰邊框)
                        local barY = sy + math.floor(subH * 0.40)
                        local barW = math.max(4, subW - 6)
                        local barH = math.max(4, math.min(10, math.floor(subH * 0.18)))
                        gpu.fillRect(dispId, sx + 3, barY, barW, barH, 10, 16, 24)
                        local fillW = math.floor(barW * math.min(1.0, math.max(0.0, val / 15.0)))
                        local barCol = (val > 12) and {255, 69, 0} or ((val > 8) and {255, 215, 0} or {50, 205, 50})
                        if fillW > 0 then gpu.fillRect(dispId, sx + 3, barY, fillW, barH, barCol[1], barCol[2], barCol[3]) end

                        -- 數值指示
                        local btmLineY = sy + subH - 9
                        gpu.drawText(dispId, string.format("%4.1f", val), sx + 2, btmLineY, 70, 255, 120, "Arial", 8, "plain")
                        local pwmStr = string.format("P:%d", FlightCore.engineOutputs[ms.slot] or 0)
                        gpu.drawText(dispId, pwmStr, sx + subW - math.floor(#pwmStr * 5) - 2, btmLineY, 150, 190, 225, "Arial", 8, "plain")
                    end
                else
                    -- 極致窄螢幕 (3x3 螢幕, colW < 85px): 乾淨直向數據流，零碰撞
                    -- 左側: PFD 飛行姿態垂直數據堆疊
                    gpu.fillRect(dispId, pfdX, instY, colW, instH, 20, 26, 38)
                    gpu.drawText(dispId, "ALT/ATT", pfdX + 3, instY + 2, 170, 210, 230, "Arial", 8, "bold")

                    local rowSp = math.max(8, math.min(12, math.floor((instH - 14) / 4)))
                    local yBase = instY + 13
                    gpu.drawText(dispId, string.format("ALT:%4.0f", currAlt), pfdX + 3, yBase, 255, 255, 255, "Arial", 8, "bold")
                    gpu.drawText(dispId, string.format("TGT:%4.0f", FlightCore.state.targetAlt), pfdX + 3, yBase + rowSp, 80, 230, 255, "Arial", 8, "bold")
                    gpu.drawText(dispId, string.format("V.S:%+4.1f", currVspeed), pfdX + 3, yBase + rowSp * 2, 255, 205, 75, "Arial", 8, "bold")
                    local mCol = (FlightCore.state.mode == "HOLD_ALT") and {80, 255, 120} or {255, 80, 80}
                    gpu.drawText(dispId, string.format("MOD:%s", FlightCore.state.mode:sub(1, 4)), pfdX + 3, yBase + rowSp * 3, mCol[1], mCol[2], mCol[3], "Arial", 8, "bold")

                    -- 右側: ECAM 四象限發動機直向數據堆疊 (FL, FR, BL, BR 逐行排布)
                    gpu.fillRect(dispId, ecamX, instY, colW, instH, 18, 24, 34)
                    gpu.drawText(dispId, "ECAM 4Q", ecamX + 3, instY + 2, 170, 210, 230, "Arial", 8, "bold")

                    local qSlots = {"FL", "FR", "BL", "BR"}
                    for idx, slot in ipairs(qSlots) do
                        local qH = FlightCore.getQuadHealth(slot)
                        local val = FlightCore.virtualOutputs[slot]
                        local sig = FlightCore.engineOutputs[slot] or 0
                        local yPos = yBase + (idx - 1) * rowSp
                        local colVal = (val > 12) and {255, 69, 0} or ((val > 8) and {255, 215, 0} or {80, 255, 120})
                        if qH.online == 0 then colVal = {255, 80, 80} end
                        gpu.drawText(dispId, string.format("%s%4.1f P%d", slot, val, sig), ecamX + 3, yPos, colVal[1], colVal[2], colVal[3], "Arial", 8, "plain")
                    end
                end

                -- 底部狀態
                gpu.fillRect(dispId, pfdX, statY, sw - 8, statH, 30, 36, 50)
                local maxStatChars = math.max(6, math.floor((sw - 20) / 6))
                gpu.drawText(dispId, string.format("STATUS: %s", FlightCore.state.statusMsg:sub(1, maxStatChars)), pfdX + 4, statY + 3, 80, 230, 255, "Arial", 8, "bold")

                -- 底部按鈕 (2 排按鈕，嚴格鎖定高度防重疊)
                local bW4 = math.floor((sw - 8 - 9) / 4)
                addBtn(pfdX, btnAreaY, bW4, btnRowH, "+10", {27, 94, 32}, {255, 255, 255}, function() FlightCore.adjustTargetAlt(10) end)
                addBtn(pfdX + bW4 + 3, btnAreaY, bW4, btnRowH, "-10", {198, 40, 40}, {255, 255, 255}, function() FlightCore.adjustTargetAlt(-10) end)
                addBtn(pfdX + (bW4 + 3)*2, btnAreaY, bW4, btnRowH, "B+", {0, 105, 92}, {255, 255, 255}, function() FlightCore.adjustBaseThrottle(1.0) end)
                addBtn(pfdX + (bW4 + 3)*3, btnAreaY, bW4, btnRowH, "B-", {55, 71, 79}, {255, 255, 255}, function() FlightCore.adjustBaseThrottle(-1.0) end)

                local bY2 = btnAreaY + btnRowH + btnSpacing
                local bW2 = math.floor((sw - 8 - 3) / 2)
                local holdBg = (FlightCore.state.mode == "HOLD_ALT") and {46, 125, 50} or {27, 94, 32}
                addBtn(pfdX, bY2, bW2, btnRowH, "HOLD", holdBg, {255, 255, 255}, function() FlightCore.holdAltitude() end)
                local stopBg = (FlightCore.state.mode == "IDLE") and {198, 40, 40} or {183, 28, 28}
                addBtn(pfdX + bW2 + 3, bY2, bW2, btnRowH, "STOP", stopBg, {255, 255, 255}, function() FlightCore.stopEngines() end)
            end

        elseif scr.currentView == "PFD" then
            local currAlt = FlightCore.altiSensor and FlightCore.altiSensor.getHeight() or 0
            local currVspeed = FlightCore.altiSensor and FlightCore.altiSensor.getVerticalSpeed() or 0
            local currPitch, currRoll = FlightCore.getGimbalData()

            local mainH = sh - headerH - btnRowH - 12
            local mainY = headerH + 4

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
            if leftW >= 80 then
                gpu.drawText(dispId, string.format("P:%+3.0f* R:%+3.0f*", currPitch, currRoll), 10, mainY + mainH - 12, 200, 230, 255, "Arial", 9, "bold")
            end

            local rightX = 6 + leftW + 5
            local rightW = sw - rightX - 6
            gpu.fillRect(dispId, rightX, mainY, rightW, mainH, 24, 32, 46)

            gpu.drawText(dispId, rightW < 55 and "ALT" or "CURRENT ALT", rightX + 4, mainY + 4, 140, 160, 180, "Arial", 8, "plain")
            local altStr = string.format("%.0fm", currAlt)
            local altSz = (isFull and rightW >= 60) and 14 or 10
            gpu.drawText(dispId, altStr, rightX + 4, mainY + 16, 255, 255, 255, "Arial", altSz, "bold")

            local midY = mainY + (isFull and 36 or 28)
            gpu.drawText(dispId, rightW < 55 and string.format("T:%.0f", FlightCore.state.targetAlt) or string.format("TGT: %.0fm", FlightCore.state.targetAlt), rightX + 4, midY, 80, 230, 255, "Arial", 8, "bold")
            gpu.drawText(dispId, rightW < 55 and string.format("V:%+.1f", currVspeed) or string.format("V.S: %+.2f", currVspeed), rightX + 4, midY + 11, 255, 205, 75, "Arial", 8, "bold")

            local bY = mainY + mainH + 4
            local bW = math.floor((sw - 12 - 12) / 4)
            addBtn(6, bY, bW, btnRowH, "+10m", {30, 100, 45}, {255, 255, 255}, function() FlightCore.adjustTargetAlt(10) end)
            addBtn(6 + (bW+4), bY, bW, btnRowH, "-10m", {190, 40, 40}, {255, 255, 255}, function() FlightCore.adjustTargetAlt(-10) end)
            local holdBg = (FlightCore.state.mode == "HOLD_ALT") and {45, 140, 60} or {30, 90, 40}
            addBtn(6 + (bW+4)*2, bY, bW, btnRowH, "HOLD", holdBg, {255, 255, 255}, function() FlightCore.holdAltitude() end)
            local stopBg = (FlightCore.state.mode == "IDLE") and {200, 45, 45} or {160, 30, 30}
            addBtn(6 + (bW+4)*3, bY, bW, btnRowH, "STOP", stopBg, {255, 255, 255}, function() FlightCore.stopEngines() end)

        elseif scr.currentView == "ECAM" then
            local topY = headerH + 3
            local btmH = isFull and 22 or 14
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

                if isFull and (quadW >= 110) then
                    local titleCol = online and {180, 230, 255} or {255, 120, 120}
                    local titleStr = (quadW < 160) and string.format("[%s] QUAD", q.slot) or q.title
                    gpu.drawText(dispId, titleStr, qx + 6, qy + 4, titleCol[1], titleCol[2], titleCol[3], "Arial", 10, "bold")

                    local engCountStr = (quadW < 130) and string.format("%dE", qH.total) or string.format("ENG:%d", qH.total)
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

                    local barY = qy + math.floor(quadH * 0.40)
                    local barW = math.max(4, quadW - 6)
                    local barH = math.max(4, math.min(10, math.floor(quadH * 0.18)))
                    gpu.fillRect(dispId, qx + 3, barY, barW, barH, 10, 16, 24)
                    local fillW = math.floor(barW * math.min(1.0, math.max(0.0, FlightCore.virtualOutputs[q.slot] / 15.0)))
                    local barCol = (FlightCore.virtualOutputs[q.slot] > 12) and {255, 69, 0} or ((FlightCore.virtualOutputs[q.slot] > 8) and {255, 215, 0} or {50, 205, 50})
                    if fillW > 0 then gpu.fillRect(dispId, qx + 3, barY, fillW, barH, barCol[1], barCol[2], barCol[3]) end

                    local btmLineY = qy + quadH - 9
                    gpu.drawText(dispId, string.format("%.1f", FlightCore.virtualOutputs[q.slot]), qx + 3, btmLineY, 70, 255, 120, "Arial", 8, "bold")
                    local actStr = (quadW < 65) and string.format("P:%d", FlightCore.engineOutputs[q.slot] or 0) or string.format("P:%d %d/%d", FlightCore.engineOutputs[q.slot] or 0, qH.online, qH.total)
                    local actCol = (qH.online > 0) and {80, 255, 120} or {255, 80, 80}
                    gpu.drawText(dispId, actStr, qx + quadW - math.floor(#actStr * 5) - 3, btmLineY, actCol[1], actCol[2], actCol[3], "Arial", 8, "plain")
                end
            end

            local statY = topY + areaH + 2
            gpu.fillRect(dispId, 4, statY, sw - 8, btmH, 25, 32, 45)
            if isFull then
                gpu.drawText(dispId, string.format("BASE: %4.2f/15 | BALANCED | STATUS: %s", FlightCore.state.baseThrottle, FlightCore.state.statusMsg:sub(1, 26)), 8, statY + 4, 80, 230, 255, "Arial", 9, "bold")
            else
                local statChars = math.max(6, math.floor((sw - 60) / 6))
                gpu.drawText(dispId, string.format("B:%4.2f | %s", FlightCore.state.baseThrottle, FlightCore.state.statusMsg:sub(1, statChars)), 6, statY + 3, 80, 230, 255, "Arial", 8, "bold")
            end

        elseif scr.currentView == "CTRL" then
            local currAlt = FlightCore.altiSensor and FlightCore.altiSensor.getHeight() or 0
            local currVspeed = FlightCore.altiSensor and FlightCore.altiSensor.getVerticalSpeed() or 0
            local currPitch, currRoll = FlightCore.getGimbalData()

            if isFull then
                -- 大型螢幕 >=5x5 專業飛控管理中心 (Full Avionics Flight Management System)
                local cardW = math.floor((sw - 16) / 2)
                local c1X = 6
                local c2X = c1X + cardW + 4

                -- 卡片 1: 飛行高度與升降剖面
                gpu.fillRect(dispId, c1X, instY, cardW, instH, 20, 30, 43)
                gpu.drawText(dispId, "ALTITUDE PROFILE", c1X + 6, instY + 4, 140, 160, 180, "Arial", 9, "bold")

                local altStr = string.format("%.1fm", currAlt)
                gpu.drawText(dispId, altStr, c1X + 8, instY + 18, 255, 255, 255, "Arial", 14, "bold")

                local textOffY = instY + 36
                local rowSp = math.max(12, math.floor((instH - 42) / 3))
                local tgtStr = (cardW < 170) and string.format("TGT:%.0fm (DIFF:%+.0fm)", FlightCore.state.targetAlt, FlightCore.state.targetAlt - currAlt) or string.format("TARGET: %5.1fm (DIFF:%+5.1fm)", FlightCore.state.targetAlt, FlightCore.state.targetAlt - currAlt)
                gpu.drawText(dispId, tgtStr, c1X + 8, textOffY, 80, 230, 255, "Arial", 9, "bold")
                local vsCol = (math.abs(currVspeed) < 0.2) and {80, 255, 120} or {255, 205, 75}
                gpu.drawText(dispId, string.format("V.SPEED: %+5.2f m/s", currVspeed), c1X + 8, textOffY + rowSp, vsCol[1], vsCol[2], vsCol[3], "Arial", 9, "bold")
                local mCol = (FlightCore.state.mode == "HOLD_ALT") and {80, 255, 120} or ((FlightCore.state.mode == "CALIBRATING") and {255, 205, 75} or {255, 80, 80})
                gpu.drawText(dispId, string.format("MODE: [%s]", FlightCore.state.mode), c1X + 8, textOffY + rowSp * 2, mCol[1], mCol[2], mCol[3], "Arial", 9, "bold")

                -- 卡片 2: 姿態平衡與推力總覽
                gpu.fillRect(dispId, c2X, instY, cardW, instH, 20, 30, 43)
                gpu.drawText(dispId, "ATTITUDE & PROPULSION", c2X + 6, instY + 4, 140, 160, 180, "Arial", 9, "bold")

                local baseStr = (cardW < 170) and string.format("BASE: %4.2f", FlightCore.state.baseThrottle) or string.format("BASE: %4.2f / 15.0", FlightCore.state.baseThrottle)
                gpu.drawText(dispId, baseStr, c2X + 8, instY + 18, 80, 255, 120, "Arial", 14, "bold")
                gpu.drawText(dispId, string.format("GYRO: P:%+4.1f*  R:%+4.1f*", currPitch, currRoll), c2X + 8, textOffY, 180, 220, 255, "Arial", 9, "bold")

                local quadSummary = (cardW < 170) and string.format("F:%.1f,%.1f B:%.1f,%.1f", FlightCore.virtualOutputs.FL, FlightCore.virtualOutputs.FR, FlightCore.virtualOutputs.BL, FlightCore.virtualOutputs.BR) or string.format("FL:%.1f FR:%.1f BL:%.1f BR:%.1f", FlightCore.virtualOutputs.FL, FlightCore.virtualOutputs.FR, FlightCore.virtualOutputs.BL, FlightCore.virtualOutputs.BR)
                gpu.drawText(dispId, quadSummary, c2X + 8, textOffY + rowSp, 255, 205, 75, "Arial", 9, "bold")
                local sysSummary = (cardW < 170) and "SYS: BALANCED & SYNCED" or "SYSTEM: BALANCED & SYNCED"
                gpu.drawText(dispId, sysSummary, c2X + 8, textOffY + rowSp * 2, 80, 230, 255, "Arial", 9, "bold")

                -- 中段狀態列
                gpu.fillRect(dispId, 6, statY, sw - 12, statH, 30, 36, 50)
                gpu.drawText(dispId, string.format("STATUS: %s", FlightCore.state.statusMsg:sub(1, 45)), 10, statY + 4, 80, 230, 255, "Arial", 9, "bold")

                -- 下半部控制按鈕群 (3 組分類)
                local bW1 = math.floor((sw - 12 - 20) / 6)
                addBtn(6, btnAreaY, bW1, btnRowH, "+50m", {20, 90, 50}, {255, 255, 255}, function() FlightCore.adjustTargetAlt(50) end)
                addBtn(6 + (bW1+4), btnAreaY, bW1, btnRowH, "+10m", {27, 94, 32}, {255, 255, 255}, function() FlightCore.adjustTargetAlt(10) end)
                addBtn(6 + (bW1+4)*2, btnAreaY, bW1, btnRowH, "+1m", {46, 125, 50}, {255, 255, 255}, function() FlightCore.adjustTargetAlt(1) end)
                addBtn(6 + (bW1+4)*3, btnAreaY, bW1, btnRowH, "-1m", {230, 81, 0}, {255, 255, 255}, function() FlightCore.adjustTargetAlt(-1) end)
                addBtn(6 + (bW1+4)*4, btnAreaY, bW1, btnRowH, "-10m", {198, 40, 40}, {255, 255, 255}, function() FlightCore.adjustTargetAlt(-10) end)
                addBtn(6 + (bW1+4)*5, btnAreaY, bW1, btnRowH, "LOCK", {0, 131, 143}, {255, 255, 255}, function() FlightCore.lockCurrentAlt() end)

                local r2Y = btnAreaY + btnRowH + btnSpacing
                local bW2 = math.floor((sw - 12 - 12) / 4)
                addBtn(6, r2Y, bW2, btnRowH, "BASE +1.0", {0, 105, 92}, {255, 255, 255}, function() FlightCore.adjustBaseThrottle(1.0) end)
                addBtn(6 + (bW2+4), r2Y, bW2, btnRowH, "BASE -1.0", {55, 71, 79}, {255, 255, 255}, function() FlightCore.adjustBaseThrottle(-1.0) end)
                addBtn(6 + (bW2+4)*2, r2Y, bW2, btnRowH, "BASE +0.1", {0, 137, 123}, {255, 255, 255}, function() FlightCore.adjustBaseThrottle(0.1) end)
                addBtn(6 + (bW2+4)*3, r2Y, bW2, btnRowH, "BASE -0.1", {69, 90, 100}, {255, 255, 255}, function() FlightCore.adjustBaseThrottle(-0.1) end)

                local r3Y = r2Y + btnRowH + btnSpacing
                local bW3 = math.floor((sw - 12 - 12) / 4)
                local holdBg = (FlightCore.state.mode == "HOLD_ALT") and {46, 125, 50} or {27, 94, 32}
                addBtn(6, r3Y, bW3, btnRowH, "[ HOLD ALT ]", holdBg, {255, 255, 255}, function() FlightCore.holdAltitude() end)
                addBtn(6 + (bW3+4), r3Y, bW3, btnRowH, "[ CALIB 200m ]", {106, 27, 154}, {255, 255, 255}, function() FlightCore.startCalibration() end)
                addBtn(6 + (bW3+4)*2, r3Y, bW3, btnRowH, "[ RE-SCAN HW ]", {21, 101, 192}, {255, 255, 255}, function() FlightCore.scanQuadTurtles(); FlightCore.state.statusMsg="Re-scanned!" end)
                local stopBg = (FlightCore.state.mode == "IDLE") and {198, 40, 40} or {183, 28, 28}
                addBtn(6 + (bW3+4)*3, r3Y, bW3, btnRowH, "[ STOP IDLE ]", stopBg, {255, 255, 255}, function() FlightCore.stopEngines() end)
            else
                -- 緊湊模式 (<5x5 Compact Mode)
                local topH = math.max(16, math.min(22, math.floor(instH * 0.28)))
                gpu.fillRect(dispId, 6, instY, sw - 12, topH, 20, 28, 42)
                local mCol = (FlightCore.state.mode == "HOLD_ALT") and {80, 255, 120} or {255, 80, 80}
                if sw < 160 then
                    gpu.drawText(dispId, string.format("[%s] A:%.0f->%.0f", FlightCore.state.mode:sub(1,4), currAlt, FlightCore.state.targetAlt), 8, instY + 3, 200, 230, 255, "Arial", 8, "plain")
                else
                    gpu.drawText(dispId, string.format("[%s]", FlightCore.state.mode:sub(1,6)), 10, instY + 4, mCol[1], mCol[2], mCol[3], "Arial", 9, "bold")
                    gpu.drawText(dispId, string.format("ALT:%.0f->%.0f | B:%.2f", currAlt, FlightCore.state.targetAlt, FlightCore.state.baseThrottle), 65, instY + 4, 200, 230, 255, "Arial", 9, "plain")
                end

                local ctrlRowH = math.max(12, math.min(24, math.floor((sh - instY - topH - 12) / 3)))
                local gridY = instY + topH + 4

                -- Row 1: Target Altitude
                local bW1 = math.floor((sw - 12 - 12) / 4)
                addBtn(6, gridY, bW1, ctrlRowH, "+10m", {30, 100, 45}, {240, 255, 240}, function() FlightCore.adjustTargetAlt(10) end)
                addBtn(6 + (bW1+4), gridY, bW1, ctrlRowH, "+1m", {45, 130, 55}, {240, 255, 240}, function() FlightCore.adjustTargetAlt(1) end)
                addBtn(6 + (bW1+4)*2, gridY, bW1, ctrlRowH, "-1m", {210, 95, 20}, {255, 245, 230}, function() FlightCore.adjustTargetAlt(-1) end)
                addBtn(6 + (bW1+4)*3, gridY, bW1, ctrlRowH, "-10m", {190, 40, 40}, {255, 235, 235}, function() FlightCore.adjustTargetAlt(-10) end)

                -- Row 2: Base Throttle
                local r2Y = gridY + ctrlRowH + 3
                local bW2 = math.floor((sw - 12 - 12) / 4)
                addBtn(6, r2Y, bW2, ctrlRowH, "B +1", {0, 110, 95}, {230, 250, 245}, function() FlightCore.adjustBaseThrottle(1.0) end)
                addBtn(6 + (bW2+4), r2Y, bW2, ctrlRowH, "B -1", {55, 70, 80}, {235, 240, 245}, function() FlightCore.adjustBaseThrottle(-1.0) end)
                addBtn(6 + (bW2+4)*2, r2Y, bW2, ctrlRowH, "B +.1", {20, 120, 110}, {230, 255, 250}, function() FlightCore.adjustBaseThrottle(0.1) end)
                addBtn(6 + (bW2+4)*3, r2Y, bW2, ctrlRowH, "B -.1", {70, 80, 90}, {235, 240, 245}, function() FlightCore.adjustBaseThrottle(-0.1) end)

                -- Row 3: Flight Ops
                local r3Y = r2Y + ctrlRowH + 3
                local bW3 = math.floor((sw - 12 - 12) / 4)
                local holdBg = (FlightCore.state.mode == "HOLD_ALT") and {45, 140, 60} or {30, 90, 40}
                addBtn(6, r3Y, bW3, ctrlRowH, "HOLD", holdBg, {255, 255, 255}, function() FlightCore.holdAltitude() end)
                addBtn(6 + (bW3+4), r3Y, bW3, ctrlRowH, "CALIB", {110, 30, 155}, {250, 230, 255}, function() FlightCore.startCalibration() end)
                addBtn(6 + (bW3+4)*2, r3Y, bW3, ctrlRowH, "RE-SCAN", {25, 100, 190}, {230, 245, 255}, function() FlightCore.scanQuadTurtles(); FlightCore.state.statusMsg="Re-scanned!" end)
                local stopBg = (FlightCore.state.mode == "IDLE") and {200, 45, 45} or {160, 30, 30}
                addBtn(6 + (bW3+4)*3, r3Y, bW3, ctrlRowH, "STOP", stopBg, {255, 255, 255}, function() FlightCore.stopEngines() end)
            end

        elseif scr.currentView == "NAV" then
            local currAlt = FlightCore.altiSensor and FlightCore.altiSensor.getHeight() or 0
            local currVspeed = FlightCore.altiSensor and FlightCore.altiSensor.getVerticalSpeed() or 0

            local statNavH = isFull and 26 or 16
            gpu.fillRect(dispId, 6, headerH + 4, sw - 12, statNavH, 20, 28, 40)
            if sw < 160 then
                gpu.drawText(dispId, string.format("ALT:%.0f TGT:%.0f V:%+.1f", currAlt, FlightCore.state.targetAlt, currVspeed), 8, headerH + 6, 80, 230, 255, "Arial", 8, "bold")
            else
                gpu.drawText(dispId, string.format("ALT: %.0fm -> TGT: %.0fm (V.S: %+.1f)", currAlt, FlightCore.state.targetAlt, currVspeed), 10, headerH + 6, 80, 230, 255, "Arial", 9, "bold")
            end

            local gridY = headerH + 4 + statNavH + 4
            local btnAreaH = sh - gridY - 4
            local rowH = math.floor((btnAreaH - 8) / 3)
            local colW = math.floor((sw - 12 - 6) / 2)

            addBtn(6, gridY, colW, rowH, isFull and "[ 0m LANDING ]" or "0m LAND", {160, 50, 50}, {255, 255, 255}, function() FlightCore.setTargetAlt(0) end)
            addBtn(6 + colW + 6, gridY, colW, rowH, isFull and "[ 80m TREETOP ]" or "80m TREE", {35, 110, 60}, {255, 255, 255}, function() FlightCore.setTargetAlt(80) end)

            addBtn(6, gridY + rowH + 4, colW, rowH, isFull and "[ 150m CRUISE ]" or "150m CRZ", {25, 90, 140}, {255, 255, 255}, function() FlightCore.setTargetAlt(150) end)
            addBtn(6 + colW + 6, gridY + rowH + 4, colW, rowH, isFull and "[ 200m CALIBRATE ]" or "200m CAL", {100, 40, 140}, {255, 255, 255}, function() FlightCore.setTargetAlt(200) end)

            addBtn(6, gridY + (rowH + 4)*2, colW, rowH, isFull and "[ 300m HIGH-ALT ]" or "300m HIGH", {20, 110, 150}, {255, 255, 255}, function() FlightCore.setTargetAlt(300) end)
            addBtn(6 + colW + 6, gridY + (rowH + 4)*2, colW, rowH, isFull and "[ LOCK CURRENT ]" or "LOCK CURR", {130, 90, 20}, {255, 255, 255}, function() FlightCore.lockCurrentAlt() end)

        elseif scr.currentView == "SYS" then
            local cardW = math.floor((sw - 18) / 2)
            local btmSysH = btnRowH
            local btmY = sh - btmSysH - 4
            local cardH = math.floor((btmY - topMargin - 6) / 2)
            local startY = topMargin

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
                local actStr = (cardW < 75) and string.format("ON:%d P:%d", qH.online, FlightCore.engineOutputs[s.slot] or 0) or string.format("ACT:%d/%d | P:%d", qH.online, qH.total, FlightCore.engineOutputs[s.slot] or 0)
                gpu.drawText(dispId, actStr, cx + 4, cy + math.max(10, math.floor(cardH * 0.45)), tagCol[1], tagCol[2], tagCol[3], "Arial", 8, "bold")
            end

            local btmW = math.floor((sw - 16) / 2)
            addBtn(6, btmY, btmW, btmSysH, isFull and "[ RE-SCAN HARDWARE ]" or "RE-SCAN HW", {25, 100, 190}, {255, 255, 255}, function() FlightCore.scanQuadTurtles(); FlightCore.state.statusMsg="Re-scanned!" end)
            addBtn(6 + btmW + 4, btmY, btmW, btmSysH, isFull and "[ STOP ALL ENGINES ]" or "STOP ALL", {190, 40, 40}, {255, 255, 255}, function() FlightCore.stopEngines() end)
        end

        for _, btn in ipairs(scr.buttons) do
            gpu.fillRect(dispId, btn.x, btn.y, btn.w, btn.h, btn.bg[1], btn.bg[2], btn.bg[3])
            local btnFontSize = math.max(8, math.min(13, math.floor(btn.h * 0.46)))
            local tx = btn.x + math.max(1, math.floor((btn.w - #btn.text * (btnFontSize * 0.58)) / 2))
            local ty = btn.y + math.floor((btn.h - btnFontSize) / 2)
            gpu.drawText(dispId, btn.text, tx, ty, btn.fg[1], btn.fg[2], btn.fg[3], "Arial", btnFontSize, "bold")
        end

        gpu.updateDisplay(dispId)
    end,
    handleEvent = function(self, event, p1, p2, p3, p4)
        if event == "peripheral" or event == "peripheral_detach" or event == "monitor_resize" or event == "directgpu_resize" then
            self:refreshScreens()
            return
        end

        local targetScreen = nil
        local clickX, clickY = nil, nil

        if event == "directgpu_touch" then
            if type(p1) == "string" and type(p2) == "number" and type(p3) == "number" and type(p4) == "number" then
                for _, scr in ipairs(self.screens) do
                    if scr.name == p1 and scr.displayId == p2 then targetScreen = scr; break end
                end
                clickX, clickY = p3, p4
            elseif type(p1) == "number" and type(p2) == "number" and type(p3) == "number" then
                for _, scr in ipairs(self.screens) do
                    if scr.displayId == p1 then targetScreen = scr; break end
                end
                clickX, clickY = p2, p3
            elseif type(p1) == "number" and type(p2) == "number" then
                targetScreen = self.screens[1]
                clickX, clickY = p1, p2
            end

        elseif event == "monitor_touch" then
            for _, scr in ipairs(self.screens) do
                if scr.name == p1 then targetScreen = scr; break end
            end
            if not targetScreen and #self.screens > 0 then targetScreen = self.screens[1] end
            if targetScreen and type(p2) == "number" and type(p3) == "number" then
                local mw, mh = 50, 19
                local mon = peripheral.wrap(p1)
                if mon and mon.getSize then
                    local ok, w, h = pcall(function() return mon.getSize() end)
                    if ok and w and h and w > 0 and h > 0 then mw, mh = w, h end
                end
                clickX = math.floor(((p2 - 0.5) / mw) * (targetScreen.screenW or 320))
                clickY = math.floor(((p3 - 0.5) / mh) * (targetScreen.screenH or 240))
            end

        elseif event == "mouse_click" then
            targetScreen = self.screens[1]
            if targetScreen and type(p2) == "number" and type(p3) == "number" then
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
    ['&'] = {0x36, 0x49, 0x55, 0x22, 0x50},
    [' '] = {0x00, 0x00, 0x00, 0x00, 0x00}
}

Drivers.tom = {
    screens = {},
    refreshScreens = function(self)
        local existing = {}
        for _, scr in ipairs(self.screens) do
            if scr.id then existing[scr.id] = scr end
        end
        local newScreens = {}
        for _, name in ipairs(peripheral.getNames()) do
            local pType = peripheral.getType(name)
            if pType == "tm_gpu" or pType == "toms_gpu" or pType == "gpu" or pType == "tm_monitor" then
                local gpu = peripheral.wrap(name)
                if gpu and gpu.fill and gpu.filledRectangle then
                    -- 必須先 setSize + refreshSize 才能 getSize 得到正確像素解析度
                    pcall(function() if gpu.setSize then gpu.setSize(64) end end)
                    pcall(function() if gpu.refreshSize then gpu.refreshSize() end end)
                    local w, h = 192, 192
                    local ok, gw, gh = pcall(function() return gpu.getSize() end)
                    if ok and gw and gh and gw > 0 and gh > 0 then
                        if gw < 32 then gw = gw * 64 end
                        if gh < 32 then gh = gh * 64 end
                        w, h = gw, gh
                    end
                    local prev = existing[name]
                    table.insert(newScreens, {
                        id = name,
                        gpu = gpu,
                        screenW = w,
                        screenH = h,
                        currentView = prev and prev.currentView or "OVERVIEW",
                        isMenuOpen = prev and prev.isMenuOpen or false,
                        buttons = prev and prev.buttons or {}
                    })
                end
            end
        end
        self.screens = newScreens
        return #self.screens > 0
    end,
    init = function(self)
        -- 先做一次掃描讓 GPU setSize，等待硬體初始化完成後再重新讀取真實尺寸
        self:refreshScreens()
        sleep(0.1)
        return self:refreshScreens()
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
            local ok, err = pcall(function() self:drawScreen(scr) end)
            if not ok then
                pcall(function() self:refreshScreens() end)
                break
            end
        end
    end,
    drawScreen = function(self, scr)
        local ok, w, h = pcall(function() return scr.gpu.getSize() end)
        -- Tom's GPU getSize() 返回像素大小；若回傳值為圖塊單位（< 32）則以 64px (setSize 64) 換算
        if ok and w and h and w > 0 and h > 0 then
            if w < 32 then w = w * 64 end
            if h < 32 then h = h * 64 end
            scr.screenW, scr.screenH = w, h
        end
        local sw, sh = scr.screenW or 192, scr.screenH or 192
        local isFull = (sw >= 300 and sh >= 300) -- 5x5 (320x320) 或以上為完整顯示，3x3/4x4/5x4/4x5 (<=256) 為精簡版

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
        local headerH = isFull and math.max(26, math.min(36, math.floor(sh * 0.06))) or (sh < 220 and 14 or 20)
        sFR(1, 1, sw, headerH, 0x192841)

        local menuBtnW = isFull and 54 or (sw < 220 and 18 or 24)
        local menuBtnH = headerH - 4
        local menuBtnX = sw - menuBtnW - 3
        local menuBtnY = 2
        if scr.isMenuOpen then
            addBtn(menuBtnX, menuBtnY, menuBtnW, menuBtnH, "[X]", 0x992222, 0xFFFFFF, function() scr.isMenuOpen = false end)
        else
            addBtn(menuBtnX, menuBtnY, menuBtnW, menuBtnH, isFull and "[MENU]" or "[=]", 0x224488, 0xFFFFFF, function() scr.isMenuOpen = true end)
        end

        local maxTitleW = menuBtnX - 12
        local viewTitle = scr.isMenuOpen and "SELECT VIEW" or (isFull and (VIEW_TITLES[scr.currentView] or scr.currentView) or scr.currentView)
        local titleSize = (isFull and #viewTitle * 12 <= maxTitleW and headerH >= 24) and 2 or 1
        sTxt(6, math.floor((headerH - 7 * titleSize) / 2) + 1, viewTitle, 0xF0F5FF, titleSize)

        -- 佈局高度自適應分配器
        local numBtnRows = isFull and 3 or 2
        local btnRowH = isFull and math.max(24, math.min(32, math.floor(sh * 0.08))) or (sh < 220 and 12 or (sh < 350 and 16 or 22))
        local btnSpacing = (sh < 220) and 2 or 3
        local totalBtnH = numBtnRows * btnRowH + (numBtnRows - 1) * btnSpacing
        local statH = isFull and 20 or (sh < 220 and 11 or 15)

        local topMargin = headerH + ((sh < 220) and 2 or 4)
        local statY = sh - totalBtnH - statH - ((sh < 220) and 3 or 5)
        local instY = topMargin
        local instH = statY - instY - ((sh < 220) and 2 or 4)
        local btnAreaY = statY + statH + ((sh < 220) and 2 or 3)

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

            local shortTitle = cardW < 65
            addMenuCard(1, 1, isFull and "1. OVERVIEW (ALL)" or (shortTitle and "1. OVR" or "1. OVERVIEW"), "OVERVIEW", 0x1B3A60)
            addMenuCard(2, 1, isFull and "2. PFD (FLIGHT)" or (shortTitle and "2. PFD" or "2. PFD"), "PFD", 0x145A32)
            addMenuCard(1, 2, isFull and "3. ECAM (QUAD ENG)" or (shortTitle and "3. ECAM" or "3. ECAM"), "ECAM", 0x78281F)
            addMenuCard(2, 2, isFull and "4. CTRL (CONTROLS)" or (shortTitle and "4. CTRL" or "4. CTRL"), "CTRL", 0x117864)
            addMenuCard(1, 3, isFull and "5. NAV (PRESETS)" or (shortTitle and "5. NAV" or "5. NAV"), "NAV", 0x2471A3)
            addMenuCard(2, 3, isFull and "6. SYS (DIAGNOSE)" or (shortTitle and "6. SYS" or "6. SYS"), "SYS", 0x512E5F)

        elseif scr.currentView == "OVERVIEW" then
            local currAlt = FlightCore.altiSensor and FlightCore.altiSensor.getHeight() or 0
            local currVspeed = FlightCore.altiSensor and FlightCore.altiSensor.getVerticalSpeed() or 0
            local currPitch, currRoll = FlightCore.getGimbalData()

            if isFull then
                -- 5x5 或以上：完整航電儀表板 (Full Avionics Dashboard)
                local leftW = math.max(140, math.floor(sw * 0.38))
                local pfdX = 6
                local ecamX = pfdX + leftW + 6
                local ecamW = sw - ecamX - 6

                -- 左側卡片: 主飛行儀表 (PFD)
                sFR(pfdX, instY, leftW, instH, 0x141A26)
                sR(pfdX, instY, leftW, instH, 0x283850)
                sTxt(pfdX + 6, instY + 4, "PRIMARY FLIGHT", 0xAAD2E6, 1)

                local gR = math.max(16, math.min(36, math.floor(instH * 0.26), math.floor(leftW * 0.24)))
                local gCX = pfdX + math.floor(leftW * 0.25)
                local gCY = instY + math.floor(instH * 0.54)
                for dy = -gR, gR do
                    local dx = math.floor(math.sqrt(math.max(0, gR*gR - dy*dy)) + 0.5)
                    sL(gCX - dx, gCY + dy, gCX + dx, gCY + dy, 0x1E2837)
                end
                sArc(gCX, gCY, gR, 0, 360, 10, 0x50AAF0)
                local altText = string.format("%.0f", currAlt)
                local altSz = (gR >= 22) and 2 or 1
                sTxt(gCX - math.floor(#altText * 6 * altSz / 2), gCY - 3 * altSz - 1, altText, 0xFFFFFF, altSz)

                local textX = pfdX + math.floor(leftW * 0.52)
                local rowSpacing = math.max(12, math.floor((instH - 24) / 4))
                sTxt(textX, instY + 14, string.format("TGT: %.0fm", FlightCore.state.targetAlt), 0x50E6FF, 1)
                sTxt(textX, instY + 14 + rowSpacing, string.format("V.S: %+.2f", currVspeed), 0xFFCD4B, 1)
                local mCol = (FlightCore.state.mode == "HOLD_ALT") and 0x50FF78 or 0xFF5050
                sTxt(textX, instY + 14 + rowSpacing * 2, string.format("MOD: %s", FlightCore.state.mode:sub(1,6)), mCol, 1)
                sTxt(textX, instY + 14 + rowSpacing * 3, string.format("P:%+2.0f R:%+2.0f", currPitch, currRoll), 0xB4DCFF, 1)

                -- 右側卡片: 2x2 四象限發動機動力監控
                sFR(ecamX, instY, ecamW, instH, 0x121822)
                sR(ecamX, instY, ecamW, instH, 0x283850)

                local subW = math.floor((ecamW - 8) / 2)
                local subH = math.floor((instH - 8) / 2)
                local miniSlots = {
                    {slot="FL", col=1, row=1, name="FRONT-LEFT"},
                    {slot="FR", col=2, row=1, name="FRONT-RIGHT"},
                    {slot="BL", col=1, row=2, name="BACK-LEFT"},
                    {slot="BR", col=2, row=2, name="BACK-RIGHT"}
                }
                for _, ms in ipairs(miniSlots) do
                    local sx = ecamX + 3 + (ms.col - 1) * (subW + 2)
                    local sy = instY + 3 + (ms.row - 1) * (subH + 2)
                    local qH = FlightCore.getQuadHealth(ms.slot)
                    local val = FlightCore.virtualOutputs[ms.slot]
                    local online = qH.online > 0

                    sFR(sx, sy, subW, subH, 0x18202E)
                    sR(sx, sy, subW, subH, online and 0x284864 or 0x642828)

                    local slotTitle = (subW < 100) and string.format("[%s]", ms.slot) or string.format("[%s] %d ENG", ms.slot, qH.total)
                    sTxt(sx + 4, sy + 3, slotTitle, 0xC8E6FF, 1)
                    local onCol = online and 0x50FF78 or 0xFF5050
                    local onText = (subW < 80) and string.format("%d/%d", qH.online, qH.total) or string.format("%d/%d ON", qH.online, qH.total)
                    sTxt(sx + subW - #onText * 6 - 4, sy + 3, onText, onCol, 1)

                    -- 動態油門條
                    local barY = sy + math.floor(subH * 0.38)
                    local barW = subW - 8
                    local barH = math.max(6, math.min(14, math.floor(subH * 0.20)))
                    sFR(sx + 4, barY, barW, barH, 0x0C121C)
                    local fillW = math.floor(barW * math.min(1.0, math.max(0.0, val / 15.0)))
                    local barCol = (val > 12) and 0xFF4500 or ((val > 8) and 0xFFD700 or 0x32CD32)
                    if fillW > 0 then sFR(sx + 4, barY, fillW, barH, barCol) end
                    sR(sx + 4, barY, barW, barH, 0x3C4B64)

                    -- 數值指示
                    local thrText = (subW < 90) and string.format("%.1f", val) or string.format("THR: %.1f", val)
                    sTxt(sx + 4, sy + subH - 10, thrText, 0x46FF78, 1)
                    local pwmText = string.format("PWM:%d", FlightCore.engineOutputs[ms.slot] or 0)
                    sTxt(sx + subW - #pwmText * 6 - 4, sy + subH - 10, pwmText, 0x96BEE1, 1)
                end

                -- 中段狀態通報
                sFR(pfdX, statY, sw - 12, statH, 0x1E2432)
                sR(pfdX, statY, sw - 12, statH, 0x3C4B64)
                sTxt(pfdX + 6, statY + 4, string.format("STATUS: %s", FlightCore.state.statusMsg:sub(1, 45)), 0x50E6FF, 1)

                -- 底部 3 排控制按鈕群
                local bY1 = btnAreaY
                local bW1 = math.floor((sw - 12 - 20) / 6)
                addBtn(pfdX, bY1, bW1, btnRowH, "+50m", 0x145A32, 0xFFFFFF, function() FlightCore.adjustTargetAlt(50) end)
                addBtn(pfdX + (bW1+4), bY1, bW1, btnRowH, "+10m", 0x1B5E20, 0xFFFFFF, function() FlightCore.adjustTargetAlt(10) end)
                addBtn(pfdX + (bW1+4)*2, bY1, bW1, btnRowH, "+1m", 0x2E7D32, 0xFFFFFF, function() FlightCore.adjustTargetAlt(1) end)
                addBtn(pfdX + (bW1+4)*3, bY1, bW1, btnRowH, "-1m", 0xE65100, 0xFFFFFF, function() FlightCore.adjustTargetAlt(-1) end)
                addBtn(pfdX + (bW1+4)*4, bY1, bW1, btnRowH, "-10m", 0xC62828, 0xFFFFFF, function() FlightCore.adjustTargetAlt(-10) end)
                addBtn(pfdX + (bW1+4)*5, bY1, bW1, btnRowH, "LOCK", 0x00838F, 0xFFFFFF, function() FlightCore.lockCurrentAlt() end)

                local bY2 = bY1 + btnRowH + btnSpacing
                local bW2 = math.floor((sw - 12 - 12) / 4)
                addBtn(pfdX, bY2, bW2, btnRowH, "BASE +1", 0x00695C, 0xFFFFFF, function() FlightCore.adjustBaseThrottle(1.0) end)
                addBtn(pfdX + bW2 + 4, bY2, bW2, btnRowH, "BASE -1", 0x37474F, 0xFFFFFF, function() FlightCore.adjustBaseThrottle(-1.0) end)
                addBtn(pfdX + (bW2 + 4)*2, bY2, bW2, btnRowH, "RE-SCAN", 0x1565C0, 0xFFFFFF, function() FlightCore.scanQuadTurtles(); FlightCore.state.statusMsg="Re-scanned!" end)
                addBtn(pfdX + (bW2 + 4)*3, bY2, bW2, btnRowH, "CALIB", 0x6A1B9A, 0xFFFFFF, function() FlightCore.startCalibration() end)

                local bY3 = bY2 + btnRowH + btnSpacing
                local bW3 = math.floor((sw - 12 - 4) / 2)
                local holdBg = (FlightCore.state.mode == "HOLD_ALT") and 0x2E7D32 or 0x1B5E20
                addBtn(pfdX, bY3, bW3, btnRowH, "[ HOLD ALTITUDE ]", holdBg, 0xFFFFFF, function() FlightCore.holdAltitude() end)
                local stopBg = (FlightCore.state.mode == "IDLE") and 0xC62828 or 0xB71C1C
                addBtn(pfdX + bW3 + 4, bY3, bW3, btnRowH, "[ STOP / IDLE ]", stopBg, 0xFFFFFF, function() FlightCore.stopEngines() end)
            else
                -- 3x3 ~ 4x5 精簡模式 (<5x5 Compact Mode, 適配 384x384 到 512x640 及小螢幕)
                local colW = math.floor((sw - 12) / 2)
                local pfdX = 4
                local ecamX = pfdX + colW + 4

                if colW >= 85 then
                    -- 寬度充足精簡版 (4x4, 5x4): 圓形高度錶 + 2x2 四象限進度條
                    sFR(pfdX, instY, colW, instH, 0x141A26)
                    sR(pfdX, instY, colW, instH, 0x283850)
                    sTxt(pfdX + 4, instY + 3, "ALT/ATT", 0xAAD2E6, 1)

                    local gR = math.max(8, math.min(22, math.floor(colW * 0.18), math.floor(instH * 0.22)))
                    local gCX = pfdX + math.max(gR + 4, math.floor(colW * 0.24))
                    local gCY = instY + math.floor(instH * 0.54)
                    for dy = -gR, gR do
                        local dx = math.floor(math.sqrt(math.max(0, gR*gR - dy*dy)) + 0.5)
                        sL(gCX - dx, gCY + dy, gCX + dx, gCY + dy, 0x1E2837)
                    end
                    sArc(gCX, gCY, gR, 0, 360, 15, 0x50AAF0)
                    local altText = string.format("%.0f", currAlt)
                    sTxt(gCX - math.floor(#altText * 3), gCY - 3, altText, 0xFFFFFF, 1)

                    local textX = gCX + gR + 6
                    local rowSpacing = math.max(8, math.min(18, math.floor((instH - 18) / 4)))
                    sTxt(textX, instY + 10, string.format("T:%.0f", FlightCore.state.targetAlt), 0x50E6FF, 1)
                    sTxt(textX, instY + 10 + rowSpacing, string.format("V:%+.1f", currVspeed), 0xFFCD4B, 1)
                    local mCol = (FlightCore.state.mode == "HOLD_ALT") and 0x50FF78 or 0xFF5050
                    sTxt(textX, instY + 10 + rowSpacing * 2, string.format("M:%s", FlightCore.state.mode:sub(1,4)), mCol, 1)
                    sTxt(textX, instY + 10 + rowSpacing * 3, string.format("P:%+.0f", currPitch), 0xB4DCFF, 1)

                    -- 右側: 2x2 Mini Quad Engines (FL/FR/BL/BR)
                    sFR(ecamX, instY, colW, instH, 0x121822)
                    sR(ecamX, instY, colW, instH, 0x283850)

                    local subW = math.floor((colW - 6) / 2)
                    local subH = math.floor((instH - 6) / 2)
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
                        local online = qH.online > 0

                        sFR(sx, sy, subW, subH, 0x18202E)
                        sR(sx, sy, subW, subH, online and 0x284864 or 0x642828)
                        sTxt(sx + 2, sy + 2, string.format("[%s] %dE", ms.slot, qH.total), 0xC8E6FF, 1)

                        -- 動態微型進度條 (帶底框和邊框)
                        local barY = sy + math.floor(subH * 0.40)
                        local barW = math.max(4, subW - 6)
                        local barH = math.max(4, math.min(10, math.floor(subH * 0.18)))
                        sFR(sx + 3, barY, barW, barH, 0x0A1018)
                        local fillW = math.floor(barW * math.min(1.0, math.max(0.0, val / 15.0)))
                        local barCol = (val > 12) and 0xFF4500 or ((val > 8) and 0xFFD700 or 0x32CD32)
                        if fillW > 0 then sFR(sx + 3, barY, fillW, barH, barCol) end
                        sR(sx + 3, barY, barW, barH, 0x384860)

                        -- 數值指示
                        local btmLineY = sy + subH - 9
                        sTxt(sx + 2, btmLineY, string.format("%4.1f", val), 0x46FF78, 1)
                        local pwmStr = string.format("P:%d", FlightCore.engineOutputs[ms.slot] or 0)
                        sTxt(sx + subW - #pwmStr * 6 - 2, btmLineY, pwmStr, 0x96BEE1, 1)
                    end
                else
                    -- 極致窄螢幕 (3x3 螢幕, colW < 85px): 乾淨直向數據流，零碰撞
                    -- 左側: PFD 飛行姿態垂直數據堆疊
                    sFR(pfdX, instY, colW, instH, 0x141A26)
                    sR(pfdX, instY, colW, instH, 0x283850)
                    sTxt(pfdX + 3, instY + 2, "ALT/ATT", 0xAAD2E6, 1)

                    local rowSp = math.max(8, math.min(12, math.floor((instH - 14) / 4)))
                    local yBase = instY + 13
                    sTxt(pfdX + 3, yBase, string.format("ALT:%4.0f", currAlt), 0xFFFFFF, 1)
                    sTxt(pfdX + 3, yBase + rowSp, string.format("TGT:%4.0f", FlightCore.state.targetAlt), 0x50E6FF, 1)
                    sTxt(pfdX + 3, yBase + rowSp * 2, string.format("V.S:%+4.1f", currVspeed), 0xFFCD4B, 1)
                    local mCol = (FlightCore.state.mode == "HOLD_ALT") and 0x50FF78 or 0xFF5050
                    sTxt(pfdX + 3, yBase + rowSp * 3, string.format("MOD:%s", FlightCore.state.mode:sub(1, 4)), mCol, 1)

                    -- 右側: ECAM 四象限發動機直向數據堆疊 (FL, FR, BL, BR 逐行排布)
                    sFR(ecamX, instY, colW, instH, 0x121822)
                    sR(ecamX, instY, colW, instH, 0x283850)
                    sTxt(ecamX + 3, instY + 2, "ECAM 4Q", 0xAAD2E6, 1)

                    local qSlots = {"FL", "FR", "BL", "BR"}
                    for idx, slot in ipairs(qSlots) do
                        local qH = FlightCore.getQuadHealth(slot)
                        local val = FlightCore.virtualOutputs[slot]
                        local sig = FlightCore.engineOutputs[slot] or 0
                        local yPos = yBase + (idx - 1) * rowSp
                        local colVal = (val > 12) and 0xFF4500 or ((val > 8) and 0xFFD700 or 0x50FF78)
                        if qH.online == 0 then colVal = 0xFF5050 end
                        sTxt(ecamX + 3, yPos, string.format("%s%4.1f P%d", slot, val, sig), colVal, 1)
                    end
                end

                -- 底部狀態列
                sFR(pfdX, statY, sw - 8, statH, 0x1E2432)
                sR(pfdX, statY, sw - 8, statH, 0x3C4B64)
                local maxStatChars = math.max(6, math.floor((sw - 20) / 6))
                sTxt(pfdX + 4, statY + 3, string.format("STATUS: %s", FlightCore.state.statusMsg:sub(1, maxStatChars)), 0x50E6FF, 1)

                -- 底部 2 排按鈕 (高度充足防重疊)
                local bW4 = math.floor((sw - 8 - 9) / 4)
                addBtn(pfdX, btnAreaY, bW4, btnRowH, "+10", 0x1B5E20, 0xFFFFFF, function() FlightCore.adjustTargetAlt(10) end)
                addBtn(pfdX + bW4 + 3, btnAreaY, bW4, btnRowH, "-10", 0xC62828, 0xFFFFFF, function() FlightCore.adjustTargetAlt(-10) end)
                addBtn(pfdX + (bW4 + 3)*2, btnAreaY, bW4, btnRowH, "B+", 0x00695C, 0xFFFFFF, function() FlightCore.adjustBaseThrottle(1.0) end)
                addBtn(pfdX + (bW4 + 3)*3, btnAreaY, bW4, btnRowH, "B-", 0x37474F, 0xFFFFFF, function() FlightCore.adjustBaseThrottle(-1.0) end)

                local bY2 = btnAreaY + btnRowH + btnSpacing
                local bW2 = math.floor((sw - 8 - 3) / 2)
                local holdBg = (FlightCore.state.mode == "HOLD_ALT") and 0x2E7D32 or 0x1B5E20
                addBtn(pfdX, bY2, bW2, btnRowH, "HOLD", holdBg, 0xFFFFFF, function() FlightCore.holdAltitude() end)
                local stopBg = (FlightCore.state.mode == "IDLE") and 0xC62828 or 0xB71C1C
                addBtn(pfdX + bW2 + 3, bY2, bW2, btnRowH, "STOP", stopBg, 0xFFFFFF, function() FlightCore.stopEngines() end)
            end

        elseif scr.currentView == "PFD" then
            local currAlt = FlightCore.altiSensor and FlightCore.altiSensor.getHeight() or 0
            local currVspeed = FlightCore.altiSensor and FlightCore.altiSensor.getVerticalSpeed() or 0
            local currPitch, currRoll = FlightCore.getGimbalData()

            local mainH = sh - headerH - btnRowH - 12
            local mainY = headerH + 4

            local leftW = math.floor(sw * 0.52)
            sFR(6, mainY, leftW, mainH, 0x141A26)
            sR(6, mainY, leftW, mainH, 0x283850)

            local hCX = 6 + math.floor(leftW / 2)
            local hCY = mainY + math.floor(mainH / 2)
            local hR = math.min(math.floor(leftW * 0.38), math.floor(mainH * 0.40))
            sArc(hCX, hCY, hR, 0, 360, 10, 0x50AAF0)

            local rollRad = math.rad(-currRoll)
            local lx1 = math.floor(hCX - hR * 0.85 * math.cos(rollRad) + 0.5)
            local ly1 = math.floor(hCY - hR * 0.85 * math.sin(rollRad) + 0.5)
            local lx2 = math.floor(hCX + hR * 0.85 * math.cos(rollRad) + 0.5)
            local ly2 = math.floor(hCY + hR * 0.85 * math.sin(rollRad) + 0.5)
            sLS(lx1, ly1, lx2, ly2, 0xFFD700)
            if leftW >= 80 then
                sTxt(10, mainY + mainH - 12, string.format("P:%+3.0f* R:%+3.0f*", currPitch, currRoll), 0xC8E6FF, 1)
            end

            local rightX = 6 + leftW + 5
            local rightW = sw - rightX - 6
            sFR(rightX, mainY, rightW, mainH, 0x18202E)
            sR(rightX, mainY, rightW, mainH, 0x283850)

            sTxt(rightX + 4, mainY + 4, rightW < 55 and "ALT" or "CURRENT ALT", 0x8CA0B4, 1)
            local altStr = string.format("%.0fm", currAlt)
            local altSz = (isFull and rightW >= 60) and 2 or 1
            sTxt(rightX + 4, mainY + 16, altStr, 0xFFFFFF, altSz)

            local midY = mainY + (isFull and 36 or 28)
            sTxt(rightX + 4, midY, rightW < 55 and string.format("T:%.0f", FlightCore.state.targetAlt) or string.format("TGT: %.0fm", FlightCore.state.targetAlt), 0x50E6FF, 1)
            sTxt(rightX + 4, midY + 11, rightW < 55 and string.format("V:%+.1f", currVspeed) or string.format("V.S: %+.2f", currVspeed), 0xFFCD4B, 1)

            local bY = mainY + mainH + 4
            local bW = math.floor((sw - 12 - 12) / 4)
            addBtn(6, bY, bW, btnRowH, isFull and "+10m" or "+10", 0x1E642D, 0xFFFFFF, function() FlightCore.adjustTargetAlt(10) end)
            addBtn(6 + (bW+4), bY, bW, btnRowH, isFull and "-10m" or "-10", 0xBE2828, 0xFFFFFF, function() FlightCore.adjustTargetAlt(-10) end)
            local holdBg = (FlightCore.state.mode == "HOLD_ALT") and 0x2D8C3C or 0x1E5A28
            addBtn(6 + (bW+4)*2, bY, bW, btnRowH, "HOLD", holdBg, 0xFFFFFF, function() FlightCore.holdAltitude() end)
            local stopBg = (FlightCore.state.mode == "IDLE") and 0xC82D2D or 0xA01E1E
            addBtn(6 + (bW+4)*3, bY, bW, btnRowH, "STOP", stopBg, 0xFFFFFF, function() FlightCore.stopEngines() end)

        elseif scr.currentView == "ECAM" then
            local topY = headerH + 3
            local btmH = isFull and 22 or 14
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
                sFR(qx, qy, quadW, quadH, 0x121822)
                sR(qx, qy, quadW, quadH, 0x283850)

                local qH = FlightCore.getQuadHealth(q.slot)
                local online = qH.online > 0
                local val = FlightCore.virtualOutputs[q.slot]

                if isFull and (quadW >= 110) then
                    local titleCol = online and 0xB4E6FF or 0xFF7878
                    local titleStr = (quadW < 160) and string.format("[%s] QUAD", q.slot) or q.title
                    sTxt(qx + 6, qy + 4, titleStr, titleCol, 1)

                    local engCountStr = (quadW < 130) and string.format("%dE", qH.total) or string.format("ENG:%d", qH.total)
                    sTxt(qx + quadW - #engCountStr * 6 - 6, qy + 4, engCountStr, 0x96BEE6, 1)

                    -- 圓形刻度盤 (Tom's GPU - Airbus A350/A320 ECAM Style)
                    local dialR = math.min(math.floor(quadW * 0.28), math.floor(quadH * 0.28))
                    local engCenterX = qx + math.floor(quadW / 2)
                    local engCenterY = qy + math.floor(quadH * 0.46)

                    -- 1. 主圓弧軌跡 (210° 底左 -> 90° 頂部 -> -30° 底右，頂部拱起，底部開放)
                    sArc(engCenterX, engCenterY, dialR, 210, -30, 8, 0x647890)
                    -- 紅色超限警告區間 (10° -> -30°)
                    sArc(engCenterX, engCenterY, dialR, 10, -30, 5, 0xFF3737)

                    -- 2. 刻度標記線 (210°=0, 90°=7.5, -30°=15)
                    for _, deg in ipairs({210, 90, -30}) do
                        local rad = math.rad(deg)
                        local tx1 = engCenterX + math.floor((dialR - 3) * math.cos(rad) + 0.5)
                        local ty1 = engCenterY - math.floor((dialR - 3) * math.sin(rad) + 0.5)
                        local tx2 = engCenterX + math.floor((dialR + 3) * math.cos(rad) + 0.5)
                        local ty2 = engCenterY - math.floor((dialR + 3) * math.sin(rad) + 0.5)
                        sLS(tx1, ty1, tx2, ty2, 0x96B4DC)
                    end

                    -- 3. 指針 (0 時指向 210° 底左，隨推力順時針旋轉至 -30° 底右)
                    local ratio = math.min(1.0, math.max(0.0, val / 15.0))
                    local needleDeg = 210 - ratio * 240
                    local nRad = math.rad(needleDeg)
                    local px = engCenterX + math.floor((dialR - 2) * math.cos(nRad) + 0.5)
                    local py = engCenterY - math.floor((dialR - 2) * math.sin(nRad) + 0.5)
                    sLS(engCenterX, engCenterY, px, py, 0x50FF78)

                    -- 中心軸心點
                    sFR(engCenterX - 1, engCenterY - 1, 3, 3, 0xC8E6FF)

                    -- 4. 底部數位指示方框 (Airbus ECAM 數位讀數框，位於指針下方不遮擋)
                    local boxW = math.max(28, math.floor(dialR * 1.3))
                    local boxH = 11
                    local boxX = engCenterX - math.floor(boxW / 2)
                    local boxY = engCenterY + math.floor(dialR * 0.35)
                    sFR(boxX, boxY, boxW, boxH, 0x0C121C)
                    sR(boxX, boxY, boxW, boxH, 0x384860)

                    local thrValStr = string.format("%4.1f", val)
                    sTxt(boxX + math.floor((boxW - #thrValStr * 6) / 2) + 1, boxY + 2, thrValStr, 0x50FF78, 1)

                    local statText = string.format("ACT:%d/%d", qH.online, qH.total)
                    if qH.total == 0 then statText = "NO ENG" end
                    local statCol = (qH.online > 0) and 0x50FF78 or 0xFF5050
                    sTxt(qx + 6, qy + quadH - 12, statText, statCol, 1)

                    local pwmText = string.format("PWM:%d", FlightCore.engineOutputs[q.slot] or 0)
                    sTxt(qx + quadW - #pwmText * 6 - 6, qy + quadH - 12, pwmText, 0x96BEE1, 1)
                else
                    -- 緊湊/直立螢幕版
                    local qTitle = string.format("[%s] %dE", q.slot, qH.total)
                    local titleCol = online and 0xB4E6FF or 0xFF7878
                    sTxt(qx + 3, qy + 3, qTitle, titleCol, 1)

                    local barY = qy + math.floor(quadH * 0.40)
                    local barW = math.max(4, quadW - 6)
                    local barH = math.max(4, math.min(10, math.floor(quadH * 0.18)))
                    sFR(qx + 3, barY, barW, barH, 0x0A1018)
                    local fillW = math.floor(barW * math.min(1.0, math.max(0.0, val / 15.0)))
                    local barCol = (val > 12) and 0xFF4500 or ((val > 8) and 0xFFD700 or 0x32CD32)
                    if fillW > 0 then sFR(qx + 3, barY, fillW, barH, barCol) end
                    sR(qx + 3, barY, barW, barH, 0x384860)

                    local btmLineY = qy + quadH - 9
                    sTxt(qx + 3, btmLineY, string.format("%.1f", val), 0x46FF78, 1)
                    local actStr = (quadW < 65) and string.format("P:%d", FlightCore.engineOutputs[q.slot] or 0) or string.format("P:%d %d/%d", FlightCore.engineOutputs[q.slot] or 0, qH.online, qH.total)
                    local actCol = (qH.online > 0) and 0x50FF78 or 0xFF5050
                    sTxt(qx + quadW - #actStr * 6 - 3, btmLineY, actStr, actCol, 1)
                end
            end

            local statY = topY + areaH + 2
            sFR(4, statY, sw - 8, btmH, 0x19202D)
            sR(4, statY, sw - 8, btmH, 0x3C4B64)
            if isFull then
                local statChars = math.max(6, math.floor((sw - 160) / 6))
                sTxt(8, statY + 4, string.format("BASE: %4.2f/15 | BALANCED | STATUS: %s", FlightCore.state.baseThrottle, FlightCore.state.statusMsg:sub(1, statChars)), 0x50E6FF, 1)
            else
                local statChars = math.max(6, math.floor((sw - 60) / 6))
                sTxt(6, statY + 3, string.format("B:%4.2f | %s", FlightCore.state.baseThrottle, FlightCore.state.statusMsg:sub(1, statChars)), 0x50E6FF, 1)
            end

        elseif scr.currentView == "CTRL" then
            local currAlt = FlightCore.altiSensor and FlightCore.altiSensor.getHeight() or 0
            local currVspeed = FlightCore.altiSensor and FlightCore.altiSensor.getVerticalSpeed() or 0
            local currPitch, currRoll = FlightCore.getGimbalData()

            if isFull then
                -- 大型螢幕 >=5x5 專業飛控管理中心 (Full Avionics Flight Management System)
                local cardW = math.floor((sw - 16) / 2)
                local c1X = 6
                local c2X = c1X + cardW + 4

                -- 卡片 1: 飛行高度與升降剖面
                sFR(c1X, instY, cardW, instH, 0x141E2B)
                sR(c1X, instY, cardW, instH, 0x284864)
                sTxt(c1X + 6, instY + 4, "ALTITUDE PROFILE", 0x8CA0B4, 1)
                sTxt(c1X + 8, instY + 18, string.format("%.1fm", currAlt), 0xFFFFFF, 2)

                local textOffY = instY + 36
                local rowSp = math.max(12, math.floor((instH - 42) / 3))
                local tgtStr = (cardW < 170) and string.format("TGT:%.0fm (DIFF:%+.0fm)", FlightCore.state.targetAlt, FlightCore.state.targetAlt - currAlt) or string.format("TARGET: %5.1fm (DIFF:%+5.1fm)", FlightCore.state.targetAlt, FlightCore.state.targetAlt - currAlt)
                sTxt(c1X + 8, textOffY, tgtStr, 0x50E6FF, 1)
                local vsCol = (math.abs(currVspeed) < 0.2) and 0x50FF78 or 0xFFCD4B
                sTxt(c1X + 8, textOffY + rowSp, string.format("V.SPEED: %+5.2f m/s", currVspeed), vsCol, 1)
                local mCol = (FlightCore.state.mode == "HOLD_ALT") and 0x50FF78 or ((FlightCore.state.mode == "CALIBRATING") and 0xFFCD4B or 0xFF5050)
                sTxt(c1X + 8, textOffY + rowSp * 2, string.format("MODE: [%s]", FlightCore.state.mode), mCol, 1)

                -- 卡片 2: 姿態平衡與推力總覽
                sFR(c2X, instY, cardW, instH, 0x141E2B)
                sR(c2X, instY, cardW, instH, 0x284864)
                sTxt(c2X + 6, instY + 4, "ATTITUDE & PROPULSION", 0x8CA0B4, 1)
                local baseStr = (cardW < 170) and string.format("BASE: %4.2f", FlightCore.state.baseThrottle) or string.format("BASE: %4.2f / 15.0", FlightCore.state.baseThrottle)
                sTxt(c2X + 8, instY + 18, baseStr, 0x50FF78, 2)
                sTxt(c2X + 8, textOffY, string.format("GYRO: P:%+4.1f*  R:%+4.1f*", currPitch, currRoll), 0xB4DCFF, 1)

                local quadSummary = (cardW < 170) and string.format("F:%.1f,%.1f B:%.1f,%.1f", FlightCore.virtualOutputs.FL, FlightCore.virtualOutputs.FR, FlightCore.virtualOutputs.BL, FlightCore.virtualOutputs.BR) or string.format("FL:%.1f FR:%.1f BL:%.1f BR:%.1f", FlightCore.virtualOutputs.FL, FlightCore.virtualOutputs.FR, FlightCore.virtualOutputs.BL, FlightCore.virtualOutputs.BR)
                sTxt(c2X + 8, textOffY + rowSp, quadSummary, 0xFFCD4B, 1)
                local sysSummary = (cardW < 170) and "SYS: BALANCED & SYNCED" or "SYSTEM: BALANCED & SYNCED"
                sTxt(c2X + 8, textOffY + rowSp * 2, sysSummary, 0x50E6FF, 1)

                -- 中段狀態列
                sFR(6, statY, sw - 12, statH, 0x1E2432)
                sR(6, statY, sw - 12, statH, 0x3C4B64)
                sTxt(10, statY + 4, string.format("STATUS: %s", FlightCore.state.statusMsg:sub(1, 45)), 0x50E6FF, 1)

                -- 下半部控制按鈕群 (3 組分類)
                local bW1 = math.floor((sw - 12 - 20) / 6)
                addBtn(6, btnAreaY, bW1, btnRowH, "+50m", 0x145A32, 0xFFFFFF, function() FlightCore.adjustTargetAlt(50) end)
                addBtn(6 + (bW1+4), btnAreaY, bW1, btnRowH, "+10m", 0x1B5E20, 0xFFFFFF, function() FlightCore.adjustTargetAlt(10) end)
                addBtn(6 + (bW1+4)*2, btnAreaY, bW1, btnRowH, "+1m", 0x2E7D32, 0xFFFFFF, function() FlightCore.adjustTargetAlt(1) end)
                addBtn(6 + (bW1+4)*3, btnAreaY, bW1, btnRowH, "-1m", 0xE65100, 0xFFFFFF, function() FlightCore.adjustTargetAlt(-1) end)
                addBtn(6 + (bW1+4)*4, btnAreaY, bW1, btnRowH, "-10m", 0xC62828, 0xFFFFFF, function() FlightCore.adjustTargetAlt(-10) end)
                addBtn(6 + (bW1+4)*5, btnAreaY, bW1, btnRowH, "LOCK", 0x00838F, 0xFFFFFF, function() FlightCore.lockCurrentAlt() end)

                local r2Y = btnAreaY + btnRowH + btnSpacing
                local bW2 = math.floor((sw - 12 - 12) / 4)
                addBtn(6, r2Y, bW2, btnRowH, "BASE +1.0", 0x00695C, 0xFFFFFF, function() FlightCore.adjustBaseThrottle(1.0) end)
                addBtn(6 + (bW2+4), r2Y, bW2, btnRowH, "BASE -1.0", 0x37474F, 0xFFFFFF, function() FlightCore.adjustBaseThrottle(-1.0) end)
                addBtn(6 + (bW2+4)*2, r2Y, bW2, btnRowH, "BASE +0.1", 0x00897B, 0xFFFFFF, function() FlightCore.adjustBaseThrottle(0.1) end)
                addBtn(6 + (bW2+4)*3, r2Y, bW2, btnRowH, "BASE -0.1", 0x455A64, 0xFFFFFF, function() FlightCore.adjustBaseThrottle(-0.1) end)

                local r3Y = r2Y + btnRowH + btnSpacing
                local bW3 = math.floor((sw - 12 - 12) / 4)
                local holdBg = (FlightCore.state.mode == "HOLD_ALT") and 0x2E7D32 or 0x1B5E20
                addBtn(6, r3Y, bW3, btnRowH, "[ HOLD ALT ]", holdBg, 0xFFFFFF, function() FlightCore.holdAltitude() end)
                addBtn(6 + (bW3+4), r3Y, bW3, btnRowH, "[ CALIB 200m ]", 0x6A1B9A, 0xFFFFFF, function() FlightCore.startCalibration() end)
                addBtn(6 + (bW3+4)*2, r3Y, bW3, btnRowH, "[ RE-SCAN HW ]", 0x1565C0, 0xFFFFFF, function() FlightCore.scanQuadTurtles(); FlightCore.state.statusMsg="Re-scanned!" end)
                local stopBg = (FlightCore.state.mode == "IDLE") and 0xC62828 or 0xB71C1C
                addBtn(6 + (bW3+4)*3, r3Y, bW3, btnRowH, "[ STOP IDLE ]", stopBg, 0xFFFFFF, function() FlightCore.stopEngines() end)
            else
                -- 緊湊模式 (<5x5 Compact Mode)
                local topH = math.max(16, math.min(22, math.floor(instH * 0.28)))
                sFR(6, instY, sw - 12, topH, 0x141C2A)
                sR(6, instY, sw - 12, topH, 0x284864)
                local mCol = (FlightCore.state.mode == "HOLD_ALT") and 0x50FF78 or 0xFF5050
                if sw < 160 then
                    sTxt(8, instY + 3, string.format("[%s] A:%.0f->%.0f", FlightCore.state.mode:sub(1,4), currAlt, FlightCore.state.targetAlt), 0xC8E6FF, 1)
                else
                    sTxt(10, instY + 4, string.format("[%s]", FlightCore.state.mode:sub(1,6)), mCol, 1)
                    sTxt(65, instY + 4, string.format("ALT:%.0f->%.0f | B:%.2f", currAlt, FlightCore.state.targetAlt, FlightCore.state.baseThrottle), 0xC8E6FF, 1)
                end

                local ctrlRowH = math.max(12, math.min(24, math.floor((sh - instY - topH - 12) / 3)))
                local gridY = instY + topH + 4

                -- Row 1: Target Altitude
                local bW1 = math.floor((sw - 12 - 12) / 4)
                addBtn(6, gridY, bW1, ctrlRowH, "+10m", 0x1E642D, 0xFFFFFF, function() FlightCore.adjustTargetAlt(10) end)
                addBtn(6 + (bW1+4), gridY, bW1, ctrlRowH, "+1m", 0x2D8237, 0xFFFFFF, function() FlightCore.adjustTargetAlt(1) end)
                addBtn(6 + (bW1+4)*2, gridY, bW1, ctrlRowH, "-1m", 0xD25F14, 0xFFFFFF, function() FlightCore.adjustTargetAlt(-1) end)
                addBtn(6 + (bW1+4)*3, gridY, bW1, ctrlRowH, "-10m", 0xBE2828, 0xFFFFFF, function() FlightCore.adjustTargetAlt(-10) end)

                -- Row 2: Base Throttle
                local r2Y = gridY + ctrlRowH + 3
                local bW2 = math.floor((sw - 12 - 12) / 4)
                addBtn(6, r2Y, bW2, ctrlRowH, "B +1", 0x006E5F, 0xFFFFFF, function() FlightCore.adjustBaseThrottle(1.0) end)
                addBtn(6 + (bW2+4), r2Y, bW2, ctrlRowH, "B -1", 0x374650, 0xFFFFFF, function() FlightCore.adjustBaseThrottle(-1.0) end)
                addBtn(6 + (bW2+4)*2, r2Y, bW2, ctrlRowH, "B +.1", 0x14786E, 0xFFFFFF, function() FlightCore.adjustBaseThrottle(0.1) end)
                addBtn(6 + (bW2+4)*3, r2Y, bW2, ctrlRowH, "B -.1", 0x46505A, 0xFFFFFF, function() FlightCore.adjustBaseThrottle(-0.1) end)

                -- Row 3: Flight Ops
                local r3Y = r2Y + ctrlRowH + 3
                local bW3 = math.floor((sw - 12 - 12) / 4)
                local holdBg = (FlightCore.state.mode == "HOLD_ALT") and 0x2D8C3C or 0x1E5A28
                addBtn(6, r3Y, bW3, ctrlRowH, "HOLD", holdBg, 0xFFFFFF, function() FlightCore.holdAltitude() end)
                addBtn(6 + (bW3+4), r3Y, bW3, ctrlRowH, "CALIB", 0x6E1E9B, 0xFFFFFF, function() FlightCore.startCalibration() end)
                addBtn(6 + (bW3+4)*2, r3Y, bW3, ctrlRowH, "RE-SCAN", 0x1964BE, 0xFFFFFF, function() FlightCore.scanQuadTurtles(); FlightCore.state.statusMsg="Re-scanned!" end)
                local stopBg = (FlightCore.state.mode == "IDLE") and 0xC82D2D or 0xA01E1E
                addBtn(6 + (bW3+4)*3, r3Y, bW3, ctrlRowH, "STOP", stopBg, 0xFFFFFF, function() FlightCore.stopEngines() end)
            end

        elseif scr.currentView == "NAV" then
            local currAlt = FlightCore.altiSensor and FlightCore.altiSensor.getHeight() or 0
            local currVspeed = FlightCore.altiSensor and FlightCore.altiSensor.getVerticalSpeed() or 0

            local statNavH = isFull and 26 or 16
            sFR(6, headerH + 4, sw - 12, statNavH, 0x141C28)
            sR(6, headerH + 4, sw - 12, statNavH, 0x284864)
            if sw < 160 then
                sTxt(8, headerH + 6, string.format("ALT:%.0f TGT:%.0f V:%+.1f", currAlt, FlightCore.state.targetAlt, currVspeed), 0x50E6FF, 1)
            else
                sTxt(10, headerH + 6, string.format("ALT: %.0fm -> TGT: %.0fm (V.S: %+.1f)", currAlt, FlightCore.state.targetAlt, currVspeed), 0x50E6FF, 1)
            end

            local gridY = headerH + 4 + statNavH + 4
            local btnAreaH = sh - gridY - 4
            local rowH = math.floor((btnAreaH - 8) / 3)
            local colW = math.floor((sw - 12 - 6) / 2)

            addBtn(6, gridY, colW, rowH, isFull and "[ 0m LANDING ]" or "0m LAND", 0xA03232, 0xFFFFFF, function() FlightCore.setTargetAlt(0) end)
            addBtn(6 + colW + 6, gridY, colW, rowH, isFull and "[ 80m TREETOP ]" or "80m TREE", 0x236E3C, 0xFFFFFF, function() FlightCore.setTargetAlt(80) end)

            addBtn(6, gridY + rowH + 4, colW, rowH, isFull and "[ 150m CRUISE ]" or "150m CRZ", 0x195A8C, 0xFFFFFF, function() FlightCore.setTargetAlt(150) end)
            addBtn(6 + colW + 6, gridY + rowH + 4, colW, rowH, isFull and "[ 200m CALIBRATE ]" or "200m CAL", 0x64288C, 0xFFFFFF, function() FlightCore.setTargetAlt(200) end)

            addBtn(6, gridY + (rowH + 4)*2, colW, rowH, isFull and "[ 300m HIGH-ALT ]" or "300m HIGH", 0x146E96, 0xFFFFFF, function() FlightCore.setTargetAlt(300) end)
            addBtn(6 + colW + 6, gridY + (rowH + 4)*2, colW, rowH, isFull and "[ LOCK CURRENT ]" or "LOCK CURR", 0x825A14, 0xFFFFFF, function() FlightCore.lockCurrentAlt() end)

        elseif scr.currentView == "SYS" then
            local cardW = math.floor((sw - 18) / 2)
            local btmSysH = btnRowH
            local btmY = sh - btmSysH - 4
            local cardH = math.floor((btmY - topMargin - 6) / 2)
            local startY = topMargin

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
                sFR(cx, cy, cardW, cardH, online and 0x14231E or 0x281414)
                sR(cx, cy, cardW, cardH, online and 0x28643C or 0x642828)

                local tagCol = online and 0x50FF78 or 0xFF5050
                sTxt(cx + 4, cy + 4, string.format("[%s] %d ENG", s.slot, qH.total), 0xDCEDFF, 1)
                local actStr = (cardW < 75) and string.format("ON:%d P:%d", qH.online, FlightCore.engineOutputs[s.slot] or 0) or string.format("ACT:%d/%d | P:%d", qH.online, qH.total, FlightCore.engineOutputs[s.slot] or 0)
                sTxt(cx + 4, cy + math.max(10, math.floor(cardH * 0.45)), actStr, tagCol, 1)
            end

            local btmW = math.floor((sw - 16) / 2)
            addBtn(6, btmY, btmW, btmSysH, isFull and "[ RE-SCAN HARDWARE ]" or "RE-SCAN HW", 0x1964BE, 0xFFFFFF, function() FlightCore.scanQuadTurtles(); FlightCore.state.statusMsg="Re-scanned!" end)
            addBtn(6 + btmW + 4, btmY, btmW, btmSysH, isFull and "[ STOP ALL ENGINES ]" or "STOP ALL", 0xBE2828, 0xFFFFFF, function() FlightCore.stopEngines() end)
        end

        local btnTextSize = 1
        for _, btn in ipairs(scr.buttons) do
            sFR(btn.x, btn.y, btn.w, btn.h, btn.bg)
            sR(btn.x, btn.y, btn.w, btn.h, 0x8CA0B4)
            local tx = btn.x + math.max(1, math.floor((btn.w - #btn.text * 6 * btnTextSize) / 2))
            local ty = btn.y + math.floor((btn.h - 7 * btnTextSize) / 2)
            sTxt(tx, ty, btn.text, btn.fg, btnTextSize)
        end

        pcall(function() if scr.gpu.sync then scr.gpu.sync() end end)
    end,
    handleEvent = function(self, event, p1, p2, p3, p4)
        if event == "peripheral" or event == "peripheral_detach" or event == "monitor_resize" or event == "tm_monitor_resize" then
            self:refreshScreens()
            return
        end

        local targetScreen = nil
        local clickX, clickY = nil, nil

        if event == "tm_monitor_touch" then
            for _, scr in ipairs(self.screens) do
                if scr.id == p1 then targetScreen = scr; break end
            end
            if not targetScreen and #self.screens > 0 then targetScreen = self.screens[1] end
            if targetScreen and type(p2) == "number" and type(p3) == "number" then
                clickX, clickY = p2, p3
            elseif type(p1) == "number" and type(p2) == "number" then
                targetScreen = self.screens[1]
                clickX, clickY = p1, p2
            end

        elseif event == "monitor_touch" then
            for _, scr in ipairs(self.screens) do
                if scr.id == p1 then targetScreen = scr; break end
            end
            if not targetScreen and #self.screens > 0 then targetScreen = self.screens[1] end
            if targetScreen and type(p2) == "number" and type(p3) == "number" then
                clickX, clickY = p2, p3
            end

        elseif event == "mouse_click" then
            targetScreen = self.screens[1]
            if targetScreen and type(p2) == "number" and type(p3) == "number" then
                local tw, th = term.getSize()
                clickX = math.floor(((p2 - 0.5) / tw) * (targetScreen.screenW or 192))
                clickY = math.floor(((p3 - 0.5) / th) * (targetScreen.screenH or 192))
            end
        end

        if targetScreen and clickX and clickY then
            -- 1. 嘗試直接像素座標匹配
            local hit = false
            for _, btn in ipairs(targetScreen.buttons) do
                if clickX >= btn.x and clickX <= btn.x + btn.w and clickY >= btn.y and clickY <= btn.y + btn.h then
                    pcall(btn.action)
                    self:drawScreen(targetScreen)
                    hit = true
                    break
                end
            end

            -- 2. 若直接像素未命中且座標疑似為字元格（< 60），嘗試字元格縮放換算匹配
            if not hit and clickX < 60 and (targetScreen.screenW or 192) >= 80 then
                local mw, mh = 29, 19
                local mon = peripheral.wrap(p1)
                if mon and mon.getSize then
                    local ok, w, h = pcall(function() return mon.getSize() end)
                    if ok and w and h and w > 0 and h > 0 then
                        if w < 60 then mw, mh = w, h end
                    end
                end
                local mappedX = math.floor(((clickX - 0.5) / mw) * (targetScreen.screenW or 192))
                local mappedY = math.floor(((clickY - 0.5) / mh) * (targetScreen.screenH or 192))
                for _, btn in ipairs(targetScreen.buttons) do
                    if mappedX >= btn.x and mappedX <= btn.x + btn.w and mappedY >= btn.y and mappedY <= btn.y + btn.h then
                        pcall(btn.action)
                        self:drawScreen(targetScreen)
                        break
                    end
                end
            end
        end
    end
}


-- --------------------------------------------------------
-- 2.3 驅動 C: CC 原生進階彩色螢幕 (CC: Tweaked Monitor / Terminal)
-- --------------------------------------------------------
Drivers.normal = {
    screens = {},
    refreshScreens = function(self)
        local existing = {}
        for _, scr in ipairs(self.screens) do
            if scr.id then existing[scr.id] = scr end
        end
        local newScreens = {}
        for _, name in ipairs(peripheral.getNames()) do
            if peripheral.getType(name) == "monitor" then
                local mon = peripheral.wrap(name)
                if mon then
                    pcall(function() mon.setTextScale(0.5) end)
                    local mw, mh = mon.getSize()
                    local prev = existing[name]
                    table.insert(newScreens, {
                        id = name,
                        device = mon,
                        isMonitor = true,
                        screenW = mw,
                        screenH = mh,
                        currentView = prev and prev.currentView or "OVERVIEW",
                        isMenuOpen = prev and prev.isMenuOpen or false,
                        buttons = prev and prev.buttons or {}
                    })
                end
            end
        end

        if #newScreens == 0 then
            local tw, th = term.getSize()
            local prev = existing["terminal"]
            table.insert(newScreens, {
                id = "terminal",
                device = term.current(),
                isMonitor = false,
                screenW = tw,
                screenH = th,
                currentView = prev and prev.currentView or "OVERVIEW",
                isMenuOpen = prev and prev.isMenuOpen or false,
                buttons = prev and prev.buttons or {}
            })
        end
        self.screens = newScreens
        return true
    end,
    init = function(self)
        return self:refreshScreens()
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
            local ok, err = pcall(function() self:drawScreen(scr) end)
            if not ok then
                pcall(function() self:refreshScreens() end)
                break
            end
        end
    end,
    drawScreen = function(self, scr)
        local t = scr.device
        local w, h = t.getSize()
        scr.screenW, scr.screenH = w, h
        local isFull = (w >= 140 and h >= 58) -- 5x5 (approx 145x60 chars at textScale 0.5) 或以上為完整顯示，3x3/4x4/5x4/4x5 為精簡版

        t.setBackgroundColor(colors.black)
        t.clear()
        scr.buttons = {}

        local function addBtn(x, y, bw, bh, text, fg, bg, act)
            table.insert(scr.buttons, {x=x, y=y, w=bw, h=bh, text=text, fg=fg, bg=bg, action=act})
        end

        -- 1. 頂部導航列 (無版本號，僅顯示當前視圖名稱)
        local menuBtnW = isFull and 8 or 5
        local menuBtnX = w - menuBtnW + 1
        self:safeBlit(scr, 1, 1, string.rep(" ", w), "0", "b")

        local viewTitle = scr.isMenuOpen and "SELECT VIEW" or (isFull and (VIEW_TITLES[scr.currentView] or scr.currentView) or scr.currentView)
        self:safeBlit(scr, 2, 1, viewTitle, "0", "b")

        if scr.isMenuOpen then
            addBtn(menuBtnX, 1, menuBtnW, 1, isFull and "[CLOSE]" or "[X]", "0", "e", function() scr.isMenuOpen = false end)
        else
            addBtn(menuBtnX, 1, menuBtnW, 1, isFull and "[ MENU ]" or "[=]", "0", "9", function() scr.isMenuOpen = true end)
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

            addMenuCard(1, 1, isFull and "1. OVERVIEW (ALL)" or "1. OVERVIEW", "OVERVIEW", "0", "9")
            addMenuCard(2, 1, isFull and "2. PFD (FLIGHT)" or "2. PFD", "PFD", "0", "5")
            addMenuCard(1, 2, isFull and "3. ECAM (QUAD ENG)" or "3. ECAM", "ECAM", "0", "e")
            addMenuCard(2, 2, isFull and "4. CTRL (CONTROLS)" or "4. CTRL", "CTRL", "0", "3")
            addMenuCard(1, 3, isFull and "5. NAV (PRESETS)" or "5. NAV", "NAV", "0", "b")
            addMenuCard(2, 3, isFull and "6. SYS (DIAGNOSE)" or "6. SYS", "SYS", "0", "a")

        elseif scr.currentView == "OVERVIEW" then
            local currAlt = FlightCore.altiSensor and FlightCore.altiSensor.getHeight() or 0
            local currVspeed = FlightCore.altiSensor and FlightCore.altiSensor.getVerticalSpeed() or 0
            local currPitch, currRoll = FlightCore.getGimbalData()

            if isFull then
                -- 5x5 或以上：完整大儀表板 (Full Avionics Dashboard)
                self:safeBlit(scr, 2, 3, string.format("ALT: %6.1fm | TARGET: %6.1fm | V.S: %+5.2f m/s | MODE: [%s]", currAlt, FlightCore.state.targetAlt, currVspeed, FlightCore.state.mode), "0", "b")
                self:safeBlit(scr, 2, 4, string.format("ATTITUDE: PITCH %+4.1f* | ROLL %+4.1f* | BASE THROTTLE: %4.2f / 15.0", currPitch, currRoll, FlightCore.state.baseThrottle), "0", "8")

                local gridY = 6
                local quadW = math.floor((w - 3) / 2)
                local quadH = math.max(4, math.floor((h - gridY - 12) / 2))

                local quadDefs = {
                    {slot="FL", col=1, row=1, title="[FL] FRONT-LEFT QUADRANT"},
                    {slot="FR", col=2, row=1, title="[FR] FRONT-RIGHT QUADRANT"},
                    {slot="BL", col=1, row=2, title="[BL] BACK-LEFT QUADRANT"},
                    {slot="BR", col=2, row=2, title="[BR] BACK-RIGHT QUADRANT"}
                }

                for _, q in ipairs(quadDefs) do
                    local qx = 2 + (q.col - 1) * (quadW + 1)
                    local qy = gridY + (q.row - 1) * (quadH + 1)
                    local qH = FlightCore.getQuadHealth(q.slot)
                    local val = FlightCore.virtualOutputs[q.slot]
                    local online = qH.online > 0

                    for r = 0, quadH - 1 do
                        self:safeBlit(scr, qx, qy + r, string.rep(" ", quadW), "0", "7")
                    end
                    self:safeBlit(scr, qx + 1, qy, string.format("%s (ENGINES: %d | ACT: %d)", q.title, qH.total, qH.online), online and "9" or "e", "8")

                    local barLen = math.max(6, quadW - 24)
                    local ratio = math.min(1.0, math.max(0.0, val / 15.0))
                    local filled = math.floor(ratio * barLen + 0.5)
                    local barStr = "[" .. string.rep("=", filled) .. string.rep(" ", barLen - filled) .. "]"
                    self:safeBlit(scr, qx + 1, qy + 1, string.format("THR: %4.1f/15.0 %s", val, barStr), "5", "7")
                    self:safeBlit(scr, qx + 1, qy + 2, string.format("PWM OUTPUT: %2d / 15 | STATUS: %s", FlightCore.engineOutputs[q.slot] or 0, online and "HEALTHY" or "OFFLINE"), online and "3" or "e", "7")
                end

                local statY = gridY + quadH * 2 + 2
                self:safeBlit(scr, 2, statY, string.format("STATUS: %s | BASE: %4.2f/15", FlightCore.state.statusMsg, FlightCore.state.baseThrottle), "0", "b")

                -- 3 排按鈕
                local bY1 = statY + 2
                local bW1 = math.floor((w - 7) / 6)
                addBtn(2, bY1, bW1, 2, "+50m", "0", "5", function() FlightCore.adjustTargetAlt(50) end)
                addBtn(3 + bW1, bY1, bW1, 2, "+10m", "0", "5", function() FlightCore.adjustTargetAlt(10) end)
                addBtn(4 + bW1*2, bY1, bW1, 2, "+1m", "0", "5", function() FlightCore.adjustTargetAlt(1) end)
                addBtn(5 + bW1*3, bY1, bW1, 2, "-1m", "0", "e", function() FlightCore.adjustTargetAlt(-1) end)
                addBtn(6 + bW1*4, bY1, bW1, 2, "-10m", "0", "e", function() FlightCore.adjustTargetAlt(-10) end)
                addBtn(7 + bW1*5, bY1, bW1, 2, "LOCK", "0", "3", function() FlightCore.lockCurrentAlt() end)

                local bY2 = bY1 + 3
                local bW2 = math.floor((w - 5) / 4)
                addBtn(2, bY2, bW2, 2, "BASE +1.0", "3", "0", function() FlightCore.adjustBaseThrottle(1.0) end)
                addBtn(3 + bW2, bY2, bW2, 2, "BASE -1.0", "9", "0", function() FlightCore.adjustBaseThrottle(-1.0) end)
                addBtn(4 + bW2*2, bY2, bW2, 2, "RE-SCAN HW", "b", "0", function() FlightCore.scanQuadTurtles(); FlightCore.state.statusMsg="Re-scanned!" end)
                addBtn(5 + bW2*3, bY2, bW2, 2, "CALIBRATE", "a", "0", function() FlightCore.startCalibration() end)

                local bY3 = bY2 + 3
                local bW3 = math.floor((w - 3) / 2)
                local holdBg = (FlightCore.state.mode == "HOLD_ALT") and "5" or "3"
                addBtn(2, bY3, bW3, 2, "[ HOLD ALTITUDE ]", "0", holdBg, function() FlightCore.holdAltitude() end)
                local stopBg = (FlightCore.state.mode == "IDLE") and "e" or "6"
                addBtn(3 + bW3, bY3, bW3, 2, "[ STOP / IDLE ]", "0", stopBg, function() FlightCore.stopEngines() end)
            else
                -- 3x3 ~ 4x5 精簡版 (Compact Edition)
                self:safeBlit(scr, 2, 2, string.format("ALT: %5.1fm | TGT: %4.0fm | V.S: %+4.1f | [%s]", currAlt, FlightCore.state.targetAlt, currVspeed, FlightCore.state.mode:sub(1,6)), "0", "b")

                local gridY = 4
                local quadW = math.floor((w - 3) / 2)
                local quadH = math.max(2, math.floor((h - gridY - 5) / 2))

                local quadDefs = {
                    {slot="FL", col=1, row=1, title="FL"},
                    {slot="FR", col=2, row=1, title="FR"},
                    {slot="BL", col=1, row=2, title="BL"},
                    {slot="BR", col=2, row=2, title="BR"}
                }

                for _, q in ipairs(quadDefs) do
                    local qx = 2 + (q.col - 1) * (quadW + 1)
                    local qy = gridY + (q.row - 1) * (quadH + 1)
                    local qH = FlightCore.getQuadHealth(q.slot)
                    local val = FlightCore.virtualOutputs[q.slot]
                    local online = qH.online > 0

                    for r = 0, quadH - 1 do
                        self:safeBlit(scr, qx, qy + r, string.rep(" ", quadW), "0", "7")
                    end
                    self:safeBlit(scr, qx + 1, qy, string.format("[%s] %dE (%d ON)", q.slot, qH.total, qH.online), online and "9" or "e", "8")
                    self:safeBlit(scr, qx + 1, qy + 1, string.format("THR: %4.1f P:%d", val, FlightCore.engineOutputs[q.slot] or 0), "5", "7")
                end

                local statY = h - 4
                self:safeBlit(scr, 2, statY, string.format("BASE: %4.2f | %s", FlightCore.state.baseThrottle, FlightCore.state.statusMsg:sub(1, math.max(10, w - 18))), "0", "b")

                local bY1 = h - 3
                local bW4 = math.floor((w - 5) / 4)
                addBtn(2, bY1, bW4, 1, "+10m", "0", "5", function() FlightCore.adjustTargetAlt(10) end)
                addBtn(3 + bW4, bY1, bW4, 1, "-10m", "0", "e", function() FlightCore.adjustTargetAlt(-10) end)
                addBtn(4 + bW4*2, bY1, bW4, 1, "B+1", "3", "0", function() FlightCore.adjustBaseThrottle(1.0) end)
                addBtn(5 + bW4*3, bY1, bW4, 1, "B-1", "9", "0", function() FlightCore.adjustBaseThrottle(-1.0) end)

                local bY2 = h - 1
                local bW2 = math.floor((w - 3) / 2)
                local holdBg = (FlightCore.state.mode == "HOLD_ALT") and "5" or "3"
                addBtn(2, bY2, bW2, 1, "HOLD", "0", holdBg, function() FlightCore.holdAltitude() end)
                local stopBg = (FlightCore.state.mode == "IDLE") and "e" or "6"
                addBtn(3 + bW2, bY2, bW2, 1, "STOP", "0", stopBg, function() FlightCore.stopEngines() end)
            end

        elseif scr.currentView == "PFD" then
            local currAlt = FlightCore.altiSensor and FlightCore.altiSensor.getHeight() or 0
            local currVspeed = FlightCore.altiSensor and FlightCore.altiSensor.getVerticalSpeed() or 0
            local currPitch, currRoll = FlightCore.getGimbalData()

            self:safeBlit(scr, 2, 3, string.format("ALT: %6.1fm | TARGET: %6.1fm", currAlt, FlightCore.state.targetAlt), "0", "b")
            self:safeBlit(scr, 2, 4, string.format("VERTICAL SPEED: %+5.2f m/s", currVspeed), "0", "8")
            self:safeBlit(scr, 2, 5, string.format("PITCH: %+4.1f* | ROLL: %+4.1f*", currPitch, currRoll), "0", "8")
            self:safeBlit(scr, 2, 6, string.format("MODE: [%s]", FlightCore.state.mode), (FlightCore.state.mode == "HOLD_ALT") and "5" or "e", "8")

            local btmY = h - (isFull and 3 or 2)
            local btnW = math.floor((w - 5) / 4)
            local btnH = isFull and 3 or 2
            addBtn(2, btmY, btnW, btnH, "+10m", "0", "5", function() FlightCore.adjustTargetAlt(10) end)
            addBtn(3 + btnW, btmY, btnW, btnH, "-10m", "0", "e", function() FlightCore.adjustTargetAlt(-10) end)
            addBtn(4 + btnW*2, btmY, btnW, btnH, "HOLD", "0", "3", function() FlightCore.holdAltitude() end)
            addBtn(5 + btnW*3, btmY, btnW, btnH, "STOP", "0", "e", function() FlightCore.stopEngines() end)

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
                local online = qH.online > 0

                for r = 0, quadH - 1 do
                    self:safeBlit(scr, qx, qy + r, string.rep(" ", quadW), "0", "7")
                end
                self:safeBlit(scr, qx + 1, qy, string.format("%s (%dE)", q.title, qH.total), online and "9" or "e", "8")
                self:safeBlit(scr, qx + 1, qy + 1, string.format("THR: %4.1f/15.0", val), "5", "7")
                self:safeBlit(scr, qx + 1, qy + 2, string.format("PWM: %2d/15", sig), "3", "7")
                if quadH >= 4 then
                    self:safeBlit(scr, qx + 1, qy + 3, string.format("ACTIVE: %d/%d", qH.online, qH.total), online and "5" or "e", "7")
                end
            end

            self:safeBlit(scr, 2, h, string.format("BASE: %4.2f/15 | %s", FlightCore.state.baseThrottle, FlightCore.state.statusMsg:sub(1, w - 20)), "0", "b")

        elseif scr.currentView == "CTRL" then
            local currAlt = FlightCore.altiSensor and FlightCore.altiSensor.getHeight() or 0
            local currVspeed = FlightCore.altiSensor and FlightCore.altiSensor.getVerticalSpeed() or 0
            local currPitch, currRoll = FlightCore.getGimbalData()

            if isFull then
                -- CC Monitor 大螢幕版 (>= 5x5)
                self:safeBlit(scr, 2, 3, string.format("ALT: %6.1fm | TGT: %4.0fm | V.S: %+5.2f", currAlt, FlightCore.state.targetAlt, currVspeed), "0", "b")
                self:safeBlit(scr, 2, 4, string.format("MODE: [%s] | BASE: %4.2f | GYRO: %+2.0f*, %+2.0f*", FlightCore.state.mode, FlightCore.state.baseThrottle, currPitch, currRoll), "0", "8")

                local btnAreaH = h - 6
                local rowH = math.max(2, math.floor((btnAreaH - 4) / 3))

                -- Row 1: Target Alt
                local bY1 = 6
                local bW1 = math.floor((w - 7) / 6)
                addBtn(2, bY1, bW1, rowH, "+50m", "0", "5", function() FlightCore.adjustTargetAlt(50) end)
                addBtn(3 + bW1, bY1, bW1, rowH, "+10m", "0", "5", function() FlightCore.adjustTargetAlt(10) end)
                addBtn(4 + bW1*2, bY1, bW1, rowH, "+1m", "0", "5", function() FlightCore.adjustTargetAlt(1) end)
                addBtn(5 + bW1*3, bY1, bW1, rowH, "-1m", "0", "e", function() FlightCore.adjustTargetAlt(-1) end)
                addBtn(6 + bW1*4, bY1, bW1, rowH, "-10m", "0", "e", function() FlightCore.adjustTargetAlt(-10) end)
                addBtn(7 + bW1*5, bY1, bW1, rowH, "LOCK", "0", "3", function() FlightCore.lockCurrentAlt() end)

                -- Row 2: Throttle
                local bY2 = bY1 + rowH + 1
                local bW2 = math.floor((w - 5) / 4)
                addBtn(2, bY2, bW2, rowH, "BASE +1.0", "3", "0", function() FlightCore.adjustBaseThrottle(1.0) end)
                addBtn(3 + bW2, bY2, bW2, rowH, "BASE -1.0", "9", "0", function() FlightCore.adjustBaseThrottle(-1.0) end)
                addBtn(4 + bW2*2, bY2, bW2, rowH, "BASE +0.1", "b", "0", function() FlightCore.adjustBaseThrottle(0.1) end)
                addBtn(5 + bW2*3, bY2, bW2, rowH, "BASE -0.1", "7", "0", function() FlightCore.adjustBaseThrottle(-0.1) end)

                -- Row 3: Flight Ops
                local bY3 = bY2 + rowH + 1
                local bW3 = math.floor((w - 5) / 4)
                addBtn(2, bY3, bW3, rowH, "HOLD", "5", "0", function() FlightCore.holdAltitude() end)
                addBtn(3 + bW3, bY3, bW3, rowH, "CALIB", "a", "0", function() FlightCore.startCalibration() end)
                addBtn(4 + bW3*2, bY3, bW3, rowH, "RE-SCAN", "b", "0", function() FlightCore.scanQuadTurtles(); FlightCore.state.statusMsg="Re-scanned!" end)
                addBtn(5 + bW3*3, bY3, bW3, rowH, "STOP", "e", "0", function() FlightCore.stopEngines() end)
            else
                -- 3x3 ~ 4x5 精簡版
                self:safeBlit(scr, 2, 3, string.format("[%s] ALT:%.0f->%.0f | B:%.2f", FlightCore.state.mode:sub(1,6), currAlt, FlightCore.state.targetAlt, FlightCore.state.baseThrottle), "0", "b")

                local rowH = math.max(1, math.floor((h - 7) / 3))

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
            end

        elseif scr.currentView == "NAV" then
            local currAlt = FlightCore.altiSensor and FlightCore.altiSensor.getHeight() or 0
            self:safeBlit(scr, 2, 3, string.format("ALT: %6.1fm -> TGT: %6.1fm", currAlt, FlightCore.state.targetAlt), "0", "b")

            local gridY = 5
            local colW = math.floor((w - 3) / 2)
            local rowH = math.max(2, math.floor((h - gridY - 2) / 3))

            addBtn(2, gridY, colW, rowH, isFull and "[ 0m LANDING ]" or "0m LAND", "e", "0", function() FlightCore.setTargetAlt(0) end)
            addBtn(3 + colW, gridY, colW, rowH, isFull and "[ 80m TREETOP ]" or "80m TREE", "5", "0", function() FlightCore.setTargetAlt(80) end)

            addBtn(2, gridY + rowH + 1, colW, rowH, isFull and "[ 150m CRUISE ]" or "150m CRZ", "3", "0", function() FlightCore.setTargetAlt(150) end)
            addBtn(3 + colW, gridY + rowH + 1, colW, rowH, isFull and "[ 200m CALIBRATE ]" or "200m CAL", "a", "0", function() FlightCore.setTargetAlt(200) end)

            addBtn(2, gridY + (rowH + 1)*2, colW, rowH, isFull and "[ 300m HIGH-ALT ]" or "300m HIGH", "b", "0", function() FlightCore.setTargetAlt(300) end)
            addBtn(3 + colW, gridY + (rowH + 1)*2, colW, rowH, isFull and "[ LOCK CURRENT ]" or "LOCK CURR", "4", "0", function() FlightCore.lockCurrentAlt() end)

        elseif scr.currentView == "SYS" then
            local slots = {
                {slot="FL", col=1, row=1, name="FL Quad"},
                {slot="FR", col=2, row=1, name="FR Quad"},
                {slot="BL", col=1, row=2, name="BL Quad"},
                {slot="BR", col=2, row=2, name="BR Quad"}
            }
            local cardW = math.floor((w - 3) / 2)
            local cardH = math.max(3, math.floor((h - 8) / 2))
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
            addBtn(2, btmY, btmW, 2, isFull and "[ RE-SCAN HARDWARE ]" or "RE-SCAN HW", "3", "0", function() FlightCore.scanQuadTurtles(); FlightCore.state.statusMsg="Re-scanned!" end)
            addBtn(3 + btmW, btmY, btmW, 2, isFull and "[ STOP ALL ENGINES ]" or "STOP ALL", "e", "0", function() FlightCore.stopEngines() end)
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
        if event == "peripheral" or event == "peripheral_detach" or event == "monitor_resize" then
            self:refreshScreens()
            return
        end
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

-- Tom's Peripherals GPU 需要至少 1 tick 才能回傳正確解析度
-- 在啟動 parallel loop 之前重新掃描一次，確保尺寸正確
sleep(0.1)
pcall(function() activeDriver:refreshScreens() end)
print(string.format("Screens (after reinit): %d", #activeDriver.screens))

local function flightLoop()
    while true do
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
        if event == "modem_message" then
            FlightCore.handleModemMessage(eventData[2], eventData[3], eventData[4], eventData[5], eventData[6])
        elseif event == "peripheral" or event == "peripheral_detach" or event == "monitor_resize" or event == "tm_monitor_resize" or event == "directgpu_resize" then
            if activeDriver and activeDriver.refreshScreens then
                pcall(function() activeDriver:refreshScreens() end)
            end
            pcall(function() FlightCore.scanQuadTurtles() end)
        end
        activeDriver:handleEvent(table.unpack(eventData))
        if event == "terminate" then
            FlightCore.stopEngines()
            break
        end
    end
end

parallel.waitForAny(flightLoop, renderLoop, eventLoop)
