--[[
    modules/flight_core.lua
    VTOL Quad-Engine Flight Control Core (PID, Sensors, Turtle Modem Networking)
--]]

local FlightCore = {}

-- 1.1 高度與垂直阻尼控制器 (PD-V Controller: 專為 Minecraft/Create 物理引擎調校)
local AltController = {}
AltController.__index = AltController

function AltController.new(kp, ki, kd, maxAdj)
    local self = setmetatable({}, AltController)
    self.kp = kp or 0.25         -- 高度比例增益 (每 1m 誤差調節 0.25 推力)
    self.ki = ki or 0.01         -- 積分微調增益 (消除長效靜態誤差)
    self.kd = kd or 0.85         -- 垂直速度阻尼 (1 m/s 速度產生 0.85 反向煞車推力，徹底消除上下震盪)
    self.maxAdj = maxAdj or 5.0  -- 最大調節推力幅度 (防失控與推力突變)
    self.integral = 0
    self.lastTime = os.epoch("utc") / 1000
    return self
end

function AltController:reset()
    self.integral = 0
    self.lastTime = os.epoch("utc") / 1000
end

function AltController:update(altError, vspeed)
    local now = os.epoch("utc") / 1000
    local dt = now - self.lastTime
    if dt <= 0 or dt > 0.5 then dt = 0.05 end
    self.lastTime = now

    local p = self.kp * altError
    
    self.integral = self.integral + altError * dt
    local iLimit = 2.0
    if self.integral > iLimit then self.integral = iLimit end
    if self.integral < -iLimit then self.integral = -iLimit end
    local i = self.ki * self.integral

    local d = -self.kd * (vspeed or 0)

    local output = p + i + d
    if output > self.maxAdj then output = self.maxAdj end
    if output < -self.maxAdj then output = -self.maxAdj end
    return output
end

-- 1.2 飛控狀態
FlightCore.state = {
    mode = "IDLE",            -- "IDLE", "HOLD_ALT", "CALIBRATING"
    targetAlt = 200.0,        -- 預設目標高度 200m
    virtualAlt = 100.0,       -- 虛擬平滑軌跡高度
    baseThrottle = 3.0,       -- 浮點基準推力 (0.01 ~ 15.0)
    statusMsg = "System Ready",
    stableTimer = 0,          -- 穩定停留計時器 (秒)
    calibStartAlt = 0,        -- 校正啟動高度
    calibTargetAlt = 0,       -- 校正爬升目標高度 (啟動高度 + 3m)
    calibPhase = "GROUND_SEARCH", -- "GROUND_SEARCH", "STABILIZE"
    calibThrottle = 0.0,      -- 校正微調油門
    rampRate = 0.015          -- 平緩線性加力速率 (每 tick +0.015)
}

-- 垂直速度阻尼高度控制器 (平滑懸停，杜絕驟開驟停與彈簧震盪)
FlightCore.altPID = AltController.new(0.25, 0.01, 0.85, 5.0)

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
    local currentAlt = FlightCore.altiSensor.getHeight() or 0
    FlightCore.state.mode = "CALIBRATING"
    FlightCore.state.calibPhase = "GROUND_SEARCH"
    FlightCore.state.calibStartAlt = currentAlt
    -- 僅在起飛點上方 3 公尺處進行懸停校正，避免過度爬升暴衝
    FlightCore.state.calibTargetAlt = currentAlt + 3.0
    FlightCore.state.calibThrottle = 0.0
    FlightCore.state.rampRate = 0.015
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

    local function approach(current, target, maxStep)
        if current < target then
            return math.min(target, current + maxStep)
        else
            return math.max(target, current - maxStep)
        end
    end

    if FlightCore.state.mode == "CALIBRATING" then
        if FlightCore.state.calibPhase == "GROUND_SEARCH" then
            FlightCore.state.calibThrottle = FlightCore.state.calibThrottle + FlightCore.state.rampRate
            FlightCore.state.baseThrottle = FlightCore.state.calibThrottle

            local altDelta = currentAlt - FlightCore.state.calibStartAlt
            if altDelta > 0.3 or currentVspeed > 0.15 then
                FlightCore.state.calibPhase = "STABILIZE"
                FlightCore.state.targetAlt = FlightCore.state.calibTargetAlt
                -- 扣除起飛初期的慣性累積量，精確鎖定懸停初值
                FlightCore.state.baseThrottle = math.max(0.5, FlightCore.state.calibThrottle - 0.15)
                FlightCore.state.stableTimer = 0
                FlightCore.state.statusMsg = string.format("Lift Found (%.2f)! Stabilizing +3m...", FlightCore.state.baseThrottle)
                FlightCore.altPID:reset()
            else
                FlightCore.state.statusMsg = string.format("Searching Lift: %.2f (Alt: %.1f)", FlightCore.state.baseThrottle, currentAlt)
            end

            FlightCore.virtualOutputs.FL = FlightCore.state.baseThrottle
            FlightCore.virtualOutputs.FR = FlightCore.state.baseThrottle
            FlightCore.virtualOutputs.BL = FlightCore.state.baseThrottle
            FlightCore.virtualOutputs.BR = FlightCore.state.baseThrottle

        elseif FlightCore.state.calibPhase == "STABILIZE" then
            local targetAlt = FlightCore.state.calibTargetAlt
            local altError = targetAlt - currentAlt
            local thrustAdj = FlightCore.altPID:update(altError, currentVspeed)

            -- 閉環微調 BaseThrottle 尋找精確無漂移懸停點
            if currentVspeed > 0.05 then
                FlightCore.state.baseThrottle = math.max(0.1, FlightCore.state.baseThrottle - 0.001)
            elseif currentVspeed < -0.05 then
                FlightCore.state.baseThrottle = math.min(15.0, FlightCore.state.baseThrottle + 0.001)
            end

            local rawFL = math.max(0, math.min(15, FlightCore.state.baseThrottle + thrustAdj))
            local rawFR = math.max(0, math.min(15, FlightCore.state.baseThrottle + thrustAdj))
            local rawBL = math.max(0, math.min(15, FlightCore.state.baseThrottle + thrustAdj))
            local rawBR = math.max(0, math.min(15, FlightCore.state.baseThrottle + thrustAdj))

            local maxSlew = 0.35
            FlightCore.virtualOutputs.FL = approach(FlightCore.virtualOutputs.FL, rawFL, maxSlew)
            FlightCore.virtualOutputs.FR = approach(FlightCore.virtualOutputs.FR, rawFR, maxSlew)
            FlightCore.virtualOutputs.BL = approach(FlightCore.virtualOutputs.BL, rawBL, maxSlew)
            FlightCore.virtualOutputs.BR = approach(FlightCore.virtualOutputs.BR, rawBR, maxSlew)

            if math.abs(currentAlt - targetAlt) < 0.4 and math.abs(currentVspeed) < 0.1 then
                FlightCore.state.stableTimer = FlightCore.state.stableTimer + 0.05
                FlightCore.state.statusMsg = string.format("Calibrating: Stable %.1fs/4.0s (Base: %.2f)", FlightCore.state.stableTimer, FlightCore.state.baseThrottle)
                if FlightCore.state.stableTimer >= 4.0 then
                    FlightCore.state.mode = "HOLD_ALT"
                    FlightCore.state.targetAlt = targetAlt
                    FlightCore.state.statusMsg = string.format("Calib Done! Hover Base: %.2f", FlightCore.state.baseThrottle)
                    FlightCore.altPID:reset()
                end
            else
                FlightCore.state.stableTimer = 0
                if currentAlt < targetAlt - 0.4 then
                    FlightCore.state.statusMsg = string.format("Climbing: %.1fm -> %.1fm (Base: %.2f)", currentAlt, targetAlt, FlightCore.state.baseThrottle)
                elseif currentAlt > targetAlt + 0.4 then
                    FlightCore.state.statusMsg = string.format("Descending: %.1fm -> %.1fm (Base: %.2f)", currentAlt, targetAlt, FlightCore.state.baseThrottle)
                else
                    FlightCore.state.statusMsg = string.format("Stabilizing: %.1fm (V.S: %+.2f)", currentAlt, currentVspeed)
                end
            end
        end

    elseif FlightCore.state.mode == "HOLD_ALT" then
        local altError = FlightCore.state.targetAlt - currentAlt
        local thrustAdj = FlightCore.altPID:update(altError, currentVspeed)

        local pitchCorr = -currPitch * 0.05
        local rollCorr = currRoll * 0.05

        local rawFL = math.max(0, math.min(15, FlightCore.state.baseThrottle + thrustAdj + pitchCorr - rollCorr))
        local rawFR = math.max(0, math.min(15, FlightCore.state.baseThrottle + thrustAdj + pitchCorr + rollCorr))
        local rawBL = math.max(0, math.min(15, FlightCore.state.baseThrottle + thrustAdj - pitchCorr - rollCorr))
        local rawBR = math.max(0, math.min(15, FlightCore.state.baseThrottle + thrustAdj - pitchCorr + rollCorr))

        local maxSlew = 0.35
        FlightCore.virtualOutputs.FL = approach(FlightCore.virtualOutputs.FL, rawFL, maxSlew)
        FlightCore.virtualOutputs.FR = approach(FlightCore.virtualOutputs.FR, rawFR, maxSlew)
        FlightCore.virtualOutputs.BL = approach(FlightCore.virtualOutputs.BL, rawBL, maxSlew)
        FlightCore.virtualOutputs.BR = approach(FlightCore.virtualOutputs.BR, rawBR, maxSlew)

        local diff = FlightCore.state.targetAlt - currentAlt
        if diff > 0.8 then
            FlightCore.state.statusMsg = string.format("Climbing: %.1fm -> %.1fm (V.S: %+.1f)", currentAlt, FlightCore.state.targetAlt, currentVspeed)
        elseif diff < -0.8 then
            FlightCore.state.statusMsg = string.format("Descending: %.1fm -> %.1fm (V.S: %+.1f)", currentAlt, FlightCore.state.targetAlt, currentVspeed)
        else
            FlightCore.state.statusMsg = string.format("Holding Alt: %.1fm (Base: %.2f)", currentAlt, FlightCore.state.baseThrottle)
        end
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

return FlightCore
