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
    rampRate = 0.015,         -- 平緩線性加力速率 (每 tick +0.015)
    fwdThrottle = 0.0,        -- 前進/後退油門 (-15.0 ~ 15.0，正為前進，負為後退)
    turnThrottle = 0.0,       -- 轉向推力 (-15.0 ~ 15.0，正為右推/面向右，負為左推/面向左)
    strafeThrottle = 0.0      -- 相容別名
}

-- 1.3 導航與定位狀態 (Navigation & Positioning)
FlightCore.nav = {
    x = nil,
    y = nil,
    z = nil,
    yaw = 0.0,            -- 當前航向角 (0~359°, 0=北, 90=東, 180=南, 270=西)
    pitch = 0.0,
    roll = 0.0,
    speed = 0.0,          -- 水平線速度 (m/s)
    source = "BARO",      -- "INS", "GPS", "BARO", "NONE"
    targetHeading = 0.0,  -- 目標航向 (0~359°)
    headingHold = false,  -- 航向鎖定開關
    targetX = nil,        -- 航點目標 X
    targetZ = nil,        -- 航點目標 Z
    wpActive = false,     -- 航點自動巡航自駕儀
    arrivalRadius = 20.0  -- 目的地到達判定半徑 (可偏差半徑 20 格)
}

-- 垂直速度阻尼高度控制器 (平滑懸停，杜絕驟開驟停與彈簧震盪)
FlightCore.altPID = AltController.new(0.25, 0.01, 0.85, 5.0)

-- 支援一側/象限/平移方向配置多個引擎 (陣列結構)
FlightCore.engines = {
    FL = {}, FR = {}, BL = {}, BR = {},
    FWD = {}, BWD = {}, LEFT = {}, RIGHT = {}
}
FlightCore.engineOutputs  = {
    FL = 0, FR = 0, BL = 0, BR = 0,
    FWD = 0, BWD = 0, LEFT = 0, RIGHT = 0
}
FlightCore.virtualOutputs = {
    FL = 0.0, FR = 0.0, BL = 0.0, BR = 0.0,
    FWD = 0.0, BWD = 0.0, LEFT = 0.0, RIGHT = 0.0
}

-- 烏龜心跳與遙測健康度資料庫 (按 Turtle ID 索引)
FlightCore.turtles = {}

FlightCore.altiSensor = nil
FlightCore.gimbalSensor = nil
FlightCore.navTable = nil
FlightCore.gimbalAvailable = false
FlightCore.modems = {}
FlightCore.outputSides = {"bottom", "top", "left", "right", "back"}
FlightCore.pwmTick = 0
FlightCore.gpsTick = 0

local hasCCPE, CCPE_SS = pcall(require, "ccpe.sensor_system")

function FlightCore.updateNavigation()
    -- 1. 優先嘗試 CCPE (INS / AIC)
    if hasCCPE and CCPE_SS and CCPE_SS.isOnBody and CCPE_SS.isOnBody() then
        local okPos, pos = pcall(function() return CCPE_SS.getBodyPosition() or CCPE_SS.getPosition() end)
        local okVel, vel = pcall(function() return CCPE_SS.getVelocity() end)
        local okAng, ang = pcall(function() return CCPE_SS.getAngles() end)

        if okPos and pos and type(pos) == "table" then
            FlightCore.nav.x = pos.x
            FlightCore.nav.y = pos.y
            FlightCore.nav.z = pos.z
            FlightCore.nav.source = "INS"
        end

        if okVel and vel and type(vel) == "table" then
            FlightCore.nav.speed = math.sqrt((vel.x or 0)^2 + (vel.z or 0)^2)
        end

        if okAng and ang and type(ang) == "table" then
            FlightCore.nav.pitch = ang.pitch or 0
            FlightCore.nav.roll = ang.roll or 0
            FlightCore.nav.yaw = ((ang.yaw or 0) % 360 + 360) % 360
        end
        return
    end

    -- 2. 嘗試 Navigation Table (Create: Avionics)
    if not FlightCore.navTable then
        FlightCore.navTable = peripheral.find("navigation_table")
    end
    if FlightCore.navTable then
        local ok, hdg = pcall(function() return FlightCore.navTable.getHeading() end)
        if ok and hdg then
            FlightCore.nav.yaw = (hdg % 360 + 360) % 360
        end
    end

    -- 3. 姿態儀 (Gimbal Sensor)
    local p, r = FlightCore.getGimbalData()
    FlightCore.nav.pitch = p
    FlightCore.nav.roll = r

    -- 4. 高度計 (Altitude Sensor)
    if FlightCore.altiSensor then
        local ok, h = pcall(function() return FlightCore.altiSensor.getHeight() end)
        if ok and h then
            FlightCore.nav.y = h
            if FlightCore.nav.source ~= "GPS" then
                FlightCore.nav.source = "BARO"
            end
        end
    end

    -- 5. GPS 非阻塞定期探測 (每 2 秒嘗試一次)
    FlightCore.gpsTick = (FlightCore.gpsTick + 1) % 40
    if FlightCore.gpsTick == 0 and gps and #FlightCore.modems > 0 then
        local gx, gy, gz = gps.locate(0.02)
        if gx then
            FlightCore.nav.x = gx
            FlightCore.nav.y = gy
            FlightCore.nav.z = gz
            FlightCore.nav.source = "GPS"
        end
    end
end

local function resolveRole(lbl)
    if not lbl or lbl == "" then return nil end
    local u = string.upper(tostring(lbl)):gsub("^%s*(.-)%s*$", "%1")

    -- 1. 四象限垂直升力 (FL, FR, BL, BR)
    if u == "FL" or u:find("^FL[-_ ]") or u:find("^前左") or u == "FRONT_LEFT" then return "FL"
    elseif u == "FR" or u:find("^FR[-_ ]") or u:find("^前右") or u == "FRONT_RIGHT" then return "FR"
    elseif u == "BL" or u:find("^BL[-_ ]") or u:find("^後左") or u == "BACK_LEFT" then return "BL"
    elseif u == "BR" or u:find("^BR[-_ ]") or u:find("^後右") or u == "BACK_RIGHT" then return "BR"
    end

    -- 2. 前進 / 前推 (Forward / FWD)
    if u == "FWD" or u == "FORWARD" or u == "FRONT" or u == "前進" or u == "前推" or u == "前" or
       u:find("^FWD[-_ ]") or u:find("^FORWARD[-_ ]") or u:find("^PUSH_FWD") or u:find("^PUSH_FORWARD") or
       u:find("^前進") or u:find("^前推") then
        return "FWD"
    end

    -- 3. 後退 / 後推 (Backward / BWD)
    if u == "BWD" or u == "BACK" or u == "BACKWARD" or u == "REVERSE" or u == "後退" or u == "後推" or u == "後" or
       u:find("^BWD[-_ ]") or u:find("^BACK[-_ ]") or u:find("^PUSH_BWD") or u:find("^PUSH_BACK") or
       u:find("^後退") or u:find("^後推") then
        return "BWD"
    end

    -- 4. 左推 / 左轉向 (Left Turn / 面向左)
    if u == "LEFT" or u == "TURN_LEFT" or u == "PUSH_LEFT" or u == "ORIENT_LEFT" or u == "TL" or
       u == "左" or u == "左推" or u == "左轉" or u == "左舵" or u == "左向" or u == "左平移" or
       u:find("^LEFT[-_ ]") or u:find("^PUSH_LEFT") or u:find("^TURN_LEFT") or
       u:find("^左推") or u:find("^左轉") or u:find("^左舵") then
        return "LEFT"
    end

    -- 5. 右推 / 右轉向 (Right Turn / 面向右)
    if u == "RIGHT" or u == "TURN_RIGHT" or u == "PUSH_RIGHT" or u == "ORIENT_RIGHT" or u == "TR" or
       u == "右" or u == "右推" or u == "右轉" or u == "右舵" or u == "右向" or u == "右平移" or
       u:find("^RIGHT[-_ ]") or u:find("^PUSH_RIGHT") or u:find("^TURN_RIGHT") or
       u:find("^右推") or u:find("^右轉") or u:find("^右舵") then
        return "RIGHT"
    end

    return nil
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
    for _, r in ipairs({"FL", "FR", "BL", "BR", "FWD", "BWD", "LEFT", "RIGHT"}) do
        FlightCore.engines[r] = {}
    end

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

            local role = resolveRole(label)
            if role and FlightCore.engines[role] then
                table.insert(FlightCore.engines[role], wrapped)
            else
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
        if msg.type == "HEARTBEAT" and msg.id then
            local determinedRole = msg.role
            if not determinedRole or determinedRole == "UNKNOWN" or determinedRole == "" then
                determinedRole = resolveRole(msg.label) or "UNKNOWN"
            else
                local r = resolveRole(determinedRole) or resolveRole(msg.label)
                if r then determinedRole = r end
            end

            FlightCore.turtles[msg.id] = {
                role = determinedRole,
                label = msg.label or ("Turtle-" .. msg.id),
                sig = msg.sig or 0,
                ver = msg.ver or "unknown",
                lastSeen = os.epoch("utc")
            }
        end
    end
end

function FlightCore.outputToEngines(sigFL, sigFR, sigBL, sigBR, sigFWD, sigBWD, sigLEFT, sigRIGHT)
    sigFL = sigFL or FlightCore.engineOutputs.FL or 0
    sigFR = sigFR or FlightCore.engineOutputs.FR or 0
    sigBL = sigBL or FlightCore.engineOutputs.BL or 0
    sigBR = sigBR or FlightCore.engineOutputs.BR or 0
    sigFWD = sigFWD or FlightCore.engineOutputs.FWD or 0
    sigBWD = sigBWD or FlightCore.engineOutputs.BWD or 0
    sigLEFT = sigLEFT or FlightCore.engineOutputs.LEFT or 0
    sigRIGHT = sigRIGHT or FlightCore.engineOutputs.RIGHT or 0

    local targets = {
        FL = sigFL, FR = sigFR, BL = sigBL, BR = sigBR,
        FWD = sigFWD, BWD = sigBWD, LEFT = sigLEFT, RIGHT = sigRIGHT
    }

    -- 1. 廣播至所有已連接之 Wired/Wireless Modem (Channel 100)
    local payload = {
        type = "FLIGHT_SYNC",
        FL = sigFL,
        FR = sigFR,
        BL = sigBL,
        BR = sigBR,
        -- 前進/前推別名廣播
        FWD = sigFWD,
        FORWARD = sigFWD,
        FRONT = sigFWD,
        ["前進"] = sigFWD,
        ["前推"] = sigFWD,
        PUSH_FWD = sigFWD,
        PUSH_FORWARD = sigFWD,
        -- 後退/後推別名廣播
        BWD = sigBWD,
        BACK = sigBWD,
        BACKWARD = sigBWD,
        REVERSE = sigBWD,
        ["後退"] = sigBWD,
        ["後推"] = sigBWD,
        PUSH_BWD = sigBWD,
        PUSH_BACK = sigBWD,
        -- 左推別名廣播
        LEFT = sigLEFT,
        STRAFE_LEFT = sigLEFT,
        PUSH_LEFT = sigLEFT,
        TL = sigLEFT,
        ["左"] = sigLEFT,
        ["左推"] = sigLEFT,
        ["左平移"] = sigLEFT,
        -- 右推別名廣播
        RIGHT = sigRIGHT,
        STRAFE_RIGHT = sigRIGHT,
        PUSH_RIGHT = sigRIGHT,
        TR = sigRIGHT,
        ["右"] = sigRIGHT,
        ["右推"] = sigRIGHT,
        ["右平移"] = sigRIGHT,
        timestamp = os.epoch("utc")
    }

    -- 針對所有已連線/回報心跳之烏龜，以其具體 ID 與 Label 精準注入 payload，確保 100% 命中
    for id, t in pairs(FlightCore.turtles) do
        local r = t.role
        if r and targets[r] ~= nil then
            local val = targets[r]
            payload[id] = val
            if t.label and t.label ~= "" then
                payload[t.label] = val
                payload[string.upper(t.label)] = val
            end
        end
    end

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

function FlightCore.setForwardThrottle(val)
    FlightCore.state.fwdThrottle = math.max(-15.0, math.min(15.0, val))
    if FlightCore.state.fwdThrottle > 0 then
        FlightCore.state.statusMsg = string.format("Forward Thrust: %.1f", FlightCore.state.fwdThrottle)
    elseif FlightCore.state.fwdThrottle < 0 then
        FlightCore.state.statusMsg = string.format("Backward Thrust: %.1f", -FlightCore.state.fwdThrottle)
    else
        FlightCore.state.statusMsg = "Forward Thrust: OFF"
    end
end

function FlightCore.adjustForwardThrottle(delta)
    FlightCore.setForwardThrottle(FlightCore.state.fwdThrottle + delta)
end

function FlightCore.setTurnThrottle(val)
    FlightCore.state.turnThrottle = math.max(-15.0, math.min(15.0, val))
    FlightCore.state.strafeThrottle = FlightCore.state.turnThrottle
    if FlightCore.state.turnThrottle > 0 then
        FlightCore.state.statusMsg = string.format("Turn Right Thrust: %.1f", FlightCore.state.turnThrottle)
    elseif FlightCore.state.turnThrottle < 0 then
        FlightCore.state.statusMsg = string.format("Turn Left Thrust: %.1f", -FlightCore.state.turnThrottle)
    else
        FlightCore.state.statusMsg = "Turn Thrust: OFF"
    end
end

function FlightCore.adjustTurnThrottle(delta)
    FlightCore.setTurnThrottle((FlightCore.state.turnThrottle or 0) + delta)
end

function FlightCore.setStrafeThrottle(val)
    FlightCore.setTurnThrottle(val)
end

function FlightCore.adjustStrafeThrottle(delta)
    FlightCore.adjustTurnThrottle(delta)
end

function FlightCore.stopHorizontalThrust()
    FlightCore.state.fwdThrottle = 0.0
    FlightCore.state.turnThrottle = 0.0
    FlightCore.state.strafeThrottle = 0.0
    FlightCore.nav.wpActive = false
    FlightCore.virtualOutputs.FWD = 0.0
    FlightCore.virtualOutputs.BWD = 0.0
    FlightCore.virtualOutputs.LEFT = 0.0
    FlightCore.virtualOutputs.RIGHT = 0.0
    FlightCore.engineOutputs.FWD = 0
    FlightCore.engineOutputs.BWD = 0
    FlightCore.engineOutputs.LEFT = 0
    FlightCore.engineOutputs.RIGHT = 0
    FlightCore.state.statusMsg = "Directional Thrust: STOPPED"
end

function FlightCore.setWaypoint(x, z, radius)
    FlightCore.nav.targetX = tonumber(x)
    FlightCore.nav.targetZ = tonumber(z)
    FlightCore.nav.arrivalRadius = tonumber(radius) or 20.0
    FlightCore.nav.wpActive = true
    FlightCore.state.statusMsg = string.format("Waypoint: (%d,%d) R:%.0fm", x, z, FlightCore.nav.arrivalRadius)
end

function FlightCore.cancelWaypoint()
    FlightCore.nav.wpActive = false
    FlightCore.state.fwdThrottle = 0.0
    FlightCore.state.statusMsg = "Waypoint Navigation CANCELLED"
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
    FlightCore.state.fwdThrottle = 0.0
    FlightCore.state.turnThrottle = 0.0
    FlightCore.state.strafeThrottle = 0.0
    FlightCore.nav.wpActive = false
    FlightCore.state.statusMsg = "Engines Stopped (IDLE)"
    FlightCore.engineOutputs = { FL = 0, FR = 0, BL = 0, BR = 0, FWD = 0, BWD = 0, LEFT = 0, RIGHT = 0 }
    FlightCore.virtualOutputs = { FL = 0.0, FR = 0.0, BL = 0.0, BR = 0.0, FWD = 0.0, BWD = 0.0, LEFT = 0.0, RIGHT = 0.0 }
    FlightCore.outputToEngines(0, 0, 0, 0, 0, 0, 0, 0)
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

function FlightCore.setTargetHeading(hdg)
    FlightCore.nav.targetHeading = (math.floor(hdg) % 360 + 360) % 360
    FlightCore.nav.headingHold = true
    FlightCore.state.statusMsg = string.format("Target Heading: %03d*", FlightCore.nav.targetHeading)
end

function FlightCore.adjustTargetHeading(delta)
    FlightCore.setTargetHeading(FlightCore.nav.targetHeading + delta)
end

function FlightCore.toggleHeadingHold()
    FlightCore.nav.headingHold = not FlightCore.nav.headingHold
    if FlightCore.nav.headingHold then
        FlightCore.nav.targetHeading = math.floor(FlightCore.nav.yaw or 0)
        FlightCore.state.statusMsg = string.format("Heading Hold: %03d*", FlightCore.nav.targetHeading)
    else
        FlightCore.state.statusMsg = "Heading Hold: OFF"
    end
end

function FlightCore.syncHeading()
    FlightCore.nav.targetHeading = math.floor(FlightCore.nav.yaw or 0)
    FlightCore.nav.headingHold = true
    FlightCore.state.statusMsg = string.format("Heading Synced: %03d*", FlightCore.nav.targetHeading)
end

function FlightCore.updateFlightLogic()
    FlightCore.pwmTick = (FlightCore.pwmTick + 1) % 10
    FlightCore.updateNavigation()

    if FlightCore.state.mode == "IDLE" then
        FlightCore.engineOutputs = { FL = 0, FR = 0, BL = 0, BR = 0, FWD = 0, BWD = 0, LEFT = 0, RIGHT = 0 }
        FlightCore.virtualOutputs = { FL = 0.0, FR = 0.0, BL = 0.0, BR = 0.0, FWD = 0.0, BWD = 0.0, LEFT = 0.0, RIGHT = 0.0 }
        FlightCore.outputToEngines(0, 0, 0, 0, 0, 0, 0, 0)
        return
    end

    if not FlightCore.altiSensor then
        FlightCore.altiSensor = peripheral.find("altitude_sensor")
    end

    local currentAlt = FlightCore.altiSensor and FlightCore.altiSensor.getHeight() or (FlightCore.nav.y or 0)
    local currentVspeed = FlightCore.altiSensor and FlightCore.altiSensor.getVerticalSpeed() or 0
    local currPitch = FlightCore.nav.pitch or 0
    local currRoll = FlightCore.nav.roll or 0

    local function approach(current, target, maxStep)
        if current < target then
            return math.min(target, current + maxStep)
        else
            return math.max(target, current - maxStep)
        end
    end

    -- 1. 航點自動導航邏輯 (Waypoint Autopilot Navigation - 偏差半徑 20 格容許)
    if FlightCore.nav.wpActive and FlightCore.nav.targetX and FlightCore.nav.targetZ then
        if FlightCore.nav.x and FlightCore.nav.z then
            local dx = FlightCore.nav.targetX - FlightCore.nav.x
            local dz = FlightCore.nav.targetZ - FlightCore.nav.z
            local dist = math.sqrt(dx * dx + dz * dz)
            local acceptRadius = FlightCore.nav.arrivalRadius or 20.0

            if dist <= acceptRadius then
                -- 進入目的地容許半徑 (20 格) -> 抵達目的地，停止前進引擎
                FlightCore.nav.wpActive = false
                FlightCore.state.fwdThrottle = 0.0
                FlightCore.state.statusMsg = string.format("ARRIVED DEST! (Dist: %.1fm <= %dm)", dist, math.floor(acceptRadius))
            else
                -- 計算指向目標的航向角 (0=北, 90=東, 180=南, 270=西)
                local targetHeading = (math.deg(math.atan2(dx, -dz)) + 360) % 360
                FlightCore.nav.targetHeading = targetHeading
                FlightCore.nav.headingHold = true

                local yawDiff = ((targetHeading - (FlightCore.nav.yaw or 0) + 180) % 360) - 180
                if math.abs(yawDiff) < 35 then
                    -- 航向基本對準，主前進引擎持續推力巡航
                    local cruisePower = math.min(15.0, math.max(4.0, dist * 0.12))
                    FlightCore.state.fwdThrottle = cruisePower
                else
                    -- 航向偏差較大，減速微推以便快速轉向
                    FlightCore.state.fwdThrottle = 1.5
                end
            end
        end
    end

    -- 2. 轉向自駕儀計算 (右推使船面向右，左推使船面向左)
    local autoTurnRight = 0.0
    local autoTurnLeft = 0.0
    local yawCorr = 0.0

    if FlightCore.nav.headingHold and FlightCore.nav.yaw then
        local yawDiff = ((FlightCore.nav.targetHeading - FlightCore.nav.yaw + 180) % 360) - 180
        -- 垂直升力差動轉向輔助 (Differential Lift Steering)
        yawCorr = math.max(-1.5, math.min(1.5, yawDiff * 0.05))

        -- 專用轉向推力引擎 (右推讓船面向右，左推讓船面向左)
        if yawDiff > 1.2 then
            -- 目標在右方，需要順時針右轉
            autoTurnRight = math.min(15.0, math.max(1.0, yawDiff * 0.20 + (yawDiff > 8 and 2.5 or 0.5)))
            autoTurnLeft = 0.0
        elseif yawDiff < -1.2 then
            -- 目標在左方，需要逆時針左轉
            autoTurnLeft = math.min(15.0, math.max(1.0, -yawDiff * 0.20 + (-yawDiff > 8 and 2.5 or 0.5)))
            autoTurnRight = 0.0
        end
    end

    -- 3. 飛行高度模式 (CALIBRATING / HOLD_ALT)
    if FlightCore.state.mode == "CALIBRATING" then
        if FlightCore.state.calibPhase == "GROUND_SEARCH" then
            FlightCore.state.calibThrottle = FlightCore.state.calibThrottle + FlightCore.state.rampRate
            FlightCore.state.baseThrottle = FlightCore.state.calibThrottle

            local altDelta = currentAlt - FlightCore.state.calibStartAlt
            if altDelta > 0.3 or currentVspeed > 0.15 then
                FlightCore.state.calibPhase = "STABILIZE"
                FlightCore.state.calibTargetAlt = FlightCore.state.calibTargetAlt
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

        local rawFL = math.max(0, math.min(15, FlightCore.state.baseThrottle + thrustAdj + pitchCorr - rollCorr - yawCorr))
        local rawFR = math.max(0, math.min(15, FlightCore.state.baseThrottle + thrustAdj + pitchCorr + rollCorr + yawCorr))
        local rawBL = math.max(0, math.min(15, FlightCore.state.baseThrottle + thrustAdj - pitchCorr - rollCorr - yawCorr))
        local rawBR = math.max(0, math.min(15, FlightCore.state.baseThrottle + thrustAdj - pitchCorr + rollCorr + yawCorr))

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
            if FlightCore.nav.wpActive and FlightCore.nav.targetX then
                local dx = FlightCore.nav.targetX - (FlightCore.nav.x or 0)
                local dz = FlightCore.nav.targetZ - (FlightCore.nav.z or 0)
                local d = math.sqrt(dx*dx + dz*dz)
                FlightCore.state.statusMsg = string.format("NAV -> (%d,%d) D:%.0fm HDG:%03d*", math.floor(FlightCore.nav.targetX), math.floor(FlightCore.nav.targetZ), d, math.floor(FlightCore.nav.targetHeading))
            elseif FlightCore.nav.headingHold then
                FlightCore.state.statusMsg = string.format("Hold: %.1fm | HDG: %03d* (Tgt: %03d*)", currentAlt, math.floor(FlightCore.nav.yaw), FlightCore.nav.targetHeading)
            else
                FlightCore.state.statusMsg = string.format("Holding Alt: %.1fm (Base: %.2f)", currentAlt, FlightCore.state.baseThrottle)
            end
        end
    end

    -- 4. 轉向與推進推力分配 (Turn & Forward/Backward Slew)
    local manTurn = FlightCore.state.turnThrottle or FlightCore.state.strafeThrottle or 0.0
    local manTurnRight = math.max(0, manTurn)
    local manTurnLeft = math.max(0, -manTurn)

    local targetRIGHT = math.max(manTurnRight, autoTurnRight)
    local targetLEFT = math.max(manTurnLeft, autoTurnLeft)
    local targetFWD = math.max(0, FlightCore.state.fwdThrottle or 0)
    local targetBWD = math.max(0, -(FlightCore.state.fwdThrottle or 0))

    local maxTransSlew = 0.5
    FlightCore.virtualOutputs.FWD = approach(FlightCore.virtualOutputs.FWD or 0, targetFWD, maxTransSlew)
    FlightCore.virtualOutputs.BWD = approach(FlightCore.virtualOutputs.BWD or 0, targetBWD, maxTransSlew)
    FlightCore.virtualOutputs.LEFT = approach(FlightCore.virtualOutputs.LEFT or 0, targetLEFT, maxTransSlew)
    FlightCore.virtualOutputs.RIGHT = approach(FlightCore.virtualOutputs.RIGHT or 0, targetRIGHT, maxTransSlew)

    local function calcPWM(val)
        local intPart = math.floor(val or 0)
        local fracPart = (val or 0) - intPart
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
    FlightCore.engineOutputs.FWD = calcPWM(FlightCore.virtualOutputs.FWD)
    FlightCore.engineOutputs.BWD = calcPWM(FlightCore.virtualOutputs.BWD)
    FlightCore.engineOutputs.LEFT = calcPWM(FlightCore.virtualOutputs.LEFT)
    FlightCore.engineOutputs.RIGHT = calcPWM(FlightCore.virtualOutputs.RIGHT)

    FlightCore.outputToEngines(
        FlightCore.engineOutputs.FL,
        FlightCore.engineOutputs.FR,
        FlightCore.engineOutputs.BL,
        FlightCore.engineOutputs.BR,
        FlightCore.engineOutputs.FWD,
        FlightCore.engineOutputs.BWD,
        FlightCore.engineOutputs.LEFT,
        FlightCore.engineOutputs.RIGHT
    )
end

return FlightCore
