--[[
    Create: Avionics & CC: Tweaked
    Standalone Flight Control Computer (FCC - 分散式純飛控大腦)
    Version: v4.0.7
    
    特點:
    - 0% 畫面渲染開銷 (No UI Rendering Overhead)，運算時間 < 0.2ms
    - 20Hz 高頻穩定 PD-V 垂直升力與高度控制，永不卡頓
    - 支援 Channel 100/101 烏龜推進動力同步
    - 支援 Channel 102 遙測狀態廣播 (10Hz) 與 Channel 103 駕駛艙螢幕指令接收
    - 徹底根除 "Too long without yielding"
--]]

local VERSION = "v4.0.7"

-- ========================================================
-- PART 1: 飛控與動力控制核心 (FLIGHT & PROPULSION CORE)
-- ========================================================

local FlightCore = {}

-- 1.1 標準 PID 控制器 (基於 v3.8.4 經典架構)
local PID = {}
PID.__index = PID

function PID.new(kp, ki, kd, minOutput, maxOutput)
    local self = setmetatable({}, PID)
    self.kp = kp or 0.50
    self.ki = ki or 0.02
    self.kd = kd or 0.60
    self.minOutput = minOutput or -1.50
    self.maxOutput = maxOutput or 1.80
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
    if dt <= 0 or dt > 0.5 then dt = 0.05 end
    self.lastTime = now

    -- P 比例項
    local p = self.kp * error

    -- I 積分項 (防飽和 Anti-Windup)
    self.integral = self.integral + error * dt
    local iMax = math.abs(self.maxOutput) * 0.5
    if self.integral > iMax then self.integral = iMax end
    if self.integral < -iMax then self.integral = -iMax end
    local i = self.ki * self.integral

    -- D 微分阻尼項 (v3.8.4 標準誤差微分)
    local derivative = (error - self.prevError) / dt
    self.prevError = error
    local d = self.kd * derivative

    local output = p + i + d
    if output > self.maxOutput then output = self.maxOutput end
    if output < self.minOutput then output = self.minOutput end
    return output
end

-- 1.2 飛控狀態 (高度維持與垂直控制)
FlightCore.state = {
    mode = "IDLE",            -- "IDLE", "HOLD_ALT", "CALIBRATING"
    targetAlt = 200.0,        -- 預設目標高度
    virtualAlt = 100.0,       -- 虛擬平滑軌跡高度
    baseThrottle = 1.0,       -- 浮點基準推力 (0.01 ~ 15.0)
    statusMsg = "System Ready",
    stableTimer = 0,          -- 穩定停留計時器 (秒)
    calibStartAlt = 0,        -- 校正啟動高度
    calibTargetAlt = 0,       -- 校正目標高度 (微升 +3m)
    calibPhase = "GROUND_SEARCH", -- "GROUND_SEARCH", "HOVER_3M"
    calibThrottle = 0.0,      -- 校正微調油門
    rampRate = 0.015,         -- 線性加力速率
    fwdThrottle = 0.0,        -- 前進/後退油門 (-15.0 ~ 15.0，正為前進，負為後退)
    turnThrottle = 0.0,       -- 轉向推力 (-15.0 ~ 15.0，正為右推/面向右，負為左推/面向左)
    strafeThrottle = 0.0      -- 相容別名
}

-- 1.3 導航與定位狀態 (Navigation & Positioning - 獨立於高度控制)
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

-- 高度控制專用 PID 控制器 (v3.8.4 經典基準: Kp=0.5, Ki=0.02, Kd=0.6, 嚴格輸出限幅 [-1.5, +1.8])
FlightCore.altPID = PID.new(0.50, 0.02, 0.60, -1.50, 1.80)

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

-- 高頻 Sigma-Delta 誤差擴散累加器 (消除 0.5s 週期性脈衝，實現 20Hz 逐 Tick 平滑交錯)
FlightCore.pwmAccumulator = {
    FL = 0.0, FR = 0.0, BL = 0.0, BR = 0.0,
    FWD = 0.0, BWD = 0.0, LEFT = 0.0, RIGHT = 0.0
}

-- 烏龜心跳與遙測健康度資料庫 (按 Turtle ID 索引)
FlightCore.turtles = {}

FlightCore.altiSensor = nil
FlightCore.gimbalSensor = nil
FlightCore.navTable = nil
FlightCore.aicSensor = nil
FlightCore.gimbalAvailable = false
FlightCore.modems = {}
FlightCore.outputSides = {"bottom", "top", "left", "right", "back"}
FlightCore.pwmTick = 0
FlightCore.gpsTick = 0

local function getCCPESensorSystem()
    local ok, ss = pcall(require, "ccpe.sensor_system")
    if ok and type(ss) == "table" then return ss end
    local ok2, ss2 = pcall(require, "ccpe/sensor_system")
    if ok2 and type(ss2) == "table" then return ss2 end
    if type(_G.ccpe) == "table" and type(_G.ccpe.sensor_system) == "table" then
        return _G.ccpe.sensor_system
    end
    return nil
end

function FlightCore.updateNavigation()
    -- 1. 優先嘗試 CCPE 物理實體感測系統 (ccpe.sensor_system)
    local ss = getCCPESensorSystem()
    if ss and ss.isOnBody then
        local okOn, onB = pcall(function() return ss.isOnBody() end)
        if okOn and onB then
            -- 讀取座標 (優先 getBodyPosition，其次 getPosition)
            local okPos, p1, p2, p3 = pcall(function()
                if ss.getBodyPosition then return ss.getBodyPosition()
                elseif ss.getPosition then return ss.getPosition()
                end
                return nil
            end)
            if okPos and p1 then
                if type(p1) == "table" then
                    local x = p1.x or p1[1]
                    local y = p1.y or p1[2]
                    local z = p1.z or p1[3]
                    if x and z then
                        FlightCore.nav.x = tonumber(x)
                        FlightCore.nav.y = tonumber(y) or FlightCore.nav.y
                        FlightCore.nav.z = tonumber(z)
                        FlightCore.nav.source = "AIC"
                    end
                elseif type(p1) == "number" and type(p3) == "number" then
                    FlightCore.nav.x = tonumber(p1)
                    FlightCore.nav.y = tonumber(p2) or FlightCore.nav.y
                    FlightCore.nav.z = tonumber(p3)
                    FlightCore.nav.source = "AIC"
                end
            end

            -- 備援：若 getBodyPosition 為空，嘗試 getSensors()
            if not FlightCore.nav.x and ss.getSensors then
                local okSensors, sList = pcall(ss.getSensors)
                if okSensors and type(sList) == "table" then
                    for _, s in ipairs(sList) do
                        if s.pos and (s.pos.x or s.pos[1]) then
                            FlightCore.nav.x = tonumber(s.pos.x or s.pos[1])
                            FlightCore.nav.y = tonumber(s.pos.y or s.pos[2]) or FlightCore.nav.y
                            FlightCore.nav.z = tonumber(s.pos.z or s.pos[3])
                            FlightCore.nav.source = "AIC"
                            break
                        end
                    end
                end
            end

            -- 讀取線速度與航速 (getVelocity)
            local okVel, v1, v2, v3 = pcall(function() return ss.getVelocity and ss.getVelocity() end)
            if okVel and v1 then
                if type(v1) == "table" then
                    local vx = v1.x or v1[1] or 0
                    local vz = v1.z or v1[3] or 0
                    FlightCore.nav.speed = math.sqrt(vx*vx + vz*vz)
                elseif type(v1) == "number" and type(v3) == "number" then
                    FlightCore.nav.speed = math.sqrt(v1*v1 + v3*v3)
                end
            end

            -- 讀取姿態與航向 (getAngles)
            local okAng, a1, a2, a3 = pcall(function() return ss.getAngles and ss.getAngles() end)
            if okAng and a1 then
                if type(a1) == "table" then
                    FlightCore.nav.pitch = a1.pitch or a1.x or a1[1] or FlightCore.nav.pitch
                    FlightCore.nav.roll = a1.roll or a1.z or a1[3] or FlightCore.nav.roll
                    local yaw = a1.yaw or a1.y or a1[2]
                    if yaw then FlightCore.nav.yaw = ((yaw % 360) + 360) % 360 end
                elseif type(a1) == "number" then
                    FlightCore.nav.pitch = a1
                    if type(a2) == "number" then FlightCore.nav.yaw = ((a2 % 360) + 360) % 360 end
                    if type(a3) == "number" then FlightCore.nav.roll = a3 end
                end
            end
        end
    end

    -- 2. 備援嘗試外設掃描 (Peripheral Scan for Navigation Table, Ship Reader, AIC, INS, etc.)
    if not (FlightCore.nav.x and FlightCore.nav.z) then
        for _, name in ipairs(peripheral.getNames()) do
            local pType = string.lower(tostring(peripheral.getType(name) or ""))
            if pType ~= "modem" and pType ~= "monitor" and pType ~= "drive" and pType ~= "speaker" and pType ~= "turtle" and pType ~= "computer" then
                local p = peripheral.wrap(name)
                if p then
                    local okPos, p1, p2, p3 = pcall(function()
                        if p.getBodyPosition then return p.getBodyPosition()
                        elseif p.getPosition then return p.getPosition()
                        elseif p.getLocation then return p.getLocation()
                        elseif p.getCoordinates then return p.getCoordinates()
                        elseif p.getWorldPosition then return p.getWorldPosition()
                        elseif p.getWorldPos then return p.getWorldPos()
                        elseif p.getShipPosition then return p.getShipPosition()
                        end
                        return nil
                    end)
                    if okPos and p1 then
                        if type(p1) == "table" then
                            local x = p1.x or p1[1]
                            local y = p1.y or p1[2]
                            local z = p1.z or p1[3]
                            if x and z then
                                FlightCore.nav.x = tonumber(x)
                                FlightCore.nav.y = tonumber(y) or FlightCore.nav.y
                                FlightCore.nav.z = tonumber(z)
                                FlightCore.nav.source = (pType:find("nav") or pType:find("table")) and "NAV_TABLE" or "INS"
                                break
                            end
                        elseif type(p1) == "number" and type(p3) == "number" then
                            FlightCore.nav.x = tonumber(p1)
                            FlightCore.nav.y = tonumber(p2) or FlightCore.nav.y
                            FlightCore.nav.z = tonumber(p3)
                            FlightCore.nav.source = (pType:find("nav") or pType:find("table")) and "NAV_TABLE" or "INS"
                            break
                        end
                    end
                end
            end
        end
    end

    -- 3. 備援海圖桌航向同步 (Navigation Table Heading)
    if not FlightCore.navTable then
        FlightCore.navTable = peripheral.find("navigation_table") or peripheral.find("avionics_navigation_table")
    end
    if FlightCore.navTable then
        local okHdg, hdg = pcall(function()
            if FlightCore.navTable.getHeading then return FlightCore.navTable.getHeading()
            elseif FlightCore.navTable.getYaw then return FlightCore.navTable.getYaw()
            end
            return nil
        end)
        if okHdg and hdg then
            FlightCore.nav.yaw = (hdg % 360 + 360) % 360
        end
    end

    -- 4. 嘗試 GPS 定位 (CC: Tweaked Wireless GPS 週期性探測)
    FlightCore.gpsTick = (FlightCore.gpsTick or 0) + 1
    if FlightCore.gpsTick % 10 == 0 and gps and FlightCore.nav.source ~= "AIC" and FlightCore.nav.source ~= "INS" and FlightCore.nav.source ~= "NAV_TABLE" then
        pcall(function()
            local gx, gy, gz = gps.locate(0.05)
            if gx and gz then
                FlightCore.nav.x = gx
                FlightCore.nav.y = gy
                FlightCore.nav.z = gz
                FlightCore.nav.source = "GPS"
            end
        end)
    end

    -- 5. 姿態儀 (Gimbal Sensor) 備援更新
    local p, r = FlightCore.getGimbalData()
    if p ~= 0 or r ~= 0 or not FlightCore.nav.pitch then
        FlightCore.nav.pitch = p
        FlightCore.nav.roll = r
    end

    -- 6. 高度計 (Altitude Sensor) 備援更新
    if FlightCore.altiSensor then
        local ok, h = pcall(function() return FlightCore.altiSensor.getHeight() end)
        if ok and h then
            FlightCore.nav.y = h
            if not FlightCore.nav.source or FlightCore.nav.source == "NONE" then
                FlightCore.nav.source = "BARO"
            end
        end
    end
end

function FlightCore.getHardwareInventory()
    -- 1. 顯示與 GPU 硬體
    local dgpu = false
    local tomCount = 0
    local monCount = 0
    for _, name in ipairs(peripheral.getNames()) do
        local pType = string.lower(tostring(peripheral.getType(name) or ""))
        if pType == "directgpu" or pType == "direct_gpu" then
            dgpu = true
        elseif pType == "tm_gpu" or pType == "tms_gpu" or pType == "tm_monitor" or pType:find("tom") then
            tomCount = tomCount + 1
        elseif pType == "monitor" then
            monCount = monCount + 1
        end
    end

    -- 2. 引擎動力
    local qFL = FlightCore.getQuadHealth("FL")
    local qFR = FlightCore.getQuadHealth("FR")
    local qBL = FlightCore.getQuadHealth("BL")
    local qBR = FlightCore.getQuadHealth("BR")
    local qFWD = FlightCore.getQuadHealth("FWD")
    local qBWD = FlightCore.getQuadHealth("BWD")
    local qLEFT = FlightCore.getQuadHealth("LEFT")
    local qRIGHT = FlightCore.getQuadHealth("RIGHT")

    local liftOnline = qFL.online + qFR.online + qBL.online + qBR.online
    local liftTotal  = qFL.total + qFR.total + qBL.total + qBR.total
    local cruiseOnline = qFWD.online + qBWD.online
    local cruiseTotal  = qFWD.total + qBWD.total
    local steerOnline  = qLEFT.online + qRIGHT.online
    local steerTotal   = qLEFT.total + qRIGHT.total

    -- 3. CCPE 物理實體與感測器探測
    local onPhysicsBody = false
    local ccpeSensors = {}
    local ss = getCCPESensorSystem()
    if ss and ss.isOnBody then
        local okOn, onB = pcall(function() return ss.isOnBody() end)
        if okOn and onB then
            onPhysicsBody = true
            local okS, sList = pcall(function() return ss.getSensors and ss.getSensors() end)
            if okS and type(sList) == "table" then
                ccpeSensors = sList
            end
        end
    end

    -- 4. 航電感測器
    local hasAlti = (FlightCore.altiSensor ~= nil)
    local hasGimbal = (FlightCore.gimbalSensor ~= nil) or FlightCore.gimbalAvailable
    local hasNavTable = (FlightCore.navTable ~= nil)
    local hasAIC = onPhysicsBody or (FlightCore.nav.source == "AIC" or FlightCore.nav.source == "INS")
    local modemCount = #(FlightCore.modems or {})

    return {
        displays = {
            hasDirectGpu = dgpu,
            tomsCount = tomCount,
            monitorsCount = monCount,
            totalScreens = (dgpu and 1 or 0) + tomCount + monCount
        },
        engines = {
            lift = { online = liftOnline, total = liftTotal, FL = qFL.total, FR = qFR.total, BL = qBL.total, BR = qBR.total, FL_on = qFL.online, FR_on = qFR.online, BL_on = qBL.online, BR_on = qBR.online },
            cruise = { online = cruiseOnline, total = cruiseTotal, FWD = qFWD.total, BWD = qBWD.total, FWD_on = qFWD.online, BWD_on = qBWD.online },
            steer = { online = steerOnline, total = steerTotal, LEFT = qLEFT.total, RIGHT = qRIGHT.total, LEFT_on = qLEFT.online, RIGHT_on = qRIGHT.online },
            totalOnline = liftOnline + cruiseOnline + steerOnline,
            totalCount = liftTotal + cruiseTotal + steerTotal
        },
        avionics = {
            onBody = onPhysicsBody,
            sensorCount = #ccpeSensors,
            aic = hasAIC,
            ins = (FlightCore.nav.source == "INS" or FlightCore.nav.source == "AIC"),
            navTable = hasNavTable,
            alti = hasAlti,
            gimbal = hasGimbal,
            modems = modemCount,
            navSource = FlightCore.nav.source or "NONE",
            hasCoords = (FlightCore.nav.x ~= nil and FlightCore.nav.z ~= nil),
            x = FlightCore.nav.x,
            y = FlightCore.nav.y,
            z = FlightCore.nav.z,
            yaw = FlightCore.nav.yaw,
            speed = FlightCore.nav.speed
        }
    }
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
    FlightCore.navTable = nil
    FlightCore.aicSensor = nil
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

    -- 備援：CCPE AIC / INS 姿態 (getAngles: {pitch, roll, yaw})
    local ss = getCCPESensorSystem()
    if ss and ss.isOnBody then
        local okOn, onB = pcall(function() return ss.isOnBody() end)
        if okOn and onB and ss.getAngles then
            local okAng, ang = pcall(ss.getAngles)
            if okAng and type(ang) == "table" then
                FlightCore.gimbalAvailable = true
                return ang.pitch or 0, ang.roll or 0
            end
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
        if not FlightCore.altiSensor then
            FlightCore.altiSensor = peripheral.find("altitude_sensor")
        end
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
        FlightCore.altiSensor = peripheral.find("altitude_sensor")
    end
    if not FlightCore.altiSensor then
        FlightCore.state.statusMsg = "ERR: No Altitude Sensor!"
        return
    end
    local curAlt = FlightCore.altiSensor.getHeight() or 0
    FlightCore.state.mode = "CALIBRATING"
    FlightCore.state.calibPhase = "GROUND_SEARCH"
    FlightCore.state.calibStartAlt = curAlt
    FlightCore.state.calibThrottle = 0.5
    FlightCore.state.baseThrottle = 0.5
    FlightCore.state.rampRate = 0.02
    FlightCore.state.stableTimer = 0
    FlightCore.state.statusMsg = "Calibrating: Searching Liftoff..."
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

    local currentAlt = 0
    local rawVspeed = nil
    if FlightCore.altiSensor then
        local ok, h = pcall(function() return FlightCore.altiSensor.getHeight() end)
        if ok and type(h) == "number" then currentAlt = h end
        local okV, vs = pcall(function() return FlightCore.altiSensor.getVerticalSpeed and FlightCore.altiSensor.getVerticalSpeed() end)
        if okV and type(vs) == "number" then rawVspeed = vs end
    end

    -- 備援：CCPE 靜壓孔 (Static Port) / 物理體高度與垂直速度
    local ss = getCCPESensorSystem()
    if ss and ss.isOnBody then
        local okOn, onB = pcall(function() return ss.isOnBody() end)
        if okOn and onB then
            if currentAlt == 0 and ss.getAltitude then
                local okAlt, a = pcall(ss.getAltitude)
                if okAlt and type(a) == "number" then currentAlt = a end
            end
            if rawVspeed == nil and ss.getVelocity then
                local okVel, vel = pcall(ss.getVelocity)
                if okVel and type(vel) == "table" and type(vel.y) == "number" then
                    rawVspeed = vel.y
                end
            end
            if currentAlt == 0 and ss.getBodyPosition then
                local okPos, pos = pcall(ss.getBodyPosition)
                if okPos and type(pos) == "table" and type(pos.y) == "number" then
                    currentAlt = pos.y
                end
            end
        end
    end

    -- 垂直速度計算與 EMA 低通平滑濾波
    local nowEpoch = os.epoch("utc") / 1000
    FlightCore.lastAlt = FlightCore.lastAlt or currentAlt
    FlightCore.lastAltTime = FlightCore.lastAltTime or nowEpoch
    local altDt = nowEpoch - FlightCore.lastAltTime
    if altDt <= 0 or altDt > 0.5 then altDt = 0.05 end

    local instantVspeed = (currentAlt - FlightCore.lastAlt) / altDt
    FlightCore.lastAlt = currentAlt
    FlightCore.lastAltTime = nowEpoch

    if rawVspeed ~= nil and type(rawVspeed) == "number" then
        FlightCore.filteredVspeed = rawVspeed
    else
        FlightCore.filteredVspeed = (FlightCore.filteredVspeed or 0) * 0.65 + instantVspeed * 0.35
    end
    local currentVspeed = FlightCore.filteredVspeed
    local currPitch, currRoll = FlightCore.getGimbalData()

    -- ============================================================
    -- [A] 高度維持與垂直升力控制核心 (基於 v3.8.4 經典架構)
    -- ============================================================
    if FlightCore.state.mode == "CALIBRATING" then
        if FlightCore.state.calibPhase == "GROUND_SEARCH" then
            FlightCore.state.calibThrottle = FlightCore.state.calibThrottle + FlightCore.state.rampRate
            FlightCore.state.baseThrottle = FlightCore.state.calibThrottle

            local altDelta = currentAlt - FlightCore.state.calibStartAlt
            if (altDelta >= 0.35) or (altDelta >= 0.15 and currentVspeed >= 0.12) then
                local hoverBase = math.max(1.0, FlightCore.state.calibThrottle)
                FlightCore.state.baseThrottle = hoverBase
                FlightCore.state.targetAlt = math.ceil(currentAlt)
                FlightCore.state.virtualAlt = currentAlt
                FlightCore.state.mode = "HOLD_ALT"
                FlightCore.state.statusMsg = string.format("Hover Locked: %.2f (Alt: %.1fm)", hoverBase, currentAlt)
                FlightCore.altPID:reset()
            else
                FlightCore.state.statusMsg = string.format("Ramping Lift: %.2f (Alt: %.1f)", FlightCore.state.baseThrottle, currentAlt)
            end

            FlightCore.virtualOutputs.FL = FlightCore.state.baseThrottle
            FlightCore.virtualOutputs.FR = FlightCore.state.baseThrottle
            FlightCore.virtualOutputs.BL = FlightCore.state.baseThrottle
            FlightCore.virtualOutputs.BR = FlightCore.state.baseThrottle
        end

    elseif FlightCore.state.mode == "HOLD_ALT" then
        local altDiff = FlightCore.state.targetAlt - FlightCore.state.virtualAlt
        local step = math.max(-2.5 * 0.05, math.min(2.5 * 0.05, altDiff))
        FlightCore.state.virtualAlt = FlightCore.state.virtualAlt + step

        local altError = FlightCore.state.virtualAlt - currentAlt

        -- 自動配平 (Auto-Trim): 當落後目標高度且未建立有效爬升速度時，自動平滑遞增基準油門
        if altError > 0.8 and currentVspeed < 0.20 then
            local trimRate = 0.025 -- 逐步推升基準油門 (每秒約 +0.5)
            FlightCore.state.baseThrottle = math.min(15.0, FlightCore.state.baseThrottle + trimRate)
        elseif altError < -0.8 and currentVspeed > -0.20 then
            local trimRate = 0.010
            FlightCore.state.baseThrottle = math.max(0.1, FlightCore.state.baseThrottle - trimRate)
        end

        local pidAdj = FlightCore.altPID:update(altError)

        -- 爬升推力權限 (Climb Boost Authority): 當目標高度顯著高於當前高度時，給予充足爬升力
        local climbBoost = 0.0
        if altError > 1.5 then
            climbBoost = math.min(5.0, (altError - 1.5) * 0.35)
        end

        local pitchCorr = -currPitch * 0.04
        local rollCorr = currRoll * 0.04

        -- 安全防墜保護底線: 最低輸出絕不低於 baseThrottle - 1.5 (杜絕空中斷電墜落)
        local minFloor = math.max(0.0, FlightCore.state.baseThrottle - 1.5)
        local baseThrust = FlightCore.state.baseThrottle + pidAdj + climbBoost

        FlightCore.virtualOutputs.FL = math.max(minFloor, math.min(15.0, baseThrust + pitchCorr - rollCorr))
        FlightCore.virtualOutputs.FR = math.max(minFloor, math.min(15.0, baseThrust + pitchCorr + rollCorr))
        FlightCore.virtualOutputs.BL = math.max(minFloor, math.min(15.0, baseThrust - pitchCorr - rollCorr))
        FlightCore.virtualOutputs.BR = math.max(minFloor, math.min(15.0, baseThrust - pitchCorr + rollCorr))

        local diff = FlightCore.state.targetAlt - currentAlt
        if diff > 1.0 then
            FlightCore.state.statusMsg = string.format("Climbing: %.1fm -> %.1fm", currentAlt, FlightCore.state.targetAlt)
        elseif diff < -1.0 then
            FlightCore.state.statusMsg = string.format("Descending: %.1fm -> %.1fm", currentAlt, FlightCore.state.targetAlt)
        else
            if FlightCore.nav.wpActive and FlightCore.nav.targetX and FlightCore.nav.targetZ then
                if FlightCore.nav.x and FlightCore.nav.z then
                    local dx = FlightCore.nav.targetX - FlightCore.nav.x
                    local dz = FlightCore.nav.targetZ - FlightCore.nav.z
                    local d = math.sqrt(dx*dx + dz*dz)
                    FlightCore.state.statusMsg = string.format("NAV -> (%d,%d) D:%.0fm HDG:%03d*", math.floor(FlightCore.nav.targetX), math.floor(FlightCore.nav.targetZ), d, math.floor(FlightCore.nav.targetHeading))
                else
                    FlightCore.state.statusMsg = string.format("NAV -> (%d,%d) [Searching GPS/INS...]", math.floor(FlightCore.nav.targetX), math.floor(FlightCore.nav.targetZ))
                end
            elseif FlightCore.nav.headingHold then
                FlightCore.state.statusMsg = string.format("Hold: %.1fm | HDG: %03d* (Tgt: %03d*)", currentAlt, math.floor(FlightCore.nav.yaw or 0), FlightCore.nav.targetHeading)
            else
                FlightCore.state.statusMsg = string.format("Holding Alt: %.1fm (Base: %.2f)", currentAlt, FlightCore.state.baseThrottle)
            end
        end
    end

    -- ============================================================
    -- [B] 獨立水平導航與轉向/推進控制 (Decoupled Navigation & Steering)
    -- ============================================================
    -- 1. 航點自動巡航自駕儀 (Waypoint Navigation - 偏差半徑 20 格容許)
    if FlightCore.nav.wpActive and FlightCore.nav.targetX and FlightCore.nav.targetZ then
        if FlightCore.nav.x and FlightCore.nav.z then
            local dx = FlightCore.nav.targetX - FlightCore.nav.x
            local dz = FlightCore.nav.targetZ - FlightCore.nav.z
            local dist = math.sqrt(dx * dx + dz * dz)
            local acceptRadius = FlightCore.nav.arrivalRadius or 20.0

            if dist <= acceptRadius then
                -- 進入目的地容許半徑 (20 格) -> 到達目的地，關閉前進推力
                FlightCore.nav.wpActive = false
                FlightCore.state.fwdThrottle = 0.0
                FlightCore.state.statusMsg = string.format("ARRIVED DEST! (Dist: %.1fm <= %dm)", dist, math.floor(acceptRadius))
            else
                -- 計算指向目標航點方位角 (0=北, 90=東, 180=南, 270=西)
                local targetHeading = (math.deg(math.atan2(dx, -dz)) + 360) % 360
                FlightCore.nav.targetHeading = targetHeading
                FlightCore.nav.headingHold = true

                local yawDiff = ((targetHeading - (FlightCore.nav.yaw or 0) + 180) % 360) - 180
                if math.abs(yawDiff) < 35 then
                    local cruisePower = math.min(15.0, math.max(4.0, dist * 0.12))
                    FlightCore.state.fwdThrottle = cruisePower
                else
                    FlightCore.state.fwdThrottle = 1.5
                end
            end
        end
    end

    -- 2. 獨立轉向推力計算 (右推讓船面向右，左推讓船面向左，完全不干擾垂直升力 FL/FR/BL/BR)
    local autoTurnRight = 0.0
    local autoTurnLeft = 0.0

    if FlightCore.nav.headingHold and FlightCore.nav.yaw then
        local yawDiff = ((FlightCore.nav.targetHeading - FlightCore.nav.yaw + 180) % 360) - 180
        if yawDiff > 1.2 then
            -- 目標在右側，啟動右推順時針轉向
            autoTurnRight = math.min(15.0, math.max(1.0, yawDiff * 0.20 + (yawDiff > 8 and 2.5 or 0.5)))
            autoTurnLeft = 0.0
        elseif yawDiff < -1.2 then
            -- 目標在左側，啟動左推逆時針轉向
            autoTurnLeft = math.min(15.0, math.max(1.0, -yawDiff * 0.20 + (-yawDiff > 8 and 2.5 or 0.5)))
            autoTurnRight = 0.0
        end
    end

    -- 3. 手動轉向推力與自駕轉向融合
    local manTurn = FlightCore.state.turnThrottle or FlightCore.state.strafeThrottle or 0.0
    local manTurnRight = math.max(0, manTurn)
    local manTurnLeft = math.max(0, -manTurn)

    local targetRIGHT = math.max(manTurnRight, autoTurnRight)
    local targetLEFT = math.max(manTurnLeft, autoTurnLeft)
    local targetFWD = math.max(0, FlightCore.state.fwdThrottle or 0)
    local targetBWD = math.max(0, -(FlightCore.state.fwdThrottle or 0))

    -- 4. 平滑過渡限制 (Slew Rate Limiting)
    local function approach(current, target, maxStep)
        if current < target then return math.min(target, current + maxStep)
        else return math.max(target, current - maxStep) end
    end

    local maxTransSlew = 0.5
    FlightCore.virtualOutputs.FWD = approach(FlightCore.virtualOutputs.FWD or 0, targetFWD, maxTransSlew)
    FlightCore.virtualOutputs.BWD = approach(FlightCore.virtualOutputs.BWD or 0, targetBWD, maxTransSlew)
    FlightCore.virtualOutputs.LEFT = approach(FlightCore.virtualOutputs.LEFT or 0, targetLEFT, maxTransSlew)
    FlightCore.virtualOutputs.RIGHT = approach(FlightCore.virtualOutputs.RIGHT or 0, targetRIGHT, maxTransSlew)

    -- ============================================================
    -- [C] 高頻 Sigma-Delta 誤差擴散 PWM 調變與底層輸出
    -- ============================================================
    local function calcDitheredPWM(role, val)
        val = math.max(0, math.min(15, val or 0))
        local intPart = math.floor(val)
        local fracPart = val - intPart

        local acc = (FlightCore.pwmAccumulator[role] or 0.0) + fracPart
        if acc >= 0.5 then
            FlightCore.pwmAccumulator[role] = acc - 1.0
            return math.min(15, intPart + 1)
        else
            FlightCore.pwmAccumulator[role] = acc
            return intPart
        end
    end

    FlightCore.engineOutputs.FL = calcDitheredPWM("FL", FlightCore.virtualOutputs.FL)
    FlightCore.engineOutputs.FR = calcDitheredPWM("FR", FlightCore.virtualOutputs.FR)
    FlightCore.engineOutputs.BL = calcDitheredPWM("BL", FlightCore.virtualOutputs.BL)
    FlightCore.engineOutputs.BR = calcDitheredPWM("BR", FlightCore.virtualOutputs.BR)
    FlightCore.engineOutputs.FWD = calcDitheredPWM("FWD", FlightCore.virtualOutputs.FWD)
    FlightCore.engineOutputs.BWD = calcDitheredPWM("BWD", FlightCore.virtualOutputs.BWD)
    FlightCore.engineOutputs.LEFT = calcDitheredPWM("LEFT", FlightCore.virtualOutputs.LEFT)
    FlightCore.engineOutputs.RIGHT = calcDitheredPWM("RIGHT", FlightCore.virtualOutputs.RIGHT)

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

-- ========================================================
-- PART 2: FCC 遙測廣播與駕駛艙指令通訊協議 (FCC TELEMETRY & COMMANDS)
-- ========================================================
local function handleFccCommand(side, ch, replyCh, msg, dist)
    if ch == 103 and type(msg) == "table" and msg.cmd then
        local cmd = msg.cmd
        if cmd == "HOLD_ALT" then FlightCore.holdAltitude()
        elseif cmd == "CALIB" then FlightCore.startCalibration()
        elseif cmd == "STOP" then FlightCore.stopEngines()
        elseif cmd == "LOCK_ALT" then FlightCore.lockCurrentAlt()
        elseif cmd == "ADJ_ALT" then FlightCore.adjustTargetAlt(msg.delta or 0)
        elseif cmd == "SET_ALT" then FlightCore.setTargetAlt(msg.alt or 0)
        elseif cmd == "ADJ_BASE" then FlightCore.adjustBaseThrottle(msg.delta or 0)
        elseif cmd == "ADJ_FWD" then FlightCore.adjustForwardThrottle(msg.delta or 0)
        elseif cmd == "SET_FWD" then FlightCore.setForwardThrottle(msg.val or 0)
        elseif cmd == "ADJ_TURN" then FlightCore.adjustTurnThrottle(msg.delta or 0)
        elseif cmd == "SET_TURN" then FlightCore.setTurnThrottle(msg.val or 0)
        elseif cmd == "STOP_HORIZ" then FlightCore.stopHorizontalThrust()
        elseif cmd == "SET_HEADING" then FlightCore.setTargetHeading(msg.hdg or 0)
        elseif cmd == "ADJ_HEADING" then FlightCore.adjustTargetHeading(msg.delta or 0)
        elseif cmd == "TOGGLE_HEADING" then FlightCore.toggleHeadingHold()
        elseif cmd == "SYNC_HEADING" then FlightCore.syncHeading()
        elseif cmd == "SET_WAYPOINT" then FlightCore.setWaypoint(msg.x, msg.z, msg.radius)
        elseif cmd == "CANCEL_WAYPOINT" then FlightCore.cancelWaypoint()
        elseif cmd == "RESCAN_HW" then FlightCore.scanQuadTurtles()
        end
    end
end

local function broadcastTelemetry()
    local currentAlt = FlightCore.altiSensor and FlightCore.altiSensor.getHeight() or 0
    local vspeed = FlightCore.altiSensor and FlightCore.altiSensor.getVerticalSpeed and FlightCore.altiSensor.getVerticalSpeed() or 0
    local currPitch, currRoll = FlightCore.getGimbalData()

    local teleMsg = {
        type = "FCC_TELEMETRY",
        ver = VERSION,
        mode = FlightCore.state.mode,
        targetAlt = FlightCore.state.targetAlt,
        virtualAlt = FlightCore.state.virtualAlt,
        baseThrottle = FlightCore.state.baseThrottle,
        statusMsg = FlightCore.state.statusMsg,
        stableTimer = FlightCore.state.stableTimer,
        calibPhase = FlightCore.state.calibPhase,
        fwdThrottle = FlightCore.state.fwdThrottle,
        turnThrottle = FlightCore.state.turnThrottle,
        strafeThrottle = FlightCore.state.strafeThrottle,
        vspeed = vspeed,
        currentAlt = currentAlt,
        pitch = currPitch,
        roll = currRoll,
        nav = {
            x = FlightCore.nav.x,
            y = FlightCore.nav.y,
            z = FlightCore.nav.z,
            yaw = FlightCore.nav.yaw,
            pitch = FlightCore.nav.pitch,
            roll = FlightCore.nav.roll,
            speed = FlightCore.nav.speed,
            source = FlightCore.nav.source,
            targetHeading = FlightCore.nav.targetHeading,
            headingHold = FlightCore.nav.headingHold,
            targetX = FlightCore.nav.targetX,
            targetZ = FlightCore.nav.targetZ,
            wpActive = FlightCore.nav.wpActive,
            arrivalRadius = FlightCore.nav.arrivalRadius
        },
        outputs = FlightCore.virtualOutputs,
        engineOutputs = FlightCore.engineOutputs,
        turtles = FlightCore.turtles,
        quadHealth = {
            FL = FlightCore.getQuadHealth("FL"),
            FR = FlightCore.getQuadHealth("FR"),
            BL = FlightCore.getQuadHealth("BL"),
            BR = FlightCore.getQuadHealth("BR"),
            FWD = FlightCore.getQuadHealth("FWD"),
            BWD = FlightCore.getQuadHealth("BWD"),
            LEFT = FlightCore.getQuadHealth("LEFT"),
            RIGHT = FlightCore.getQuadHealth("RIGHT")
        },
        hw = (FlightCore.getHardwareInventory and FlightCore.getHardwareInventory()) or nil,
        gimbalAvailable = FlightCore.gimbalAvailable,
        timestamp = os.epoch("utc")
    }

    for _, m in ipairs(FlightCore.modems) do
        pcall(function() m.transmit(102, 103, teleMsg) end)
    end
end

-- ========================================================
-- PART 3: FCC 主事件循環與終端監控 (FCC ORCHESTRATOR)
-- ========================================================
FlightCore.scanQuadTurtles()
for _, m in ipairs(FlightCore.modems) do
    pcall(function() m.open(101) end)
    pcall(function() m.open(103) end)
end

local teleTick = 0
local function flightLoop()
    while true do
        FlightCore.updateFlightLogic()
        teleTick = teleTick + 1
        if teleTick % 2 == 0 then
            broadcastTelemetry()
        end
        sleep(0.05)
    end
end

local function eventLoop()
    while true do
        local eventData = {os.pullEventRaw()}
        local event = eventData[1]
        if event == "modem_message" then
            local side, ch, replyCh, msg, dist = eventData[2], eventData[3], eventData[4], eventData[5], eventData[6]
            if ch == 101 then
                FlightCore.handleModemMessage(side, ch, replyCh, msg, dist)
            elseif ch == 103 then
                handleFccCommand(side, ch, replyCh, msg, dist)
            end
        elseif event == "peripheral" or event == "peripheral_detach" then
            pcall(function() FlightCore.scanQuadTurtles() end)
            for _, m in ipairs(FlightCore.modems) do
                pcall(function() m.open(101) end)
                pcall(function() m.open(103) end)
            end
        elseif event == "terminate" then
            FlightCore.stopEngines()
            break
        end
    end
end

local function uiLoop()
    while true do
        term.clear()
        term.setCursorPos(1, 1)
        local currentAlt = FlightCore.altiSensor and FlightCore.altiSensor.getHeight() or 0
        local vspeed = FlightCore.altiSensor and FlightCore.altiSensor.getVerticalSpeed and FlightCore.altiSensor.getVerticalSpeed() or 0
        local currPitch, currRoll = FlightCore.getGimbalData()
        local qFL = FlightCore.getQuadHealth("FL")
        local qFR = FlightCore.getQuadHealth("FR")
        local qBL = FlightCore.getQuadHealth("BL")
        local qBR = FlightCore.getQuadHealth("BR")
        local qFWD = FlightCore.getQuadHealth("FWD")

        print("========================================")
        print("  VTOL Flight Control Computer (FCC)   ")
        print(string.format("  Version: %s [DISTRIBUTED MODE]  ", VERSION))
        print("========================================")
        print(string.format(" Status : %s", FlightCore.state.statusMsg or "Ready"))
        print(string.format(" Mode   : %-10s  Alt: %.1fm (Tgt: %.1fm)", FlightCore.state.mode, currentAlt, FlightCore.state.targetAlt))
        print(string.format(" Hover  : Base: %.2f  V.S: %+.2fm/s", FlightCore.state.baseThrottle, vspeed))
        print(string.format(" Gimbal : Pitch: %+3.1f*  Roll: %+3.1f*", currPitch, currRoll))
        print(string.format(" Heading: %03d* (Hold: %s, Tgt: %03d*)", math.floor(FlightCore.nav.yaw or 0), FlightCore.nav.headingHold and "ON" or "OFF", math.floor(FlightCore.nav.targetHeading or 0)))
        if FlightCore.nav.x then
            print(string.format(" Nav Pos: X:%.0f Y:%.0f Z:%.0f (Spd:%.1f)", FlightCore.nav.x, FlightCore.nav.y or currentAlt, FlightCore.nav.z, FlightCore.nav.speed or 0))
        else
            print(" Nav Pos: Searching INS / GPS / Table...")
        end
        print(string.format(" Lift   : FL:%.1f FR:%.1f BL:%.1f BR:%.1f", FlightCore.virtualOutputs.FL or 0, FlightCore.virtualOutputs.FR or 0, FlightCore.virtualOutputs.BL or 0, FlightCore.virtualOutputs.BR or 0))
        print(string.format(" Cruis  : FWD:%.1f BWD:%.1f L:%.1f R:%.1f", FlightCore.virtualOutputs.FWD or 0, FlightCore.virtualOutputs.BWD or 0, FlightCore.virtualOutputs.LEFT or 0, FlightCore.virtualOutputs.RIGHT or 0))
        print(string.format(" Turtles: FL(%d) FR(%d) BL(%d) BR(%d) FWD(%d)", qFL.online, qFR.online, qBL.online, qBR.online, qFWD.online))
        print(" Net Ch : 100(Turtles) 102(Tele) 103(Cmd)")
        print("========================================")
        print(" FCC Core Running (0% CPU, No Render)")
        print(" Press Ctrl+T to Terminate.")
        sleep(0.5)
    end
end

parallel.waitForAny(flightLoop, eventLoop, uiLoop)
