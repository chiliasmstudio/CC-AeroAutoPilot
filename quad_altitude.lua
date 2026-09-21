--[[
    Create: Avionics & CC: Tweaked
    Quad-Engine Wired Flight Controller (四軸有線原生螢幕飛控大腦)
    Version: v3.5.0
    
    智慧校準與防暴衝姿態自穩邏輯:
    - 類比紅石純淨輸出：修正 setOutput 導致訊號被強制頂到 15 滿載暴衝的底層問題。
    - 引擎雙重推力顯示：同時顯示實體訊號 (例如 2/15) 與目前即時虛擬推力 (例如 2.3/15)。
    - 智慧變速探索起飛推力：起步極慢 -> 持續無動作則穩健加速加力 (不卡死在低檔) -> 偵測到起飛立刻放緩加力並平滑控速爬升。
    - 虛擬軌跡巡航爬升：以 0.30 m/s 柔和定速引導爬升至 +10 格懸浮，消除階躍誤差暴衝。
    - 姿態即時反向加力配平：傾斜時立刻對低側加力、高側減力，維持水平。
    - 穩定收斂判定：高度誤差 < 0.5m、垂直速度 < 0.12m/s、姿態傾斜 < 1.0° 連續 5 秒自動鎖定最佳懸停基準！
--]]

local VERSION = "v3.5.0"
print(string.format("[AVIONICS] Loading Quad-Engine Flight Controller %s...", VERSION))

-- ========================================================
-- 1. 內建 PID 控制器 (Embedded PID)
-- ========================================================
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

-- ========================================================
-- 2. 螢幕與調色盤初始化
-- ========================================================
local mon = peripheral.find("monitor")
local display = mon or term.current()

if mon then
    mon.setTextScale(0.5)
end

if display.setPaletteColor then
    display.setPaletteColor(colors.black, 0x111622)
    display.setPaletteColor(colors.gray, 0x1c2438)
    display.setPaletteColor(colors.lightGray, 0x5a6988)
    display.setPaletteColor(colors.blue, 0x2266cc)
    display.setPaletteColor(colors.cyan, 0x00d2ff)
    display.setPaletteColor(colors.lime, 0x2ed573)
    display.setPaletteColor(colors.green, 0x1e824c)
    display.setPaletteColor(colors.yellow, 0xffa502)
    display.setPaletteColor(colors.red, 0xff4757)
    display.setPaletteColor(colors.white, 0xf1f2f6)
end

local altiSensor   = peripheral.find("altitude_sensor")
local gimbalSensor = peripheral.find("gimbal_sensor")
local gimbalAvailable = false

local function updateSensors()
    if not altiSensor then
        altiSensor = peripheral.find("altitude_sensor")
    end
    gimbalSensor = peripheral.find("gimbal_sensor")
end

local function getGimbalData()
    if not gimbalSensor then
        updateSensors()
    end
    if gimbalSensor then
        local ok, angles = pcall(function() return gimbalSensor.getAngles() end)
        if ok and type(angles) == "table" then
            gimbalAvailable = true
            return angles[1] or 0, angles[2] or 0
        end
    end
    gimbalAvailable = false
    return 0, 0
end

local function safeBlit(x, y, text, fgChar, bgChar)
    display.setCursorPos(x, y)
    local len = #text
    local fg = (type(fgChar) == "string" and #fgChar == len) and fgChar or string.rep(tostring(fgChar or "0"), len)
    local bg = (type(bgChar) == "string" and #bgChar == len) and bgChar or string.rep(tostring(bgChar or "f"), len)
    display.blit(text, fg, bg)
end

-- ========================================================
-- 3. 四角烏龜引擎節點掃描與辨識 (FL, FR, BL, BR)
-- ========================================================
local engines = { FL = nil, FR = nil, BL = nil, BR = nil }
local engineOutputs  = { FL = 0, FR = 0, BL = 0, BR = 0 }
local virtualOutputs = { FL = 0.0, FR = 0.0, BL = 0.0, BR = 0.0 }

local function matchPrefix(label, prefix)
    local u = string.upper(label)
    return u == prefix or u:sub(1, #prefix + 1) == (prefix .. "_") or u:sub(1, #prefix + 1) == (prefix .. "-") or u:sub(1, #prefix + 1) == (prefix .. " ")
end

local function scanQuadTurtles()
    updateSensors()
    engines.FL = nil
    engines.FR = nil
    engines.BL = nil
    engines.BR = nil

    local unmapped = {}
    local pNames = peripheral.getNames()

    for _, name in ipairs(pNames) do
        local pType = peripheral.getType(name)
        if pType == "turtle" or pType == "computer" or pType == "redstone_relay" or pType == "modem" then
            local p = peripheral.wrap(name)
            local label = (p.getLabel and p.getLabel()) or name

            if matchPrefix(label, "FL") then
                engines.FL = { name = name, p = p, label = label }
            elseif matchPrefix(label, "FR") then
                engines.FR = { name = name, p = p, label = label }
            elseif matchPrefix(label, "BL") then
                engines.BL = { name = name, p = p, label = label }
            elseif matchPrefix(label, "BR") then
                engines.BR = { name = name, p = p, label = label }
            else
                table.insert(unmapped, { name = name, p = p, label = label })
            end
        end
    end

    local slots = {"FL", "FR", "BL", "BR"}
    local uIdx = 1
    for _, slot in ipairs(slots) do
        if not engines[slot] and unmapped[uIdx] then
            engines[slot] = unmapped[uIdx]
            uIdx = uIdx + 1
        end
    end
end

scanQuadTurtles()

-- ========================================================
-- 4. 紅石輸出（純類比輸出，排除正面，其餘 5 面）與四軸混控
-- ========================================================
local outputSides = {"bottom", "top", "left", "right", "back"}

local masterModem = peripheral.find("modem")
if masterModem then
    pcall(function() masterModem.open(101) end)
end

local pwmTick = 0

local function outputToEngine(node, signal)
    if not node or not node.p then return end
    signal = math.max(0, math.min(15, math.floor(signal + 0.5)))
    
    for _, side in ipairs(outputSides) do
        pcall(function()
            if node.p.setAnalogOutput then
                node.p.setAnalogOutput(side, signal)
            elseif node.p.setOutput then
                node.p.setOutput(side, signal > 0)
            end
        end)
    end
end

local function applyQuadThrust(baseThrust, deltaAlt, deltaPitch, deltaRoll)
    -- 20 ticks (1.0s 週期) 超高解析度 PWM 佔空比計數器 (0 ~ 19)
    pwmTick = (pwmTick + 1) % 20

    -- 四軸混控公式：
    local rawFL = baseThrust + deltaAlt - deltaPitch + deltaRoll
    local rawFR = baseThrust + deltaAlt - deltaPitch - deltaRoll
    local rawBL = baseThrust + deltaAlt + deltaPitch + deltaRoll
    local rawBR = baseThrust + deltaAlt + deltaPitch - deltaRoll

    virtualOutputs.FL = math.max(0, math.min(15, rawFL))
    virtualOutputs.FR = math.max(0, math.min(15, rawFR))
    virtualOutputs.BL = math.max(0, math.min(15, rawBL))
    virtualOutputs.BR = math.max(0, math.min(15, rawBR))

    local function resolvePwmOutput(val)
        if val <= 0.001 then return 0 end
        if val < 1.0 then
            local activeThreshold = math.floor(val * 20 + 0.5)
            if pwmTick < activeThreshold then
                return 1
            else
                return 0
            end
        else
            return math.max(0, math.min(15, math.floor(val + 0.5)))
        end
    end

    engineOutputs.FL = resolvePwmOutput(rawFL)
    engineOutputs.FR = resolvePwmOutput(rawFR)
    engineOutputs.BL = resolvePwmOutput(rawBL)
    engineOutputs.BR = resolvePwmOutput(rawBR)

    -- 1. 雙重輸出 A：透過有線網路向 Turtle/Relay 周邊呼叫
    outputToEngine(engines.FL, engineOutputs.FL)
    outputToEngine(engines.FR, engineOutputs.FR)
    outputToEngine(engines.BL, engineOutputs.BL)
    outputToEngine(engines.BR, engineOutputs.BR)

    -- 2. 雙重輸出 B：透過 Modem 廣播 (頻道 100) 給運行 turtle_startup 的烏龜
    if not masterModem then
        masterModem = peripheral.find("modem")
    end
    if masterModem then
        pcall(function()
            masterModem.transmit(100, 101, {
                FL = engineOutputs.FL,
                FR = engineOutputs.FR,
                BL = engineOutputs.BL,
                BR = engineOutputs.BR
            })
        end)
    end
end

-- ========================================================
-- 5. 飛控狀態與 PID 控制器
-- ========================================================
local state = {
    mode = "IDLE",            -- "IDLE", "HOLD_ALT", "CALIBRATING"
    targetAlt = 100.0,        -- 全局目標高度
    virtualAlt = 100.0,       -- 虛擬平滑軌跡高度 (防階躍暴衝)
    baseThrottle = 1.0,       -- 支援浮點推力 (0.01 ~ 15.0)，< 1 自動進入 PWM 脈衝模式
    autoLevel = true,         -- 自動水平姿態開關
    statusMsg = "System Ready",
    stableTimer = 0,          -- 穩定計時器 (秒)
    calibStartAlt = 0,        -- 校正啟動高度
    calibTargetAlt = 0,       -- 校正懸停目標 (起飛高度 + 10m)
    calibPhase = "GROUND_SEARCH", -- "GROUND_SEARCH", "SMOOTH_CLIMB", "STABILIZE"
    calibThrottle = 0.0,      -- 校正微調油門 (從 0.0 脈衝起步)
    rampRate = 0.0001         -- 適應性加力速率 (起步極慢)
}

-- PID 控制器：高度修正限幅在 [-1.5, +1.5]
local altPID   = PID.new(0.5, 0.02, 0.6, -1.5, 1.5)
local pitchPID = PID.new(0.20, 0.01, 0.10, -6, 6)
local rollPID  = PID.new(0.20, 0.01, 0.10, -6, 6)

local buttons = {}

local function addButton(x, y, w, h, text, bgBlit, fgBlit, action)
    table.insert(buttons, {
        x = x, y = y, w = w, h = h,
        text = text, bg = bgBlit, fg = fgBlit,
        action = action
    })
end

-- ========================================================
-- 6. 螢幕繪製函式
-- ========================================================
local function drawQuadUI()
    local w, h = display.getSize()
    display.setBackgroundColor(colors.black)
    display.clear()

    -- 1. 頂部標題列 (顯示版本號)
    local titleText = string.format("  QUAD-ENGINE AVIONICS MASTER %s  ", VERSION)
    local padL = math.floor((w - #titleText) / 2)
    local padR = w - #titleText - padL
    local header = string.rep(" ", padL) .. titleText .. string.rep(" ", padR)
    safeBlit(1, 1, header, "0", "b")

    local currAlt = altiSensor and altiSensor.getHeight() or 0
    local currVspeed = altiSensor and altiSensor.getVerticalSpeed() or 0
    local currPitch, currRoll = getGimbalData()

    -- 2. 左側高度與姿態卡片
    local cardW = math.floor(w / 2) - 2
    for y = 3, 9 do
        safeBlit(2, y, string.rep(" ", cardW), "0", "8")
    end
    safeBlit(3, 3, "ALTITUDE & ATTITUDE", "9", "8")
    
    local altStr = string.format("ALT:%5.1fm (TGT:%4.0f)", currAlt, state.targetAlt)
    if #altStr > cardW - 2 then altStr = string.format("A:%5.1f T:%4.0f", currAlt, state.targetAlt) end
    safeBlit(3, 4, altStr, "0", "8")
    
    local vspeedStr = string.format("V.SPD: %+5.2f m/s", currVspeed)
    safeBlit(3, 5, vspeedStr, "4", "8")

    -- Gimbal Sensor 狀態防呆指示
    if gimbalAvailable then
        local attStr = string.format("P:%+4.1f* R:%+4.1f*", currPitch, currRoll)
        safeBlit(3, 6, attStr, "3", "8")
        safeBlit(3, 7, "GIMBAL: ACTIVE [OK]", "5", "8")
    else
        safeBlit(3, 6, "P: ---   R: ---", "7", "8")
        safeBlit(3, 7, "GIMBAL: NO SENSOR!", "e", "8")
    end
    
    local modeFg = (state.mode == "HOLD_ALT") and "5" or (state.mode == "CALIBRATING" and "4" or "e")
    local modeRow = string.format("MODE:%-5s B:%4.2f", state.mode:sub(1,5), state.baseThrottle)
    safeBlit(3, 8, modeRow, modeFg, "8")

    -- 3. 右側四軸烏龜名稱與推力卡片 (顯示 實體訊號 與 虛擬推力)
    local rightX = cardW + 4
    local rightW = w - rightX
    for y = 3, 9 do
        safeBlit(rightX, y, string.rep(" ", rightW), "0", "8")
    end
    safeBlit(rightX + 1, 3, "ENGINES (SIG / VIRT)", "9", "8")

    local function getEngineInfo(node, defaultLabel, outVal, virtVal)
        if not node then
            return string.format("%-2s : OFFLINE", defaultLabel), "e"
        end
        local lbl = node.label or node.name or defaultLabel
        if #lbl > 4 then lbl = lbl:sub(1, 4) end
        return string.format("%-2s : %2d/15 (%4.1f)", lbl, outVal, virtVal or outVal), "5"
    end

    local flStr, flCol = getEngineInfo(engines.FL, "FL", engineOutputs.FL, virtualOutputs.FL)
    safeBlit(rightX + 1, 4, flStr, flCol, "8")

    local frStr, frCol = getEngineInfo(engines.FR, "FR", engineOutputs.FR, virtualOutputs.FR)
    safeBlit(rightX + 1, 5, frStr, frCol, "8")

    local blStr, blCol = getEngineInfo(engines.BL, "BL", engineOutputs.BL, virtualOutputs.BL)
    safeBlit(rightX + 1, 6, blStr, blCol, "8")

    local brStr, brCol = getEngineInfo(engines.BR, "BR", engineOutputs.BR, virtualOutputs.BR)
    safeBlit(rightX + 1, 7, brStr, brCol, "8")

    local autoLvlStr = string.format("AUTO-LEVEL: %s", (state.autoLevel and gimbalAvailable) and "ON" or "OFF")
    local autoLvlFg = (state.autoLevel and gimbalAvailable) and "5" or "e"
    safeBlit(rightX + 1, 8, autoLvlStr, autoLvlFg, "8")

    -- 4. 狀態訊息列
    local statusRow = string.format("STATUS: %-30s", state.statusMsg):sub(1, w - 2)
    safeBlit(2, 10, statusRow, "0", "f")

    -- 5. 觸控按鈕區
    buttons = {}

    -- 第一排：目標高度調整 (+10m, +1m, -1m, -10m, SET CURR)
    local bY1 = 12
    local bW1 = math.floor((w - 6) / 5)
    addButton(2, bY1, bW1, 2, "+10m", "d", "0", function()
        state.targetAlt = state.targetAlt + 10
    end)
    addButton(3 + bW1, bY1, bW1, 2, "+1m", "5", "0", function()
        state.targetAlt = state.targetAlt + 1
    end)
    addButton(4 + bW1*2, bY1, bW1, 2, "-1m", "1", "0", function()
        state.targetAlt = math.max(0, state.targetAlt - 1)
    end)
    addButton(5 + bW1*3, bY1, bW1, 2, "-10m", "e", "0", function()
        state.targetAlt = math.max(0, state.targetAlt - 10)
    end)
    addButton(6 + bW1*4, bY1, bW1, 2, "SET CURR", "3", "0", function()
        local cur = altiSensor and altiSensor.getHeight() or 0
        state.targetAlt = math.floor(cur + 0.5)
        state.virtualAlt = cur
        state.statusMsg = string.format("Target locked to current: %.0fm", state.targetAlt)
    end)

    -- 第二排：基準油門微調、重新掃描、校正 (+10格懸浮自穩)
    local bY2 = 15
    local bW2 = math.floor((w - 5) / 4)
    addButton(2, bY2, bW2, 2, "BASE +", "3", "0", function()
        if state.baseThrottle < 1.0 then
            state.baseThrottle = math.min(15.0, math.floor((state.baseThrottle + 0.05) * 100 + 0.5) / 100)
        else
            state.baseThrottle = math.min(15.0, state.baseThrottle + 1.0)
        end
    end)
    addButton(3 + bW2, bY2, bW2, 2, "BASE -", "9", "0", function()
        if state.baseThrottle <= 1.0 then
            state.baseThrottle = math.max(0.0, math.floor((state.baseThrottle - 0.05) * 100 + 0.5) / 100)
        else
            state.baseThrottle = math.max(0.0, state.baseThrottle - 1.0)
        end
    end)
    addButton(4 + bW2*2, bY2, bW2, 2, "RE-SCAN", "b", "0", function()
        scanQuadTurtles()
        state.statusMsg = "Hardware Re-scanned!"
    end)
    addButton(5 + bW2*3, bY2, bW2, 2, "CALIBRATE", "a", "0", function()
        scanQuadTurtles()
        local cur = altiSensor and altiSensor.getHeight() or 100
        state.calibStartAlt = cur
        state.calibTargetAlt = cur + 10.0 -- 懸停在 +10格 (起飛高度 + 10m)
        state.virtualAlt = cur
        state.targetAlt = cur + 10.0
        state.calibPhase = "GROUND_SEARCH"
        state.calibThrottle = 0.0
        state.baseThrottle = 0.0
        state.rampRate = 0.0001 -- 極慢起步加力速率 (每秒僅 +0.002)
        state.mode = "CALIBRATING"
        state.stableTimer = 0
        altPID:reset()
        pitchPID:reset()
        rollPID:reset()
        if not gimbalAvailable then
            state.statusMsg = "[WARN] No Gimbal! Soft Probing Lift..."
        else
            state.statusMsg = "Calib: Soft Probing Lift..."
        end
    end)

    -- 第三排：主要飛控模式按鈕
    local bY3 = 18
    local mainBW = math.floor((w - 3) / 2)
    local holdBg = (state.mode == "HOLD_ALT") and "5" or "d"
    addButton(2, bY3, mainBW, 3, " [ HOLD ALT ] ", holdBg, "0", function()
        state.mode = "HOLD_ALT"
        local cur = altiSensor and altiSensor.getHeight() or state.targetAlt
        state.virtualAlt = cur
        altPID:reset()
        pitchPID:reset()
        rollPID:reset()
        if not gimbalAvailable then
            state.statusMsg = "[WARN] Holding Altitude (No Gimbal)"
        else
            state.statusMsg = string.format("Holding Altitude: %.0fm", state.targetAlt)
        end
    end)

    local stopBg = (state.mode == "IDLE") and "e" or "c"
    addButton(3 + mainBW, bY3, mainBW, 3, " [ STOP / IDLE ] ", stopBg, "0", function()
        state.mode = "IDLE"
        altPID:reset()
        applyQuadThrust(0, 0, 0, 0)
        state.statusMsg = "All 4 Engines Stopped"
    end)

    for _, btn in ipairs(buttons) do
        for dy = 0, btn.h - 1 do
            safeBlit(btn.x, btn.y + dy, string.rep(" ", btn.w), btn.fg, btn.bg)
        end
        local tx = btn.x + math.floor((btn.w - #btn.text) / 2)
        local ty = btn.y + math.floor(btn.h / 2)
        safeBlit(tx, ty, btn.text, btn.fg, btn.bg)
    end
end

-- ========================================================
-- 7. 飛控閉環控制迴圈 (20Hz)
-- ========================================================
local function flightControlLoop()
    local scanTicker = 0
    while true do
        scanTicker = scanTicker + 1
        if scanTicker >= 20 then
            scanTicker = 0
            scanQuadTurtles()
        end

        local currAlt = altiSensor and altiSensor.getHeight() or 0
        local currVspeed = altiSensor and altiSensor.getVerticalSpeed() or 0
        local currPitch, currRoll = getGimbalData()

        if state.mode == "HOLD_ALT" then
            if state.virtualAlt < state.targetAlt then
                state.virtualAlt = math.min(state.targetAlt, state.virtualAlt + 0.04)
            elseif state.virtualAlt > state.targetAlt then
                state.virtualAlt = math.max(state.targetAlt, state.virtualAlt - 0.04)
            end

            local altError = state.virtualAlt - currAlt
            local deltaAlt = altPID:update(altError)

            if currVspeed > 0.8 then deltaAlt = deltaAlt - 0.3 end
            if currVspeed < -0.8 then deltaAlt = deltaAlt + 0.3 end

            local deltaPitch = 0
            local deltaRoll = 0
            if state.autoLevel and gimbalAvailable then
                deltaPitch = pitchPID:update(-currPitch)
                deltaRoll  = rollPID:update(-currRoll)
            end

            applyQuadThrust(state.baseThrottle, deltaAlt, deltaPitch, deltaRoll)

        elseif state.mode == "CALIBRATING" then
            local deltaPitch = 0
            local deltaRoll = 0
            if state.autoLevel and gimbalAvailable then
                deltaPitch = pitchPID:update(-currPitch)
                deltaRoll  = rollPID:update(-currRoll)
            end

            local climbDist = currAlt - state.calibStartAlt

            if state.calibPhase == "GROUND_SEARCH" then
                -- 階段 1: 地面極慢探索起飛推力
                state.calibThrottle = math.min(15.0, state.calibThrottle + state.rampRate)
                state.baseThrottle = math.floor(state.calibThrottle * 1000 + 0.5) / 1000

                -- 若一直都沒動作（高度無明顯上升且垂直速度 < 0.08），穩健逐步加快增加推力速度！
                if climbDist < 0.10 and currVspeed < 0.08 then
                    state.rampRate = math.min(0.005, state.rampRate + 0.00002)
                end

                applyQuadThrust(state.baseThrottle, 0, deltaPitch, deltaRoll)
                state.statusMsg = string.format("Search: %4.2f/15 (R:%5.4f)", state.baseThrottle, state.rampRate * 20)

                -- 直到有動作（高度上升 >= 0.15m 或 垂直速度 >= 0.08m/s）：立刻放緩增加速度！
                if climbDist >= 0.15 or currVspeed >= 0.08 then
                    state.rampRate = 0.0001 -- 立刻放緩加力速率
                    state.virtualAlt = currAlt
                    state.calibPhase = "SMOOTH_CLIMB"
                    altPID:reset()
                    state.statusMsg = string.format("Lifted! Smooth Climb to +10m (B:%4.2f)", state.baseThrottle)
                end

            elseif state.calibPhase == "SMOOTH_CLIMB" then
                -- 階段 2: 平滑定速爬升 (目標速度 0.30 m/s)
                -- 速度不足時 (V < 0.15 m/s 且尚未到達目標)，再次緩慢增加基礎推力以防卡死掉速
                if currVspeed < 0.15 and climbDist < 8.5 and state.baseThrottle < 14 then
                    state.calibThrottle = math.min(15.0, state.calibThrottle + 0.0002)
                    state.baseThrottle = math.floor(state.calibThrottle * 1000 + 0.5) / 1000
                elseif currVspeed > 0.40 and state.baseThrottle > 0.02 then
                    -- 速度過快時，主動微幅回調
                    state.calibThrottle = math.max(0.01, state.calibThrottle - 0.0005)
                    state.baseThrottle = math.floor(state.calibThrottle * 1000 + 0.5) / 1000
                end

                state.virtualAlt = math.min(state.calibTargetAlt, state.virtualAlt + 0.015)
                local altError = state.virtualAlt - currAlt
                local deltaAlt = altPID:update(altError)

                -- 速度阻尼
                if currVspeed > 0.35 then deltaAlt = deltaAlt - 0.25 end

                applyQuadThrust(state.baseThrottle, deltaAlt, deltaPitch, deltaRoll)
                state.statusMsg = string.format("Climbing: %5.1f / %5.1fm (V:%+.2f)", currAlt, state.calibTargetAlt, currVspeed)

                -- 接近 +10m 目標高度 (距目標 < 0.5m 或 climbDist >= 9.5m)
                if currAlt >= state.calibTargetAlt - 0.5 or climbDist >= 9.5 then
                    state.calibPhase = "STABILIZE"
                    state.virtualAlt = state.calibTargetAlt
                    state.stableTimer = 0
                    altPID:reset()
                    state.statusMsg = string.format("Arrived +10m! Hover Stabilizing...")
                end

            elseif state.calibPhase == "STABILIZE" then
                -- 階段 3: 保持在 (起飛高度 + 10m) 懸停，高精度自穩收斂
                local altError = state.calibTargetAlt - currAlt
                local deltaAlt = altPID:update(altError)

                if currVspeed > 0.25 then deltaAlt = deltaAlt - 0.2 end
                if currVspeed < -0.25 then deltaAlt = deltaAlt + 0.2 end

                applyQuadThrust(state.baseThrottle, deltaAlt, deltaPitch, deltaRoll)

                local altDiff = math.abs(altError)
                local vDiff = math.abs(currVspeed)
                local attDiff = math.max(math.abs(currPitch), math.abs(currRoll))

                local attOk = (not gimbalAvailable) or (attDiff < 1.0)
                if altDiff < 0.5 and vDiff < 0.12 and attOk then
                    state.stableTimer = state.stableTimer + 0.05
                    state.statusMsg = string.format("Hover Stabilizing (%.1f/5.0s)", state.stableTimer)

                    if state.stableTimer >= 5.0 then
                        state.mode = "HOLD_ALT"
                        state.targetAlt = state.calibTargetAlt
                        state.virtualAlt = state.calibTargetAlt
                        state.statusMsg = string.format("Calibrated! Hover Base: %.2f", state.baseThrottle)
                    end
                else
                    state.stableTimer = 0
                    if altError > 0.5 and state.baseThrottle < 14 and currVspeed < 0.10 then
                        state.baseThrottle = math.min(15.0, state.baseThrottle + 0.001)
                    elseif altError < -0.5 and state.baseThrottle > 0.01 and currVspeed > -0.10 then
                        state.baseThrottle = math.max(0.005, state.baseThrottle - 0.001)
                    end
                    state.statusMsg = string.format("Hover Trim: %5.1fm (B:%4.2f)", state.calibTargetAlt, state.baseThrottle)
                end
            end

        elseif state.mode == "IDLE" then
            applyQuadThrust(0, 0, 0, 0)
        end
        sleep(0.05)
    end
end

-- ========================================================
-- 8. 介面刷新與右鍵觸控監聽
-- ========================================================
local function renderLoop()
    while true do
        drawQuadUI()
        sleep(0.1)
    end
end

local function touchEventLoop()
    while true do
        local event, side, x, y = os.pullEvent()
        if event == "monitor_touch" or event == "mouse_click" then
            local clickX, clickY = x, y
            if event == "mouse_click" then clickX, clickY = side, x end

            for _, btn in ipairs(buttons) do
                if clickX >= btn.x and clickX < btn.x + btn.w and
                   clickY >= btn.y and clickY < btn.y + btn.h then
                    btn.action()
                    drawQuadUI()
                    break
                end
            end
        end
    end
end

parallel.waitForAll(flightControlLoop, renderLoop, touchEventLoop)
