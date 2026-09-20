--[[
    Create: Avionics & CC: Tweaked
    Quad-Engine Wired Flight Controller (四軸有線原生螢幕飛控大腦)
    
    智慧校準邏輯:
    - 啟動 CALIBRATE 時，系統會自主爬升/下降並尋找平衡點。
    - 目標鎖定在 Y = 200m，並啟動姿態自穩 PID (Pitch & Roll 平衡)。
    - 當「高度誤差 < 0.5m、垂直速度 < 0.15m/s、水平角度誤差 < 1.0°」持續穩定維持 5 秒，自動鎖定基準推力並宣告校準完成！
--]]

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

local function matchPrefix(label, prefix)
    local u = string.upper(label)
    return u == prefix or u:sub(1, #prefix + 1) == (prefix .. "_") or u:sub(1, #prefix + 1) == (prefix .. "-") or u:sub(1, #prefix + 1) == (prefix .. " ")
end

local function scanQuadTurtles()
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
-- 4. 紅石輸出（排除正面，其餘 5 面：bottom, top, left, right, back）與四軸混控
-- ========================================================
local engineOutputs = { FL = 0, FR = 0, BL = 0, BR = 0 }
local outputSides = {"bottom", "top", "left", "right", "back"}

local masterModem = peripheral.find("modem")
if masterModem then
    pcall(function() masterModem.open(101) end)
end

local pwmTick = 0

local function outputToEngine(node, signal)
    if not node or not node.p then return end
    signal = math.max(0, math.min(15, math.floor(signal + 0.5)))
    local digitalState = (signal > 0)
    
    for _, side in ipairs(outputSides) do
        pcall(function()
            if node.p.setAnalogOutput then node.p.setAnalogOutput(side, signal) end
            if node.p.setOutput then node.p.setOutput(side, digitalState) end
        end)
    end
end

local function applyQuadThrust(baseThrust, deltaAlt, deltaPitch, deltaRoll)
    -- 累加 PWM 週期計數器 (0 ~ 9)
    pwmTick = (pwmTick + 1) % 10

    local rawFL = baseThrust + deltaAlt - deltaPitch - deltaRoll
    local rawFR = baseThrust + deltaAlt - deltaPitch + deltaRoll
    local rawBL = baseThrust + deltaAlt + deltaPitch - deltaRoll
    local rawBR = baseThrust + deltaAlt + deltaPitch + deltaRoll

    local function resolvePwmOutput(val)
        if val <= 0 then return 0 end
        if val < 1.0 then
            -- 脈衝模式 (PWM): 0.1 ~ 0.9 轉為 10 ticks 內的佔空比
            local activeThreshold = math.floor(val * 10 + 0.5)
            if pwmTick < activeThreshold then
                return 1 -- 輸出 1 訊號
            else
                return 0 -- 輸出 0 訊號
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
    mode = "IDLE",       -- "IDLE", "HOLD_ALT", "CALIBRATING"
    targetAlt = 200.0,   -- 預設目標高度 200m
    baseThrottle = 1.0,  -- 支援浮點推力 (0.1 ~ 15.0)，< 1 自動進入 PWM 脈衝模式
    autoLevel = true,
    statusMsg = "System Ready",
    stableTimer = 0,     -- 穩定計時器 (秒)
    calibStartAlt = 0,   -- 校正啟動高度
    calibTargetAlt = 0,  -- 校正目標高度 (啟動高度 + 30m)
    calibPhase = "RAMP", -- "RAMP" (緩慢脈衝加力起飛), "STABILIZE" (保持定高自穩)
    calibThrottle = 0.05 -- 校正微調油門 (從極微小 0.05 脈衝起步)
}

local altPID   = PID.new(0.6, 0.05, 0.8, -8, 8)
local pitchPID = PID.new(0.08, 0.0, 0.04, -4, 4)
local rollPID  = PID.new(0.08, 0.0, 0.04, -4, 4)

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

    -- 1. 頂部標題列
    local titleText = "  QUAD-ENGINE AVIONICS MASTER  "
    local padL = math.floor((w - #titleText) / 2)
    local padR = w - #titleText - padL
    local header = string.rep(" ", padL) .. titleText .. string.rep(" ", padR)
    safeBlit(1, 1, header, "0", "b")

    local currAlt = altiSensor and altiSensor.getHeight() or 0
    local currVspeed = altiSensor and altiSensor.getVerticalSpeed() or 0
    local angles = (gimbalSensor and gimbalSensor.getAngles()) or {0, 0}
    local currPitch, currRoll = angles[1] or 0, angles[2] or 0

    -- 2. 左側高度與姿態卡片
    local cardW = math.floor(w / 2) - 2
    for y = 3, 9 do
        safeBlit(2, y, string.rep(" ", cardW), "0", "8")
    end
    safeBlit(3, 3, "ALTITUDE & ATTITUDE", "9", "8")
    
    local altStr = string.format("ALT: %6.1f m", currAlt)
    safeBlit(3, 5, altStr, "0", "8")
    
    local vspeedStr = string.format("V.SPD: %+5.1f m/s", currVspeed)
    safeBlit(3, 6, vspeedStr, "4", "8")

    local attStr = string.format("P:%+4.1f* R:%+4.1f*", currPitch, currRoll)
    safeBlit(3, 7, attStr, "3", "8")
    
    local modeFg = (state.mode == "HOLD_ALT") and "5" or (state.mode == "CALIBRATING" and "4" or "e")
    local modeRow = string.format("MODE:%-5s B:%4.1f", state.mode:sub(1,5), state.baseThrottle)
    safeBlit(3, 8, modeRow, modeFg, "8")

    -- 3. 右側四軸烏龜名稱與推力卡片
    local rightX = cardW + 4
    local rightW = w - rightX
    for y = 3, 9 do
        safeBlit(rightX, y, string.rep(" ", rightW), "0", "8")
    end
    safeBlit(rightX + 1, 3, "ENGINES (LABEL & SIG)", "9", "8")

    local function getEngineInfo(node, defaultLabel, outVal)
        if not node then
            return string.format("%-10s: OFFLINE", defaultLabel), "e"
        end
        local lbl = node.label or node.name or defaultLabel
        if #lbl > 10 then lbl = lbl:sub(1, 10) end
        return string.format("%-10s: %2d/15", lbl, outVal), "5"
    end

    local flStr, flCol = getEngineInfo(engines.FL, "FL", engineOutputs.FL)
    safeBlit(rightX + 1, 5, flStr, flCol, "8")

    local frStr, frCol = getEngineInfo(engines.FR, "FR", engineOutputs.FR)
    safeBlit(rightX + 1, 6, frStr, frCol, "8")

    local blStr, blCol = getEngineInfo(engines.BL, "BL", engineOutputs.BL)
    safeBlit(rightX + 1, 7, blStr, blCol, "8")

    local brStr, brCol = getEngineInfo(engines.BR, "BR", engineOutputs.BR)
    safeBlit(rightX + 1, 8, brStr, brCol, "8")

    -- 4. 狀態訊息列
    local statusRow = string.format("STATUS: %-30s", state.statusMsg):sub(1, w - 2)
    safeBlit(2, 10, statusRow, "0", "f")

    -- 5. 觸控按鈕區
    buttons = {}

    local bY1 = 12
    local bW1 = math.floor((w - 5) / 4)
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

    local bY2 = 15
    local bW2 = math.floor((w - 5) / 4)
    addButton(2, bY2, bW2, 2, "BASE +", "3", "0", function()
        if state.baseThrottle < 1.0 then
            state.baseThrottle = math.min(15.0, math.floor((state.baseThrottle + 0.1) * 10 + 0.5) / 10)
        else
            state.baseThrottle = math.min(15.0, state.baseThrottle + 1.0)
        end
    end)
    addButton(3 + bW2, bY2, bW2, 2, "BASE -", "9", "0", function()
        if state.baseThrottle <= 1.0 then
            state.baseThrottle = math.max(0.0, math.floor((state.baseThrottle - 0.1) * 10 + 0.5) / 10)
        else
            state.baseThrottle = math.max(0.0, state.baseThrottle - 1.0)
        end
    end)
    addButton(4 + bW2*2, bY2, bW2, 2, "RE-SCAN", "b", "0", function()
        scanQuadTurtles()
        state.statusMsg = "Engines Re-scanned!"
    end)
    addButton(5 + bW2*3, bY2, bW2, 2, "CALIBRATE", "a", "0", function()
        scanQuadTurtles()
        local cur = altiSensor and altiSensor.getHeight() or 100
        state.calibStartAlt = cur
        state.calibTargetAlt = cur + 30.0
        state.calibPhase = "RAMP"
        state.calibThrottle = 0.05 -- 從超微小脈衝起步
        state.baseThrottle = 0.1
        state.mode = "CALIBRATING"
        state.stableTimer = 0
        altPID:reset()
        pitchPID:reset()
        rollPID:reset()
        state.statusMsg = "Calib: Pulse Ramping Power..."
    end)

    local bY3 = 18
    local mainBW = math.floor((w - 3) / 2)
    local holdBg = (state.mode == "HOLD_ALT") and "5" or "d"
    addButton(2, bY3, mainBW, 3, " [ HOLD ALT ] ", holdBg, "0", function()
        state.mode = "HOLD_ALT"
        altPID:reset()
        pitchPID:reset()
        rollPID:reset()
        state.statusMsg = "Holding Altitude & Balance"
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
        local angles = (gimbalSensor and gimbalSensor.getAngles()) or {0, 0}
        local currPitch, currRoll = angles[1] or 0, angles[2] or 0

        if state.mode == "HOLD_ALT" then
            local altError = state.targetAlt - currAlt
            local deltaAlt = altPID:update(altError)

            local deltaPitch = 0
            local deltaRoll = 0
            if state.autoLevel and gimbalSensor then
                deltaPitch = pitchPID:update(-currPitch)
                deltaRoll  = rollPID:update(-currRoll)
            end

            applyQuadThrust(state.baseThrottle, deltaAlt, deltaPitch, deltaRoll)

        elseif state.mode == "CALIBRATING" then
            -- 姿態水平微調 (隨時保持飛艇平衡)
            local deltaPitch = 0
            local deltaRoll = 0
            if state.autoLevel and gimbalSensor then
                deltaPitch = pitchPID:update(-currPitch)
                deltaRoll  = rollPID:update(-currRoll)
            end

            local climbDist = currAlt - state.calibStartAlt

            if state.calibPhase == "RAMP" then
                -- 階段 1: 緩慢增加推力功率 (從 0.05 脈衝開始，每秒增加約 0.05)
                state.calibThrottle = math.min(15.0, state.calibThrottle + 0.003)
                state.baseThrottle = math.floor(state.calibThrottle * 100 + 0.5) / 100

                -- 輸出四軸推力 (此時不加 deltaAlt，純靠平穩緩慢脈衝加力)
                applyQuadThrust(state.baseThrottle, 0, deltaPitch, deltaRoll)

                state.statusMsg = string.format("Ramping: %4.2f/15 (+%.1fm)", state.baseThrottle, climbDist)

                -- 判定起飛上升超過 30m 或偵測到明顯爬升速度 (> 0.6 m/s): 立即停止提高推力，切入定高自穩！
                if climbDist >= 30.0 or (climbDist >= 1.0 and currVspeed > 0.6) then
                    state.calibPhase = "STABILIZE"
                    state.targetAlt = currAlt -- 鎖定當前高度 (不再往上衝)
                    state.stableTimer = 0
                    altPID:reset()
                    state.statusMsg = string.format("Lift-off! Holding at %.1fm", currAlt)
                end

            elseif state.calibPhase == "STABILIZE" then
                -- 階段 2: 保持在當前定高 (不再繼續往上衝)，PID 介入維持水平與懸停
                local altError = state.targetAlt - currAlt
                local deltaAlt = altPID:update(altError)

                applyQuadThrust(state.baseThrottle, deltaAlt, deltaPitch, deltaRoll)

                local altDiff = math.abs(altError)
                local vDiff = math.abs(currVspeed)
                local attDiff = math.max(math.abs(currPitch), math.abs(currRoll))

                -- 穩定標準: 高度誤差 < 0.8m, 垂直速度 < 0.2m/s, 姿態傾斜 < 1.5°
                if altDiff < 0.8 and vDiff < 0.2 and attDiff < 1.5 then
                    state.stableTimer = state.stableTimer + 0.05
                    state.statusMsg = string.format("Holding & Stabilizing... (%.1f/5.0s)", state.stableTimer)

                    if state.stableTimer >= 5.0 then
                        -- 連續 5 秒平穩懸停，鎖定當前基準檔位！
                        state.mode = "HOLD_ALT"
                        state.statusMsg = string.format("Calibrated! Hover Base: %.2f", state.baseThrottle)
                    end
                else
                    state.stableTimer = 0
                    -- 若微幅掉高或過衝，以極細微步長修正基準
                    if altError > 0.8 and state.baseThrottle < 14 and currVspeed < 0.2 then
                        state.baseThrottle = math.min(15.0, state.baseThrottle + 0.005)
                    elseif altError < -0.8 and state.baseThrottle > 0.05 and currVspeed > -0.2 then
                        state.baseThrottle = math.max(0.01, state.baseThrottle - 0.005)
                    end
                    state.statusMsg = string.format("Stabilizing at %.1fm (B:%4.2f)", state.targetAlt, state.baseThrottle)
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
