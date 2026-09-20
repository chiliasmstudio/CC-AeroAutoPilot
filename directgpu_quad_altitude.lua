--[[
    Create: Avionics & CC: Tweaked + DirectGPU
    DirectGPU Quad-Engine Wired Flight Controller (DirectGPU 四軸有線全彩飛控大腦)
    
    智慧校準與姿態自穩邏輯:
    - 內建 Gimbal Sensor 防呆與熱插拔檢測機制 (無感測器時自動降級防崩潰)。
    - 啟動 CALIBRATE 時，以極微小 PWM 脈衝緩慢均勻加力起飛，目標鎖定在 (起飛高度 + 10格) 懸浮。
    - 爬升與飛行途中即時姿態反向補償：任一側傾斜立刻對低側加力、高側減力，維持水平姿態。
    - 當高度誤差 < 0.6m、垂直速度 < 0.15m/s、姿態傾斜 < 1.0° 持續穩定 5 秒，自動鎖定基準推力並切換定高！
    - 支援自由自訂目標高度 (Target Altitude) 或一鍵同步當前高度 (SET CURR)。
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
-- 2. 硬體與 DirectGPU 初始化
-- ========================================================
local gpu = peripheral.find("directgpu")
if not gpu then
    error("未找到 DirectGPU 設備！請確保電腦相鄰放置了 DirectGPU 方塊。")
end

local displayId = gpu.autoDetectAndCreateDisplay()
if not displayId then
    error("DirectGPU 未能檢測到 Monitor 螢幕！")
end

local dispInfo = gpu.getDisplayInfo(displayId)
local screenW = dispInfo.pixelWidth or 320
local screenH = dispInfo.pixelHeight or 240

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

-- ========================================================
-- 3. 四角烏龜引擎節點掃描 (FL, FR, BL, BR)
-- ========================================================
local engines = { FL = nil, FR = nil, BL = nil, BR = nil }
local engineOutputs = { FL = 0, FR = 0, BL = 0, BR = 0 }

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
    pwmTick = (pwmTick + 1) % 10

    -- 四軸混控公式：
    -- Pitch > 0 (前傾/低頭) => deltaPitch < 0 => 前部 FL/FR 增加推力，後部 BL/BR 減少推力
    -- Roll  > 0 (右傾)     => deltaRoll  < 0 => 右側 FR/BR 增加推力，左側 FL/BL 減少推力
    local rawFL = baseThrust + deltaAlt - deltaPitch + deltaRoll
    local rawFR = baseThrust + deltaAlt - deltaPitch - deltaRoll
    local rawBL = baseThrust + deltaAlt + deltaPitch + deltaRoll
    local rawBR = baseThrust + deltaAlt + deltaPitch - deltaRoll

    local function resolvePwmOutput(val)
        if val <= 0 then return 0 end
        if val < 1.0 then
            local activeThreshold = math.floor(val * 10 + 0.5)
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
-- 4. 飛控狀態與 PID 控制器
-- ========================================================
local state = {
    mode = "IDLE",       -- "IDLE", "HOLD_ALT", "CALIBRATING"
    targetAlt = 100.0,   -- 全局目標高度
    baseThrottle = 1.0,  -- 支援浮點推力 (0.01 ~ 15.0)，< 1 自動進入 PWM 脈衝模式
    autoLevel = true,    -- 自動水平姿態開關
    statusMsg = "System Ready",
    stableTimer = 0,     -- 穩定計時器 (秒)
    calibStartAlt = 0,   -- 校正啟動高度
    calibTargetAlt = 0,  -- 校正懸停目標 (起飛高度 + 10m)
    calibPhase = "RAMP", -- "RAMP" (緩慢脈衝加力), "STABILIZE" (保持定高自穩)
    calibThrottle = 0.02 -- 校正微調油門 (從極微小 0.02 脈衝起步)
}

-- PID 控制器：針對姿態失衡加大反應速度與增益
local altPID   = PID.new(0.7, 0.05, 0.8, -8, 8)
local pitchPID = PID.new(0.25, 0.01, 0.12, -8, 8)
local rollPID  = PID.new(0.25, 0.01, 0.12, -8, 8)

local buttons = {}

local function addButton(x, y, w, h, text, bg, fg, action)
    table.insert(buttons, {
        x = x, y = y, w = w, h = h,
        text = text, bg = bg, fg = fg,
        action = action
    })
end

-- ========================================================
-- 5. DirectGPU 渲染介面 (24-bit True RGB)
-- ========================================================
local function drawDirectGPU_UI()
    gpu.clear(displayId, 15, 20, 30)

    -- 1. 頂部標題列
    gpu.fillRect(displayId, 0, 0, screenW, 26, 25, 40, 65)
    gpu.drawText(displayId, "QUAD-ENGINE AVIONICS - DIRECTGPU", 12, 6, 240, 245, 255, "Arial", 13, "bold")

    local currAlt = altiSensor and altiSensor.getHeight() or 0
    local currVspeed = altiSensor and altiSensor.getVerticalSpeed() or 0
    local currPitch, currRoll = getGimbalData()

    -- 2. 左側高度圓形儀表
    local gCX, gCY, gR = 50, 75, 34
    gpu.drawCircle(displayId, gCX, gCY, gR, 35, 45, 60, true)
    gpu.drawCircle(displayId, gCX, gCY, gR, 100, 180, 255, false)
    gpu.drawText(displayId, string.format("%.0f", currAlt), gCX - 13, gCY - 7, 255, 255, 255, "Arial", 15, "bold")
    gpu.drawText(displayId, "ALT(M)", gCX - 15, gCY + 9, 160, 200, 230, "Arial", 8, "plain")

    -- 3. 中間飛行狀態卡 (顯示目標高度、垂直速度、Gimbal 狀態與角度)
    local cardX = 96
    local cardW = 108
    gpu.fillRect(displayId, cardX, 32, cardW, 88, 25, 32, 48)
    gpu.drawText(displayId, string.format("TGT: %.0fm", state.targetAlt), cardX + 5, 38, 80, 220, 255, "Arial", 10, "bold")
    gpu.drawText(displayId, string.format("V.S: %+.1fm/s", currVspeed), cardX + 5, 52, 255, 200, 80, "Arial", 10, "bold")
    
    if gimbalAvailable then
        gpu.drawText(displayId, string.format("P:%+4.1f* R:%+4.1f*", currPitch, currRoll), cardX + 5, 66, 180, 220, 255, "Arial", 9, "plain")
        gpu.drawText(displayId, "GIMBAL: ACTIVE", cardX + 5, 80, 80, 255, 120, "Arial", 9, "bold")
    else
        gpu.drawText(displayId, "P: ---   R: ---", cardX + 5, 66, 150, 150, 150, "Arial", 9, "plain")
        gpu.drawText(displayId, "GIMBAL: NO SENSOR", cardX + 5, 80, 255, 60, 60, "Arial", 9, "bold")
    end
    
    local mCol = (state.mode == "HOLD_ALT") and {80, 255, 120} or (state.mode == "CALIBRATING" and {80, 200, 255} or {255, 80, 80})
    gpu.drawText(displayId, string.format("MODE: %s", state.mode), cardX + 5, 96, mCol[1], mCol[2], mCol[3], "Arial", 9, "bold")

    -- 4. 右側四軸烏龜名稱與推力卡 (顯示 Label 名稱)
    local qCardX = cardX + cardW + 6
    local qCardW = screenW - qCardX - 6
    gpu.fillRect(displayId, qCardX, 32, qCardW, 88, 25, 32, 48)
    gpu.drawText(displayId, "ENGINES (LABEL & SIG)", qCardX + 5, 38, 220, 230, 245, "Arial", 8, "bold")

    local function getEngText(node, def)
        local lbl = node and (node.label or node.name or def) or (def .. ":OFF")
        if #lbl > 5 then lbl = lbl:sub(1, 5) end
        return lbl
    end

    local flLbl = getEngText(engines.FL, "FL")
    local frLbl = getEngText(engines.FR, "FR")
    local blLbl = getEngText(engines.BL, "BL")
    local brLbl = getEngText(engines.BR, "BR")

    gpu.drawText(displayId, string.format("%s:%2d", flLbl, engineOutputs.FL), qCardX + 5, 52, 120, 240, 150, "Arial", 9, "bold")
    gpu.drawText(displayId, string.format("%s:%2d", frLbl, engineOutputs.FR), qCardX + 50, 52, 120, 240, 150, "Arial", 9, "bold")
    gpu.drawText(displayId, string.format("%s:%2d", blLbl, engineOutputs.BL), qCardX + 5, 67, 120, 240, 150, "Arial", 9, "bold")
    gpu.drawText(displayId, string.format("%s:%2d", brLbl, engineOutputs.BR), qCardX + 50, 67, 120, 240, 150, "Arial", 9, "bold")
    gpu.drawText(displayId, string.format("BASE: %4.2f", state.baseThrottle), qCardX + 5, 83, 200, 220, 255, "Arial", 9, "plain")
    
    local autoLvlCol = (state.autoLevel and gimbalAvailable) and {80, 255, 120} or {255, 90, 90}
    gpu.drawText(displayId, string.format("AUTO-LVL: %s", (state.autoLevel and gimbalAvailable) and "ON" or "OFF"), qCardX + 5, 96, autoLvlCol[1], autoLvlCol[2], autoLvlCol[3], "Arial", 8, "bold")

    -- 狀態訊息小橫條
    gpu.fillRect(displayId, 6, 124, screenW - 12, 16, 20, 26, 38)
    gpu.drawText(displayId, string.format("STATUS: %s", state.statusMsg), 10, 127, 240, 240, 240, "Arial", 9, "plain")

    -- 5. 觸控按鈕區
    buttons = {}

    -- 第一排按鈕：目標高度調整 (+10m, +1m, -1m, -10m, SET CURR)
    local btnY1 = 144
    local btnW1 = math.floor((screenW - 36) / 5)
    local btnH1 = 24

    addButton(6, btnY1, btnW1, btnH1, "+10m", {40, 120, 60}, {255, 255, 255}, function()
        state.targetAlt = state.targetAlt + 10
    end)
    addButton(12 + btnW1, btnY1, btnW1, btnH1, "+1m", {60, 150, 80}, {255, 255, 255}, function()
        state.targetAlt = state.targetAlt + 1
    end)
    addButton(18 + btnW1*2, btnY1, btnW1, btnH1, "-1m", {180, 110, 40}, {255, 255, 255}, function()
        state.targetAlt = math.max(0, state.targetAlt - 1)
    end)
    addButton(24 + btnW1*3, btnY1, btnW1, btnH1, "-10m", {180, 60, 50}, {255, 255, 255}, function()
        state.targetAlt = math.max(0, state.targetAlt - 10)
    end)
    addButton(30 + btnW1*4, btnY1, btnW1, btnH1, "SET CURR", {40, 110, 160}, {255, 255, 255}, function()
        local cur = altiSensor and altiSensor.getHeight() or 0
        state.targetAlt = math.floor(cur + 0.5)
        state.statusMsg = string.format("Target locked to current: %.0fm", state.targetAlt)
    end)

    -- 第二排按鈕：基準油門微調、重新掃描、校正 (+10格懸浮自穩)
    local btnY2 = 172
    local btnW2 = math.floor((screenW - 30) / 4)
    local btnH2 = 24

    addButton(6, btnY2, btnW2, btnH2, "BASE +", {40, 90, 140}, {255, 255, 255}, function()
        if state.baseThrottle < 1.0 then
            state.baseThrottle = math.min(15.0, math.floor((state.baseThrottle + 0.05) * 100 + 0.5) / 100)
        else
            state.baseThrottle = math.min(15.0, state.baseThrottle + 1.0)
        end
    end)
    addButton(12 + btnW2, btnY2, btnW2, btnH2, "BASE -", {50, 70, 120}, {255, 255, 255}, function()
        if state.baseThrottle <= 1.0 then
            state.baseThrottle = math.max(0.0, math.floor((state.baseThrottle - 0.05) * 100 + 0.5) / 100)
        else
            state.baseThrottle = math.max(0.0, state.baseThrottle - 1.0)
        end
    end)
    addButton(18 + btnW2*2, btnY2, btnW2, btnH2, "RE-SCAN", {40, 120, 150}, {255, 255, 255}, function()
        scanQuadTurtles()
        state.statusMsg = "Hardware Re-scanned!"
    end)
    addButton(24 + btnW2*3, btnY2, btnW2, btnH2, "CALIBRATE", {120, 60, 160}, {255, 255, 255}, function()
        scanQuadTurtles()
        local cur = altiSensor and altiSensor.getHeight() or 100
        state.calibStartAlt = cur
        state.calibTargetAlt = cur + 10.0 -- 懸停在 +10格 (起飛高度 + 10m)
        state.targetAlt = cur + 10.0
        state.calibPhase = "RAMP"
        state.calibThrottle = 0.02 -- 從極微小脈衝起步
        state.baseThrottle = 0.02
        state.mode = "CALIBRATING"
        state.stableTimer = 0
        altPID:reset()
        pitchPID:reset()
        rollPID:reset()
        if not gimbalAvailable then
            state.statusMsg = "[WARN] No Gimbal! Calib Ramping..."
        else
            state.statusMsg = "Calib: Auto-Level Ramping (+10m)..."
        end
    end)

    -- 第三排按鈕：主要飛控模式
    local btnY3 = 200
    local mainBW = math.floor((screenW - 20) / 2)
    local mainBH = 30

    local holdBg = (state.mode == "HOLD_ALT") and {30, 180, 80} or {50, 80, 60}
    addButton(6, btnY3, mainBW, mainBH, "HOLD ALT", holdBg, {255, 255, 255}, function()
        state.mode = "HOLD_ALT"
        altPID:reset()
        pitchPID:reset()
        rollPID:reset()
        if not gimbalAvailable then
            state.statusMsg = "[WARN] Holding Altitude (No Gimbal)"
        else
            state.statusMsg = string.format("Holding Altitude: %.0fm", state.targetAlt)
        end
    end)

    local stopBg = (state.mode == "IDLE") and {180, 40, 40} or {90, 40, 40}
    addButton(14 + mainBW, btnY3, mainBW, mainBH, "STOP / IDLE", stopBg, {255, 255, 255}, function()
        state.mode = "IDLE"
        altPID:reset()
        applyQuadThrust(0, 0, 0, 0)
        state.statusMsg = "All 4 Engines Stopped"
    end)

    for _, btn in ipairs(buttons) do
        gpu.fillRect(displayId, btn.x, btn.y, btn.w, btn.h, btn.bg[1], btn.bg[2], btn.bg[3])
        local tx = btn.x + math.max(2, math.floor((btn.w - #btn.text * 7) / 2))
        local ty = btn.y + math.floor((btn.h - 10) / 2)
        gpu.drawText(displayId, btn.text, tx, ty, btn.fg[1], btn.fg[2], btn.fg[3], "Arial", 10, "bold")
    end

    gpu.updateDisplay(displayId)
end

-- ========================================================
-- 6. 飛控閉環控制迴圈 (20Hz)
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
            local altError = state.targetAlt - currAlt
            local deltaAlt = altPID:update(altError)

            local deltaPitch = 0
            local deltaRoll = 0
            if state.autoLevel and gimbalAvailable then
                deltaPitch = pitchPID:update(-currPitch)
                deltaRoll  = rollPID:update(-currRoll)
            end

            applyQuadThrust(state.baseThrottle, deltaAlt, deltaPitch, deltaRoll)

        elseif state.mode == "CALIBRATING" then
            -- 姿態水平微調 (途中不平衡立刻對另一邊增加推力補償)
            local deltaPitch = 0
            local deltaRoll = 0
            if state.autoLevel and gimbalAvailable then
                deltaPitch = pitchPID:update(-currPitch)
                deltaRoll  = rollPID:update(-currRoll)
            end

            local climbDist = currAlt - state.calibStartAlt

            if state.calibPhase == "RAMP" then
                -- 階段 1: 緩慢平均增加推力功率 (從 0.02 脈衝開始，每秒增加約 0.04)
                state.calibThrottle = math.min(15.0, state.calibThrottle + 0.002)
                state.baseThrottle = math.floor(state.calibThrottle * 100 + 0.5) / 100

                -- 輸出四軸推力：加力爬升途中，若有傾斜立即對另一側加力配平
                applyQuadThrust(state.baseThrottle, 0, deltaPitch, deltaRoll)

                state.statusMsg = string.format("Ramping: %4.2f/15 (+%.1f/+10m)", state.baseThrottle, climbDist)

                -- 判定起飛上升到達 +10m (或偵測到起飛明顯垂直爬升速度 > 0.6 m/s):
                -- 立即停止增加基礎推力，無縫切入 +10m 定高懸停自穩階段！
                if climbDist >= 10.0 or (climbDist >= 0.8 and currVspeed > 0.6) then
                    state.calibPhase = "STABILIZE"
                    state.targetAlt = state.calibTargetAlt
                    state.stableTimer = 0
                    altPID:reset()
                    state.statusMsg = string.format("Lifted! Hovering at +10m (%.1fm)", state.targetAlt)
                end

            elseif state.calibPhase == "STABILIZE" then
                -- 階段 2: 保持在 (起飛高度 + 10m) 懸停，PID 自動調節並尋找最佳懸停推力
                local altError = state.targetAlt - currAlt
                local deltaAlt = altPID:update(altError)

                applyQuadThrust(state.baseThrottle, deltaAlt, deltaPitch, deltaRoll)

                local altDiff = math.abs(altError)
                local vDiff = math.abs(currVspeed)
                local attDiff = math.max(math.abs(currPitch), math.abs(currRoll))

                -- 穩定標準: 高度誤差 < 0.6m, 垂直速度 < 0.15m/s, 姿態傾角 < 1.0° (若無 Gimbal 則忽略姿態條件)
                local attOk = (not gimbalAvailable) or (attDiff < 1.0)
                if altDiff < 0.6 and vDiff < 0.15 and attOk then
                    state.stableTimer = state.stableTimer + 0.05
                    state.statusMsg = string.format("Hover Stabilizing... (%.1f/5.0s)", state.stableTimer)
                    
                    if state.stableTimer >= 5.0 then
                        -- 連續 5 秒懸停平穩，鎖定當前基準檔位！
                        state.mode = "HOLD_ALT"
                        state.statusMsg = string.format("Calibrated! Hover Base: %.2f", state.baseThrottle)
                    end
                else
                    state.stableTimer = 0
                    -- 若微幅掉高或過衝，以極細微步長修正基準
                    if altError > 0.6 and state.baseThrottle < 14 and currVspeed < 0.2 then
                        state.baseThrottle = math.min(15.0, state.baseThrottle + 0.003)
                    elseif altError < -0.6 and state.baseThrottle > 0.02 and currVspeed > -0.2 then
                        state.baseThrottle = math.max(0.01, state.baseThrottle - 0.003)
                    end
                    state.statusMsg = string.format("Balancing at %.1fm (B:%4.2f)", state.targetAlt, state.baseThrottle)
                end
            end
            
        elseif state.mode == "IDLE" then
            applyQuadThrust(0, 0, 0, 0)
        end
        sleep(0.05)
    end
end

-- ========================================================
-- 7. 畫面渲染與 DirectGPU 專屬事件監聽
-- ========================================================
local function processClick(clickX, clickY)
    if not clickX or not clickY then return end
    for _, btn in ipairs(buttons) do
        if clickX >= btn.x and clickX < btn.x + btn.w and
           clickY >= btn.y and clickY < btn.y + btn.h then
            btn.action()
            drawDirectGPU_UI()
            break
        end
    end
end

local function renderLoop()
    while true do
        -- 處理 DirectGPU 專屬內部隊列事件 (gpu.pollEvent)
        if gpu.hasEvents and gpu.pollEvent then
            while gpu.hasEvents(displayId) do
                local ev = gpu.pollEvent(displayId)
                if ev and (ev.type == "mouse_click" or ev.type == "click" or ev.type == "touch") then
                    processClick(ev.x, ev.y)
                end
            end
        end

        drawDirectGPU_UI()
        sleep(0.05)
    end
end

local function touchEventLoop()
    local mon = peripheral.find("monitor")
    local monCharW, monCharH = 1, 1
    if mon then
        monCharW, monCharH = mon.getSize()
    end

    while true do
        local event, p1, p2, p3 = os.pullEvent()

        if event == "directgpu_touch" then
            -- p1=displayId, p2=pixelX, p3=pixelY
            processClick(p2, p3)
        elseif event == "monitor_touch" then
            if not mon then mon = peripheral.find("monitor") end
            if mon then monCharW, monCharH = mon.getSize() end
            local clickX = math.floor(((p2 - 0.5) / monCharW) * screenW)
            local clickY = math.floor(((p3 - 0.5) / monCharH) * screenH)
            processClick(clickX, clickY)
        elseif event == "mouse_click" then
            -- 支援原生事件 table 或字元座標
            if type(p1) == "table" and p1.x and p1.y then
                processClick(p1.x, p1.y)
            elseif type(p2) == "number" and type(p3) == "number" then
                local termW, termH = term.getSize()
                local clickX = math.floor(((p2 - 0.5) / termW) * screenW)
                local clickY = math.floor(((p3 - 0.5) / termH) * screenH)
                processClick(clickX, clickY)
            end
        end
    end
end

parallel.waitForAll(flightControlLoop, renderLoop, touchEventLoop)
