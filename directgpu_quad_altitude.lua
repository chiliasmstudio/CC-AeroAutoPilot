--[[
    Create: Avionics & CC: Tweaked + DirectGPU
    DirectGPU Quad-Engine Wired Flight Controller (DirectGPU 四軸有線全彩飛控大腦)
    
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

    local rawFL = baseThrust + deltaAlt - deltaPitch - deltaRoll
    local rawFR = baseThrust + deltaAlt - deltaPitch + deltaRoll
    local rawBL = baseThrust + deltaAlt + deltaPitch - deltaRoll
    local rawBR = baseThrust + deltaAlt + deltaPitch + deltaRoll

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
    targetAlt = 200.0,   -- 預設目標 200m
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

    -- 1. 標題列
    gpu.fillRect(displayId, 0, 0, screenW, 28, 25, 40, 65)
    gpu.drawText(displayId, "QUAD-ENGINE AVIONICS - DIRECTGPU", 12, 7, 240, 245, 255, "Arial", 14, "bold")

    local currAlt = altiSensor and altiSensor.getHeight() or 0
    local currVspeed = altiSensor and altiSensor.getVerticalSpeed() or 0
    local angles = (gimbalSensor and gimbalSensor.getAngles()) or {0, 0}
    local currPitch, currRoll = angles[1] or 0, angles[2] or 0

    -- 2. 左側高度圓形儀表
    local gCX, gCY, gR = 60, 85, 38
    gpu.drawCircle(displayId, gCX, gCY, gR, 35, 45, 60, true)
    gpu.drawCircle(displayId, gCX, gCY, gR, 100, 180, 255, false)
    gpu.drawText(displayId, string.format("%.0f", currAlt), gCX - 15, gCY - 8, 255, 255, 255, "Arial", 16, "bold")
    gpu.drawText(displayId, "ALT(M)", gCX - 15, gCY + 10, 160, 200, 230, "Arial", 9, "plain")

    -- 3. 中間飛行狀態卡
    local cardX = 115
    local cardW = 95
    gpu.fillRect(displayId, cardX, 35, cardW, 85, 25, 32, 48)
    gpu.drawText(displayId, string.format("TGT: %.0fm", state.targetAlt), cardX + 6, 42, 80, 220, 255, "Arial", 11, "bold")
    gpu.drawText(displayId, string.format("V.S: %+.1f", currVspeed), cardX + 6, 58, 255, 200, 80, "Arial", 11, "bold")
    gpu.drawText(displayId, string.format("P:%+.1f*", currPitch), cardX + 6, 74, 180, 200, 220, "Arial", 10, "plain")
    gpu.drawText(displayId, string.format("R:%+.1f*", currRoll), cardX + 6, 88, 180, 200, 220, "Arial", 10, "plain")
    
    local mCol = (state.mode == "HOLD_ALT") and {80, 255, 120} or (state.mode == "CALIBRATING" and {80, 200, 255} or {255, 80, 80})
    gpu.drawText(displayId, state.mode, cardX + 6, 103, mCol[1], mCol[2], mCol[3], "Arial", 10, "bold")

    -- 4. 右側四軸烏龜名稱與推力卡 (顯示 Label 名稱)
    local qCardX = cardX + cardW + 6
    local qCardW = screenW - qCardX - 8
    gpu.fillRect(displayId, qCardX, 35, qCardW, 85, 25, 32, 48)
    gpu.drawText(displayId, "ENGINES (LABEL & SIG)", qCardX + 6, 42, 220, 230, 245, "Arial", 9, "bold")

    local function getEngText(node, def)
        local lbl = node and (node.label or node.name or def) or (def .. ":OFF")
        if #lbl > 6 then lbl = lbl:sub(1, 6) end
        return lbl
    end

    local flLbl = getEngText(engines.FL, "FL")
    local frLbl = getEngText(engines.FR, "FR")
    local blLbl = getEngText(engines.BL, "BL")
    local brLbl = getEngText(engines.BR, "BR")

    gpu.drawText(displayId, string.format("%s:%2d", flLbl, engineOutputs.FL), qCardX + 6, 58, 120, 240, 150, "Arial", 10, "bold")
    gpu.drawText(displayId, string.format("%s:%2d", frLbl, engineOutputs.FR), qCardX + 50, 58, 120, 240, 150, "Arial", 10, "bold")
    gpu.drawText(displayId, string.format("%s:%2d", blLbl, engineOutputs.BL), qCardX + 6, 76, 120, 240, 150, "Arial", 10, "bold")
    gpu.drawText(displayId, string.format("%s:%2d", brLbl, engineOutputs.BR), qCardX + 50, 76, 120, 240, 150, "Arial", 10, "bold")
    gpu.drawText(displayId, string.format("BASE:%4.1f", state.baseThrottle), qCardX + 6, 96, 200, 220, 255, "Arial", 10, "plain")

    -- 5. 觸控按鈕區
    buttons = {}

    local btnY1 = 130
    local btnW1 = math.floor((screenW - 35) / 4)
    local btnH1 = 26

    addButton(8, btnY1, btnW1, btnH1, "+10m", {40, 120, 60}, {255, 255, 255}, function()
        state.targetAlt = state.targetAlt + 10
    end)
    addButton(13 + btnW1, btnY1, btnW1, btnH1, "+1m", {60, 150, 80}, {255, 255, 255}, function()
        state.targetAlt = state.targetAlt + 1
    end)
    addButton(18 + btnW1*2, btnY1, btnW1, btnH1, "-1m", {180, 110, 40}, {255, 255, 255}, function()
        state.targetAlt = math.max(0, state.targetAlt - 1)
    end)
    addButton(23 + btnW1*3, btnY1, btnW1, btnH1, "-10m", {180, 60, 50}, {255, 255, 255}, function()
        state.targetAlt = math.max(0, state.targetAlt - 10)
    end)

    local btnY2 = 162
    local btnW2 = math.floor((screenW - 35) / 4)
    addButton(8, btnY2, btnW2, btnH1, "BASE +", {40, 90, 140}, {255, 255, 255}, function()
        if state.baseThrottle < 1.0 then
            state.baseThrottle = math.min(15.0, math.floor((state.baseThrottle + 0.1) * 10 + 0.5) / 10)
        else
            state.baseThrottle = math.min(15.0, state.baseThrottle + 1.0)
        end
    end)
    addButton(13 + btnW2, btnY2, btnW2, btnH1, "BASE -", {50, 70, 120}, {255, 255, 255}, function()
        if state.baseThrottle <= 1.0 then
            state.baseThrottle = math.max(0.0, math.floor((state.baseThrottle - 0.1) * 10 + 0.5) / 10)
        else
            state.baseThrottle = math.max(0.0, state.baseThrottle - 1.0)
        end
    end)
    addButton(18 + btnW2*2, btnY2, btnW2, btnH1, "RE-SCAN", {40, 120, 150}, {255, 255, 255}, function()
        scanQuadTurtles()
        state.statusMsg = "Engines Re-scanned!"
    end)
    addButton(23 + btnW2*3, btnY2, btnW2, btnH1, "CALIBRATE", {120, 60, 160}, {255, 255, 255}, function()
        scanQuadTurtles()
        local cur = altiSensor and altiSensor.getHeight() or 100
        state.calibStartAlt = cur
        state.calibTargetAlt = cur + 30.0
        state.calibPhase = "RAMP"
        state.calibThrottle = 0.05
        state.baseThrottle = 0.1
        state.mode = "CALIBRATING"
        state.stableTimer = 0
        altPID:reset()
        pitchPID:reset()
        rollPID:reset()
        state.statusMsg = "Calib: Pulse Ramping Power..."
    end)

    local btnY3 = 194
    local mainBW = math.floor((screenW - 25) / 2)
    local mainBH = 32

    local holdBg = (state.mode == "HOLD_ALT") and {30, 180, 80} or {50, 80, 60}
    addButton(8, btnY3, mainBW, mainBH, "HOLD ALT", holdBg, {255, 255, 255}, function()
        state.mode = "HOLD_ALT"
        altPID:reset()
        pitchPID:reset()
        rollPID:reset()
        state.statusMsg = "Holding Altitude & Balance"
    end)

    local stopBg = (state.mode == "IDLE") and {180, 40, 40} or {90, 40, 40}
    addButton(13 + mainBW, btnY3, mainBW, mainBH, "STOP / IDLE", stopBg, {255, 255, 255}, function()
        state.mode = "IDLE"
        altPID:reset()
        applyQuadThrust(0, 0, 0, 0)
        state.statusMsg = "All 4 Engines Stopped"
    end)

    for _, btn in ipairs(buttons) do
        gpu.fillRect(displayId, btn.x, btn.y, btn.w, btn.h, btn.bg[1], btn.bg[2], btn.bg[3])
        local tx = btn.x + math.max(4, math.floor((btn.w - #btn.text * 7) / 2))
        local ty = btn.y + math.floor((btn.h - 11) / 2)
        gpu.drawText(displayId, btn.text, tx, ty, btn.fg[1], btn.fg[2], btn.fg[3], "Arial", 11, "bold")
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
            local deltaPitch = 0
            local deltaRoll = 0
            if state.autoLevel and gimbalSensor then
                deltaPitch = pitchPID:update(-currPitch)
                deltaRoll  = rollPID:update(-currRoll)
            end

            local climbDist = currAlt - state.calibStartAlt

            if state.calibPhase == "RAMP" then
                state.calibThrottle = math.min(15.0, state.calibThrottle + 0.003)
                state.baseThrottle = math.floor(state.calibThrottle * 100 + 0.5) / 100

                applyQuadThrust(state.baseThrottle, 0, deltaPitch, deltaRoll)

                state.statusMsg = string.format("Ramping: %4.2f/15 (+%.1fm)", state.baseThrottle, climbDist)

                if climbDist >= 30.0 or (climbDist >= 1.0 and currVspeed > 0.6) then
                    state.calibPhase = "STABILIZE"
                    state.targetAlt = currAlt
                    state.stableTimer = 0
                    altPID:reset()
                    state.statusMsg = string.format("Lift-off! Holding at %.1fm", currAlt)
                end

            elseif state.calibPhase == "STABILIZE" then
                local altError = state.targetAlt - currAlt
                local deltaAlt = altPID:update(altError)

                applyQuadThrust(state.baseThrottle, deltaAlt, deltaPitch, deltaRoll)

                local altDiff = math.abs(altError)
                local vDiff = math.abs(currVspeed)
                local attDiff = math.max(math.abs(currPitch), math.abs(currRoll))

                if altDiff < 0.8 and vDiff < 0.2 and attDiff < 1.5 then
                    state.stableTimer = state.stableTimer + 0.05
                    state.statusMsg = string.format("Holding & Stabilizing (%.1f/5.0s)", state.stableTimer)
                    
                    if state.stableTimer >= 5.0 then
                        state.mode = "HOLD_ALT"
                        state.statusMsg = string.format("Calibrated! Hover Base: %.2f", state.baseThrottle)
                    end
                else
                    state.stableTimer = 0
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
