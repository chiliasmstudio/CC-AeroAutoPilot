--[[
    Create: Avionics & CC: Tweaked + DirectGPU
    DirectGPU Quad-Engine Wired Flight Controller (DirectGPU 四軸有線飛控大腦)
    
    特色:
    - 採用 DirectGPU 24-bit True RGB 全彩高解析度渲染 + 觸控互動。
    - 四軸有線混控 (Quad-Mixer Matrix): 左前 (FL)、右前 (FR)、左後 (BL)、右後 (BR)。
    - 烏龜【100% 免跑程式、免開機】，由主電腦直接遠程控制。
    - 閉環高度 PID + 姿態自穩 (Pitch / Roll 平衡)。
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

local function scanQuadTurtles()
    engines.FL = nil
    engines.FR = nil
    engines.BL = nil
    engines.BR = nil

    local unmapped = {}
    local pNames = peripheral.getNames()

    for _, name in ipairs(pNames) do
        local pType = peripheral.getType(name)
        if pType == "turtle" or pType == "computer" or pType == "redstone_relay" then
            local p = peripheral.wrap(name)
            local label = (p.getLabel and p.getLabel()) or ""
            local upperLabel = string.upper(label)

            if upperLabel == "FL" or upperLabel == "FRONT_LEFT" or string.find(upperLabel, "FL") then
                engines.FL = { name = name, p = p, label = label }
            elseif upperLabel == "FR" or upperLabel == "FRONT_RIGHT" or string.find(upperLabel, "FR") then
                engines.FR = { name = name, p = p, label = label }
            elseif upperLabel == "BL" or upperLabel == "BACK_LEFT" or string.find(upperLabel, "BL") then
                engines.BL = { name = name, p = p, label = label }
            elseif upperLabel == "BR" or upperLabel == "BACK_RIGHT" or string.find(upperLabel, "BR") then
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

local function outputToEngine(node, signal)
    if not node or not node.p then return end
    signal = math.max(0, math.min(15, math.floor(signal + 0.5)))
    for _, side in ipairs({"bottom", "top", "left", "right", "front", "back"}) do
        pcall(function() node.p.setAnalogOutput(side, signal) end)
    end
end

local function applyQuadThrust(baseThrust, deltaAlt, deltaPitch, deltaRoll)
    local outFL = baseThrust + deltaAlt - deltaPitch - deltaRoll
    local outFR = baseThrust + deltaAlt - deltaPitch + deltaRoll
    local outBL = baseThrust + deltaAlt + deltaPitch - deltaRoll
    local outBR = baseThrust + deltaAlt + deltaPitch + deltaRoll

    engineOutputs.FL = math.max(0, math.min(15, math.floor(outFL + 0.5)))
    engineOutputs.FR = math.max(0, math.min(15, math.floor(outFR + 0.5)))
    engineOutputs.BL = math.max(0, math.min(15, math.floor(outBL + 0.5)))
    engineOutputs.BR = math.max(0, math.min(15, math.floor(outBR + 0.5)))

    outputToEngine(engines.FL, engineOutputs.FL)
    outputToEngine(engines.FR, engineOutputs.FR)
    outputToEngine(engines.BL, engineOutputs.BL)
    outputToEngine(engines.BR, engineOutputs.BR)
end

-- ========================================================
-- 4. 飛控狀態與 PID 控制器
-- ========================================================
local state = {
    mode = "IDLE",       -- "IDLE", "HOLD_ALT", "CALIBRATING"
    targetAlt = 120.0,
    baseThrottle = 7,
    autoLevel = true,
    statusMsg = "DirectGPU Quad Engine Ready"
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
    local cardW = 100
    gpu.fillRect(displayId, cardX, 35, cardW, 85, 25, 32, 48)
    gpu.drawText(displayId, string.format("TGT: %.0fm", state.targetAlt), cardX + 8, 42, 80, 220, 255, "Arial", 11, "bold")
    gpu.drawText(displayId, string.format("V.S: %+.1f", currVspeed), cardX + 8, 58, 255, 200, 80, "Arial", 11, "bold")
    gpu.drawText(displayId, string.format("P:%+.1f*", currPitch), cardX + 8, 74, 180, 200, 220, "Arial", 10, "plain")
    gpu.drawText(displayId, string.format("R:%+.1f*", currRoll), cardX + 8, 88, 180, 200, 220, "Arial", 10, "plain")
    
    local mCol = (state.mode == "HOLD_ALT") and {80, 255, 120} or {255, 80, 80}
    gpu.drawText(displayId, state.mode, cardX + 8, 103, mCol[1], mCol[2], mCol[3], "Arial", 10, "bold")

    -- 4. 右側四軸引擎推力卡 (FL, FR, BL, BR)
    local qCardX = cardX + cardW + 8
    local qCardW = screenW - qCardX - 8
    gpu.fillRect(displayId, qCardX, 35, qCardW, 85, 25, 32, 48)
    gpu.drawText(displayId, "ENGINES", qCardX + 8, 42, 220, 230, 245, "Arial", 10, "bold")

    local function engCol(node) return node and {80, 255, 120} or {255, 70, 70} end
    local flC, frC = engCol(engines.FL), engCol(engines.FR)
    local blC, brC = engCol(engines.BL), engCol(engines.BR)

    gpu.drawText(displayId, string.format("FL:%2d", engineOutputs.FL), qCardX + 8, 58, flC[1], flC[2], flC[3], "Arial", 11, "bold")
    gpu.drawText(displayId, string.format("FR:%2d", engineOutputs.FR), qCardX + 50, 58, frC[1], frC[2], frC[3], "Arial", 11, "bold")
    gpu.drawText(displayId, string.format("BL:%2d", engineOutputs.BL), qCardX + 8, 78, blC[1], blC[2], blC[3], "Arial", 11, "bold")
    gpu.drawText(displayId, string.format("BR:%2d", engineOutputs.BR), qCardX + 50, 78, brC[1], brC[2], brC[3], "Arial", 11, "bold")
    gpu.drawText(displayId, string.format("BASE:%2d", state.baseThrottle), qCardX + 8, 98, 200, 220, 255, "Arial", 10, "plain")

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
    local btnW2 = math.floor((screenW - 30) / 3)
    addButton(8, btnY2, btnW2, btnH1, "BASE +1", {40, 90, 140}, {255, 255, 255}, function()
        state.baseThrottle = math.min(15, state.baseThrottle + 1)
    end)
    addButton(13 + btnW2, btnY2, btnW2, btnH1, "BASE -1", {50, 70, 120}, {255, 255, 255}, function()
        state.baseThrottle = math.max(0, state.baseThrottle - 1)
    end)
    addButton(18 + btnW2*2, btnY2, btnW2, btnH1, "CALIBRATE", {120, 60, 160}, {255, 255, 255}, function()
        scanQuadTurtles()
        state.mode = "CALIBRATING"
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
    while true do
        if state.mode == "HOLD_ALT" and altiSensor then
            local currentAlt = altiSensor.getHeight()
            local altError = state.targetAlt - currentAlt
            local deltaAlt = altPID:update(altError)

            local deltaPitch = 0
            local deltaRoll = 0
            if state.autoLevel and gimbalSensor then
                local angles = gimbalSensor.getAngles() or {0, 0}
                local currPitch, currRoll = angles[1] or 0, angles[2] or 0
                deltaPitch = pitchPID:update(-currPitch)
                deltaRoll  = rollPID:update(-currRoll)
            end

            applyQuadThrust(state.baseThrottle, deltaAlt, deltaPitch, deltaRoll)
            
        elseif state.mode == "CALIBRATING" then
            state.statusMsg = "Calibrating 4 engines..."
            drawDirectGPU_UI()
            local bestSignal = 0
            local minVspeed = 999

            for sig = 0, 15 do
                applyQuadThrust(sig, 0, 0, 0)
                state.statusMsg = string.format("Testing Quad [%2d/15]", sig)
                drawDirectGPU_UI()
                sleep(1.0)

                local vspeed = altiSensor and altiSensor.getVerticalSpeed() or 0
                if math.abs(vspeed) < minVspeed then
                    minVspeed = math.abs(vspeed)
                    bestSignal = sig
                end
                if vspeed > 0.3 and sig > 0 then break end
            end

            state.baseThrottle = bestSignal
            applyQuadThrust(bestSignal, 0, 0, 0)
            state.mode = "HOLD_ALT"
            state.statusMsg = string.format("Calib Done: %d", bestSignal)
            
        elseif state.mode == "IDLE" then
            applyQuadThrust(0, 0, 0, 0)
        end
        sleep(0.05)
    end
end

-- ========================================================
-- 7. 畫面渲染與觸控監聽
-- ========================================================
local function renderLoop()
    while true do
        drawDirectGPU_UI()
        sleep(0.05)
    end
end

local function touchEventLoop()
    while true do
        local event, id, x, y = os.pullEvent()
        if event == "directgpu_touch" or event == "monitor_touch" or event == "mouse_click" then
            local clickX, clickY = x, y
            if event == "mouse_click" then clickX, clickY = id, x end

            for _, btn in ipairs(buttons) do
                if clickX >= btn.x and clickX < btn.x + btn.w and
                   clickY >= btn.y and clickY < btn.y + btn.h then
                    btn.action()
                    drawDirectGPU_UI()
                    break
                end
            end
        end
    end
end

parallel.waitForAll(flightControlLoop, renderLoop, touchEventLoop)
