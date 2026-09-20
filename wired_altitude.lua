--[[
    Create: Avionics & CC: Tweaked
    Wired Cable Multi-Engine Altitude Controller (純有線網路纜線版 - 無需烏龜)
    
    硬體需求:
    - 駕駛艙主電腦 (Advanced Computer)
    - CC Networking Cable (有線網路纜線)
    - 在每個動力機構 (變速箱/離合器/油門拉桿) 旁放置:
        * Redstone Relay (紅石中繼周邊) 或 Wired Modem (有線數據機) 或 Throttle Lever
    - 飛艇上放置 1 個 Altitude Sensor
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

local altiSensor = peripheral.find("altitude_sensor")

-- ========================================================
-- 3. 自動掃描有線網路上的所有紅石輸出節點 (Wired Actuators)
-- ========================================================
local function scanWiredEngines()
    local engineNodes = {}
    local pNames = peripheral.getNames()

    for _, name in ipairs(pNames) do
        local pType = peripheral.getType(name)
        
        -- 支援 1: Redstone Relay
        if pType == "redstone_relay" then
            table.insert(engineNodes, { type = "relay", p = peripheral.wrap(name), name = name })
            
        -- 支援 2: Throttle Lever (油門拉桿)
        elseif pType == "throttle_lever" then
            table.insert(engineNodes, { type = "lever", p = peripheral.wrap(name), name = name })
            
        -- 支援 3: Wired Modem 上的紅石輸出 (如果該面是 modem 且支援紅石)
        elseif pType == "modem" and peripheral.hasType(name, "redstone") then
            table.insert(engineNodes, { type = "modem_rs", p = peripheral.wrap(name), name = name })
        end
    end
    return engineNodes
end

local engines = scanWiredEngines()

-- 同步向全網所有有線節點輸出類比紅石
local function outputToAllWiredEngines(signal)
    signal = math.max(0, math.min(15, math.floor(signal + 0.5)))
    
    for _, node in ipairs(engines) do
        if node.type == "relay" then
            -- Redstone Relay 支援向各面輸出
            node.p.setAnalogOutput("bottom", signal)
            node.p.setAnalogOutput("top", signal)
            node.p.setAnalogOutput("left", signal)
            node.p.setAnalogOutput("right", signal)
            node.p.setAnalogOutput("front", signal)
            node.p.setAnalogOutput("back", signal)
        elseif node.type == "lever" then
            node.p.setSignal(signal)
        elseif node.type == "modem_rs" then
            node.p.setAnalogOutput(signal)
        end
    end

    -- 本機電腦自帶紅石端也同步輸出
    for _, side in ipairs({"top", "bottom", "left", "right", "back", "front"}) do
        pcall(function() redstone.setAnalogOutput(side, signal) end)
    end
end

-- ========================================================
-- 4. 飛控狀態與按鈕定義
-- ========================================================
local state = {
    mode = "IDLE",       -- "IDLE", "HOLD_ALT", "CALIBRATING"
    targetAlt = 120.0,
    baseThrottle = 7,
    currentOutput = 0,
    statusMsg = string.format("Wired Engines: %d found", #engines)
}

local altPID = PID.new(0.6, 0.05, 0.8, -8, 8)
local buttons = {}

local function addButton(x, y, w, h, text, bgBlit, fgBlit, action)
    table.insert(buttons, {
        x = x, y = y, w = w, h = h,
        text = text, bg = bgBlit, fg = fgBlit,
        action = action
    })
end

-- ========================================================
-- 5. GUI 繪製函式
-- ========================================================
local function drawWiredUI()
    local w, h = display.getSize()
    display.setBackgroundColor(colors.black)
    display.clear()

    -- 1. 標題列
    display.setCursorPos(1, 1)
    local titleText = "  WIRED CABLE FLIGHT MASTER  "
    local padL = math.floor((w - #titleText) / 2)
    local padR = w - #titleText - padL
    local fullTitle = string.rep(" ", padL) .. titleText .. string.rep(" ", padR)
    display.blit(fullTitle, string.rep("0", w), string.rep("b", w))

    local currAlt = altiSensor and altiSensor.getHeight() or 0
    local currVspeed = altiSensor and altiSensor.getVerticalSpeed() or 0

    -- 2. 左側高度卡片
    local cardW = math.floor(w / 2) - 2
    for y = 3, 7 do
        display.setCursorPos(2, y)
        display.blit(string.rep(" ", cardW), string.rep("0", cardW), string.rep("8", cardW))
    end
    display.setCursorPos(3, 3)
    display.blit("ALTITUDE (m)", "999999999999", "888888888888")
    
    local altStr = string.format("%6.1f", currAlt)
    display.setCursorPos(3, 5)
    display.blit(altStr, string.rep("0", #altStr), string.rep("8", #altStr))
    display.setCursorPos(3, 6)
    local vspeedStr = string.format("V.SPD: %+5.1f m/s", currVspeed)
    display.blit(vspeedStr, string.rep("4", #vspeedStr), string.rep("8", #vspeedStr))

    -- 3. 右側狀態卡片
    local rightX = cardW + 4
    local rightW = w - rightX
    for y = 3, 7 do
        display.setCursorPos(rightX, y)
        display.blit(string.rep(" ", rightW), string.rep("0", rightW), string.rep("8", rightW))
    end
    display.setCursorPos(rightX + 1, 3)
    display.blit(string.format("TARGET : %5.1fm", state.targetAlt), string.rep("3", rightW - 1), string.rep("8", rightW - 1))
    display.setCursorPos(rightX + 1, 4)
    display.blit(string.format("BASE   : %2d/15", state.baseThrottle), string.rep("3", rightW - 1), string.rep("8", rightW - 1))
    display.setCursorPos(rightX + 1, 5)
    local modeFg = (state.mode == "HOLD_ALT") and "5" or "e"
    display.blit(string.format("ENGINES: %2d node", #engines), string.rep("0", rightW - 1), string.rep("8", rightW - 1))
    display.setCursorPos(rightX + 1, 6)
    display.blit(string.format("OUTPUT : %2d/15", state.currentOutput), string.rep(modeFg, rightW - 1), string.rep("8", rightW - 1))

    -- 4. 推力即時長條圖
    display.setCursorPos(2, 9)
    display.blit("THRUST [", "00000000", string.rep("f", 8))
    local barW = w - 12
    local filled = math.floor((state.currentOutput / 15) * barW)
    local barText = string.rep("|", filled) .. string.rep(".", barW - filled)
    local barFg = string.rep("5", filled) .. string.rep("7", barW - filled)
    local barBg = string.rep("f", barW)
    display.blit(barText, barFg, barBg)
    display.blit("]", "0", "f")

    -- 5. 觸控按鈕
    buttons = {}

    local bY1 = 11
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

    local bY2 = 14
    local bW2 = math.floor((w - 4) / 3)
    addButton(2, bY2, bW2, 2, "BASE +1", "3", "0", function()
        state.baseThrottle = math.min(15, state.baseThrottle + 1)
    end)
    addButton(3 + bW2, bY2, bW2, 2, "BASE -1", "9", "0", function()
        state.baseThrottle = math.max(0, state.baseThrottle - 1)
    end)
    addButton(4 + bW2*2, bY2, bW2, 2, "SCAN/CALIB", "a", "0", function()
        engines = scanWiredEngines()
        state.mode = "CALIBRATING"
    end)

    local bY3 = 17
    local mainBW = math.floor((w - 3) / 2)
    local holdBg = (state.mode == "HOLD_ALT") and "5" or "d"
    addButton(2, bY3, mainBW, 3, " [ HOLD ALT ] ", holdBg, "0", function()
        state.mode = "HOLD_ALT"
        altPID:reset()
        state.statusMsg = "Holding Altitude"
    end)

    local stopBg = (state.mode == "IDLE") and "e" or "c"
    addButton(3 + mainBW, bY3, mainBW, 3, " [ STOP / IDLE ] ", stopBg, "0", function()
        state.mode = "IDLE"
        altPID:reset()
        outputToAllWiredEngines(0)
        state.statusMsg = "Engines Stopped"
    end)

    for _, btn in ipairs(buttons) do
        for dy = 0, btn.h - 1 do
            display.setCursorPos(btn.x, btn.y + dy)
            display.blit(string.rep(" ", btn.w), string.rep(btn.fg, btn.w), string.rep(btn.bg, btn.w))
        end
        local tx = btn.x + math.floor((btn.w - #btn.text) / 2)
        local ty = btn.y + math.floor(btn.h / 2)
        display.setCursorPos(tx, ty)
        display.blit(btn.text, string.rep(btn.fg, #btn.text), string.rep(btn.bg, #btn.text))
    end
end

-- ========================================================
-- 6. 飛控控制迴圈 (20Hz)
-- ========================================================
local function flightControlLoop()
    while true do
        if state.mode == "HOLD_ALT" and altiSensor then
            local currentAlt = altiSensor.getHeight()
            local altError = state.targetAlt - currentAlt
            local pidAdjustment = altPID:update(altError)
            local finalSignal = state.baseThrottle + pidAdjustment
            state.currentOutput = math.max(0, math.min(15, math.floor(finalSignal + 0.5)))
            outputToAllWiredEngines(state.currentOutput)
        elseif state.mode == "CALIBRATING" then
            state.statusMsg = "Calibrating..."
            drawWiredUI()
            local bestSignal = 0
            local minVspeed = 999

            for sig = 0, 15 do
                state.currentOutput = sig
                outputToAllWiredEngines(sig)
                state.statusMsg = string.format("Testing [%2d/15]", sig)
                drawWiredUI()
                sleep(1.0)

                local vspeed = altiSensor and altiSensor.getVerticalSpeed() or 0
                if math.abs(vspeed) < minVspeed then
                    minVspeed = math.abs(vspeed)
                    bestSignal = sig
                end
                if vspeed > 0.3 and sig > 0 then break end
            end

            state.baseThrottle = bestSignal
            state.currentOutput = bestSignal
            outputToAllWiredEngines(bestSignal)
            state.mode = "HOLD_ALT"
            state.statusMsg = string.format("Calib Done: %d", bestSignal)
        elseif state.mode == "IDLE" then
            state.currentOutput = 0
            outputToAllWiredEngines(0)
        end
        sleep(0.05)
    end
end

-- ========================================================
-- 7. 畫面與事件監聽
-- ========================================================
local function renderLoop()
    while true do
        drawWiredUI()
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
                    drawWiredUI()
                    break
                end
            end
        end
    end
end

parallel.waitForAll(flightControlLoop, renderLoop, touchEventLoop)
