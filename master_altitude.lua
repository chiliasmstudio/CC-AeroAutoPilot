--[[
    Create: Avionics & CC: Tweaked
    Master Touchscreen / Monitor Altitude Controller (觸控螢幕 GUI 版)
    
    特性:
    - 全觸控/滑鼠 GUI 介面，支援外接 Advanced Monitor 或 電腦自帶螢幕！
    - 即時高度、垂直速度儀表盤與推力量表。
    - 觸控按鈕:
        [ALT +10] [ALT +1]   |   [ALT -10] [ALT -1]
        [BASE +1] [BASE -1]  |   [CALIBRATE]
        [HOLD ALT] [STOP / IDLE]
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
-- 2. 硬體與螢幕初始化 (Display Setup)
-- ========================================================
-- 優先尋找外接 Monitor，若無則使用本機 Term
local display = peripheral.find("monitor") or term.current()
local isColor = display.isColor and display.isColor()

if peripheral.find("monitor") then
    display.setTextScale(1)
end

-- 開啟無線 Modem
local modemSide = nil
for _, side in ipairs({"left", "right", "top", "back", "bottom", "front"}) do
    if peripheral.getType(side) == "modem" then
        modemSide = side
        break
    end
end

if modemSide then
    rednet.open(modemSide)
end

local altiSensor = peripheral.find("altitude_sensor")

-- ========================================================
-- 3. 飛控狀態與按鈕定義
-- ========================================================
local state = {
    mode = "IDLE",       -- "IDLE", "HOLD_ALT", "CALIBRATING"
    targetAlt = 120.0,
    baseThrottle = 7,
    currentOutput = 0,
    statusMsg = "System Ready"
}

local altPID = PID.new(0.6, 0.05, 0.8, -8, 8)
local buttons = {}

local function addButton(x, y, w, h, text, bg, fg, action)
    table.insert(buttons, {
        x = x, y = y, w = w, h = h,
        text = text, bg = bg, fg = fg,
        action = action
    })
end

-- 廣播推力訊號給所有烏龜
local function broadcastThrottle(signal)
    signal = math.max(0, math.min(15, math.floor(signal + 0.5)))
    state.currentOutput = signal
    rednet.broadcast({ cmd = "THROTTLE", signal = signal }, "flight_control")
end

-- ========================================================
-- 4. GUI 繪製函式 (Render Engine)
-- ========================================================
local function drawUI()
    local w, h = display.getSize()
    display.setBackgroundColor(colors.black)
    display.clear()

    -- 標題列
    display.setCursorPos(1, 1)
    display.setBackgroundColor(colors.blue)
    display.setTextColor(colors.white)
    display.clearLine()
    local title = " AIRSHIP ALTITUDE MASTER "
    display.setCursorPos(math.floor((w - #title) / 2) + 1, 1)
    display.write(title)

    -- 讀取感測數據
    local currAlt = altiSensor and altiSensor.getHeight() or 0
    local currVspeed = altiSensor and altiSensor.getVerticalSpeed() or 0

    -- 資訊看板區
    display.setBackgroundColor(colors.gray)
    for y = 3, 7 do
        display.setCursorPos(2, y)
        display.write(string.rep(" ", w - 2))
    end

    display.setTextColor(colors.yellow)
    display.setCursorPos(3, 3)
    display.write(string.format("Current Alt: %6.1f m", currAlt))
    display.setCursorPos(math.floor(w / 2) + 2, 3)
    display.write(string.format("V.Speed: %+5.1f m/s", currVspeed))

    display.setTextColor(colors.cyan)
    display.setCursorPos(3, 4)
    display.write(string.format("Target  Alt: %6.1f m", state.targetAlt))
    display.setCursorPos(math.floor(w / 2) + 2, 4)
    display.write(string.format("Base Thrust: %2d/15", state.baseThrottle))

    display.setTextColor(colors.white)
    display.setCursorPos(3, 5)
    display.write(string.format("Active Mode: %-10s", state.mode))
    display.setCursorPos(math.floor(w / 2) + 2, 5)
    display.write(string.format("Output Sig : %2d/15", state.currentOutput))

    -- 狀態訊息提示
    display.setTextColor(colors.lightGray)
    display.setCursorPos(3, 6)
    display.write(string.format("Status: %s", state.statusMsg))

    -- 繪製推力長條圖 (Gauge Bar)
    display.setCursorPos(3, 7)
    display.setTextColor(colors.white)
    display.write("Thrust: [")
    local barWidth = w - 15
    local filled = math.floor((state.currentOutput / 15) * barWidth)
    display.setTextColor(colors.lime)
    display.write(string.rep("=", filled))
    display.setTextColor(colors.lightGray)
    display.write(string.rep("-", barWidth - filled))
    display.setTextColor(colors.white)
    display.write("]")

    -- 繪製所有觸控按鈕
    buttons = {} -- 重置按鈕熱區

    -- 第一排：高度微調
    addButton(2, 9, 8, 2, "ALT +10", colors.green, colors.white, function()
        state.targetAlt = state.targetAlt + 10
    end)
    addButton(11, 9, 8, 2, "ALT +1", colors.lime, colors.black, function()
        state.targetAlt = state.targetAlt + 1
    end)
    addButton(20, 9, 8, 2, "ALT -1", colors.orange, colors.black, function()
        state.targetAlt = math.max(0, state.targetAlt - 1)
    end)
    addButton(29, 9, 8, 2, "ALT -10", colors.red, colors.white, function()
        state.targetAlt = math.max(0, state.targetAlt - 10)
    end)

    -- 第二排：基準推力微調 & 自動校準
    addButton(2, 12, 8, 2, "BASE +1", colors.cyan, colors.black, function()
        state.baseThrottle = math.min(15, state.baseThrottle + 1)
    end)
    addButton(11, 12, 8, 2, "BASE -1", colors.lightBlue, colors.black, function()
        state.baseThrottle = math.max(0, state.baseThrottle - 1)
    end)
    addButton(20, 12, 17, 2, "AUTO CALIBRATE", colors.purple, colors.white, function()
        state.mode = "CALIBRATING"
    end)

    -- 第三排：啟動與停機
    local holdColor = (state.mode == "HOLD_ALT") and colors.green or colors.gray
    addButton(2, 15, 17, 3, " [ HOLD ALT ] ", holdColor, colors.white, function()
        state.mode = "HOLD_ALT"
        altPID:reset()
        state.statusMsg = "Holding Target Altitude"
    end)

    local stopColor = (state.mode == "IDLE") and colors.red or colors.brown
    addButton(20, 15, 17, 3, " [ STOP / IDLE ] ", stopColor, colors.white, function()
        state.mode = "IDLE"
        altPID:reset()
        broadcastThrottle(0)
        state.statusMsg = "Engines Stopped"
    end)

    -- 渲染按鈕實體
    for _, btn in ipairs(buttons) do
        display.setBackgroundColor(btn.bg)
        display.setTextColor(btn.fg)
        for dy = 0, btn.h - 1 do
            display.setCursorPos(btn.x, btn.y + dy)
            display.write(string.rep(" ", btn.w))
        end
        -- 文字居中
        local tx = btn.x + math.floor((btn.w - #btn.text) / 2)
        local ty = btn.y + math.floor(btn.h / 2)
        display.setCursorPos(tx, ty)
        display.write(btn.text)
    end
end

-- ========================================================
-- 5. 高度控制主迴圈 (20Hz)
-- ========================================================
local function altitudeControlLoop()
    while true do
        if state.mode == "HOLD_ALT" and altiSensor then
            local currentAlt = altiSensor.getHeight()
            local altError = state.targetAlt - currentAlt
            local pidAdjustment = altPID:update(altError)
            local finalSignal = state.baseThrottle + pidAdjustment
            broadcastThrottle(finalSignal)
        elseif state.mode == "CALIBRATING" then
            state.statusMsg = "Calibrating thrust..."
            drawUI()
            local bestSignal = 0
            local minVspeed = 999

            for sig = 0, 15 do
                broadcastThrottle(sig)
                state.statusMsg = string.format("Testing throttle [%2d/15]...", sig)
                drawUI()
                sleep(1.0)

                local vspeed = altiSensor and altiSensor.getVerticalSpeed() or 0
                if math.abs(vspeed) < minVspeed then
                    minVspeed = math.abs(vspeed)
                    bestSignal = sig
                end
                if vspeed > 0.3 and sig > 0 then break end
            end

            state.baseThrottle = bestSignal
            broadcastThrottle(bestSignal)
            state.mode = "HOLD_ALT"
            state.statusMsg = string.format("Calib Done! Base: %d", bestSignal)
        elseif state.mode == "IDLE" then
            broadcastThrottle(0)
        end
        sleep(0.05)
    end
end

-- ========================================================
-- 6. 螢幕更新與觸控事件監聽 (Touch Interaction)
-- ========================================================
local function uiLoop()
    while true do
        drawUI()
        sleep(0.1) -- 10 FPS 介面刷新
    end
end

local function touchEventLoop()
    while true do
        local event, side, x, y = os.pullEvent()
        -- 同時支援外接螢幕觸控 (monitor_touch) 與電腦本機點擊 (mouse_click)
        if event == "monitor_touch" or event == "mouse_click" then
            local clickX, clickY = x, y
            if event == "mouse_click" then
                clickX, clickY = side, x -- mouse_click 參數為 (button, x, y)
            end

            -- 檢查點擊是否命中任何按鈕
            for _, btn in ipairs(buttons) do
                if clickX >= btn.x and clickX < btn.x + btn.w and
                   clickY >= btn.y and clickY < btn.y + btn.h then
                    btn.action()
                    drawUI()
                    break
                end
            end
        end
    end
end

-- 三者平行運算
parallel.waitForAll(altitudeControlLoop, uiLoop, touchEventLoop)
