--[[
    Create: Avionics & CC: Tweaked + DirectGPU
    DirectGPU High-Resolution Airship Flight Display & Altitude Controller
    
    使用 DirectGPU (24-bit True RGB + 高解析度渲染 + 觸控互動):
    - 高解析度圓形高度儀表 (Altimeter Gauge)
    - 垂直升降速度指針 (Variometer)
    - 平滑推力量表與全觸控按鈕
    - 支援 True Color (全彩 RGB)
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
    error("DirectGPU 未能自動檢測到相鄰 Monitor 螢幕！")
end

local dispInfo = gpu.getDisplayInfo(displayId)
local screenW = dispInfo.pixelWidth or 320
local screenH = dispInfo.pixelHeight or 240

print(string.format("[DirectGPU] Display Initialized: ID=%d, Res=%dx%d", displayId, screenW, screenH))

-- 開啟無線網路 (Rednet)
local modemSide = nil
for _, side in ipairs({"left", "right", "top", "back", "bottom", "front"}) do
    if peripheral.getType(side) == "modem" then
        modemSide = side
        break
    end
end
if modemSide then
    rednet.open(modemSide)
    print(string.format("[Radio] Rednet opened on [%s]", modemSide))
end

local altiSensor = peripheral.find("altitude_sensor")
if altiSensor then
    print("[Sensor] Connected to Altitude Sensor")
end

-- ========================================================
-- 3. 飛控狀態與按鈕熱區
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

local function addButton(x, y, w, h, text, bgColor, fgColor, action)
    table.insert(buttons, {
        x = x, y = y, w = w, h = h,
        text = text, bg = bgColor, fg = fgColor,
        action = action
    })
end

local function broadcastThrottle(signal)
    signal = math.max(0, math.min(15, math.floor(signal + 0.5)))
    state.currentOutput = signal
    rednet.broadcast({ cmd = "THROTTLE", signal = signal }, "flight_control")
end

-- ========================================================
-- 4. DirectGPU 渲染介面 (High-Res 2D Graphics)
-- ========================================================
local function drawDirectGPU_UI()
    -- 1. 清空背景 (深深空藍灰)
    gpu.clear(displayId, 15, 20, 30)

    -- 2. 頂部儀表板標題列
    gpu.fillRect(displayId, 0, 0, screenW, 30, 25, 40, 65)
    gpu.drawText(displayId, "AIRSHIP AVIONICS - DIRECT GPU", 15, 8, 240, 245, 255, "Arial", 16, "bold")

    -- 讀取感測數據
    local currAlt = altiSensor and altiSensor.getHeight() or 0
    local currVspeed = altiSensor and altiSensor.getVerticalSpeed() or 0

    -- 3. 左側高度圓形儀表 (Round Altimeter Gauge)
    local gaugeCX, gaugeCY, gaugeR = 70, 95, 45
    gpu.drawCircle(displayId, gaugeCX, gaugeCY, gaugeR, 35, 45, 60, true)   -- 外圈底盤
    gpu.drawCircle(displayId, gaugeCX, gaugeCY, gaugeR, 100, 180, 255, false) -- 外光圈邊框
    
    -- 儀表中心高度數字
    gpu.drawText(displayId, string.format("%.0f", currAlt), gaugeCX - 18, gaugeCY - 10, 255, 255, 255, "Arial", 18, "bold")
    gpu.drawText(displayId, "ALT (M)", gaugeCX - 16, gaugeCY + 12, 160, 200, 230, "Arial", 10, "plain")

    -- 4. 儀表數據資訊卡 (Data Cards)
    local cardX = 135
    gpu.fillRect(displayId, cardX, 40, screenW - cardX - 10, 80, 25, 30, 45)
    
    gpu.drawText(displayId, string.format("TARGET ALT : %.1f m", state.targetAlt), cardX + 10, 48, 80, 220, 255, "Arial", 12, "bold")
    gpu.drawText(displayId, string.format("V. SPEED   : %+.2f m/s", currVspeed), cardX + 10, 66, 255, 200, 80, "Arial", 12, "bold")
    gpu.drawText(displayId, string.format("BASE THRUST: %d / 15", state.baseThrottle), cardX + 10, 84, 180, 255, 120, "Arial", 12, "bold")
    
    local modeColor = (state.mode == "HOLD_ALT") and {80, 255, 120} or {255, 80, 80}
    gpu.drawText(displayId, string.format("MODE: %s", state.mode), cardX + 10, 102, modeColor[1], modeColor[2], modeColor[3], "Arial", 12, "bold")

    -- 5. 推力即時進度條 (Thrust Bar)
    local barY = 130
    gpu.drawText(displayId, "OUTPUT THRUST", 15, barY + 2, 200, 210, 220, "Arial", 11, "bold")
    
    local barX = 120
    local barW = screenW - barX - 15
    gpu.fillRect(displayId, barX, barY, barW, 14, 40, 45, 55)
    
    local filledW = math.floor((state.currentOutput / 15) * barW)
    if filledW > 0 then
        gpu.fillRect(displayId, barX, barY, filledW, 14, 50, 200, 90) -- 綠色推力條
    end

    -- 6. 觸控按鈕區 (Touch Buttons)
    buttons = {} -- 清空熱區

    local btnY1 = 155
    local btnW1 = math.floor((screenW - 40) / 4)
    local btnH1 = 28

    -- 第一排：高度加減按鈕
    addButton(10, btnY1, btnW1, btnH1, "+10m", {40, 120, 60}, {255, 255, 255}, function()
        state.targetAlt = state.targetAlt + 10
    end)
    addButton(15 + btnW1, btnY1, btnW1, btnH1, "+1m", {60, 150, 80}, {255, 255, 255}, function()
        state.targetAlt = state.targetAlt + 1
    end)
    addButton(20 + btnW1*2, btnY1, btnW1, btnH1, "-1m", {180, 110, 40}, {255, 255, 255}, function()
        state.targetAlt = math.max(0, state.targetAlt - 1)
    end)
    addButton(25 + btnW1*3, btnY1, btnW1, btnH1, "-10m", {180, 60, 50}, {255, 255, 255}, function()
        state.targetAlt = math.max(0, state.targetAlt - 10)
    end)

    -- 第二排：基準推力與校準
    local btnY2 = 190
    local btnW2 = math.floor((screenW - 35) / 3)
    addButton(10, btnY2, btnW2, btnH1, "BASE +1", {40, 90, 140}, {255, 255, 255}, function()
        state.baseThrottle = math.min(15, state.baseThrottle + 1)
    end)
    addButton(15 + btnW2, btnY2, btnW2, btnH1, "BASE -1", {50, 70, 120}, {255, 255, 255}, function()
        state.baseThrottle = math.max(0, state.baseThrottle - 1)
    end)
    addButton(20 + btnW2*2, btnY2, btnW2, btnH1, "CALIBRATE", {120, 60, 160}, {255, 255, 255}, function()
        state.mode = "CALIBRATING"
    end)

    -- 第三排：核心主控開關 (HOLD ALT / STOP)
    local btnY3 = 225
    local mainBtnW = math.floor((screenW - 30) / 2)
    local mainBtnH = 35

    local holdBg = (state.mode == "HOLD_ALT") and {30, 180, 80} or {50, 80, 60}
    addButton(10, btnY3, mainBtnW, mainBtnH, "HOLD ALT", holdBg, {255, 255, 255}, function()
        state.mode = "HOLD_ALT"
        altPID:reset()
        state.statusMsg = "Holding Target Altitude"
    end)

    local stopBg = (state.mode == "IDLE") and {180, 40, 40} or {90, 40, 40}
    addButton(15 + mainBtnW, btnY3, mainBtnW, mainBtnH, "STOP / IDLE", stopBg, {255, 255, 255}, function()
        state.mode = "IDLE"
        altPID:reset()
        broadcastThrottle(0)
        state.statusMsg = "Engines Stopped"
    end)

    -- 繪製所有按鈕實體
    for _, btn in ipairs(buttons) do
        gpu.fillRect(displayId, btn.x, btn.y, btn.w, btn.h, btn.bg[1], btn.bg[2], btn.bg[3])
        -- 文字水平居中計算
        local textLen = #btn.text * 7
        local tx = btn.x + math.max(4, math.floor((btn.w - textLen) / 2))
        local ty = btn.y + math.floor((btn.h - 12) / 2)
        gpu.drawText(displayId, btn.text, tx, ty, btn.fg[1], btn.fg[2], btn.fg[3], "Arial", 12, "bold")
    end

    -- 7. 提交畫面至 DirectGPU 顯示器 (Push Frame)
    gpu.updateDisplay(displayId)
end

-- ========================================================
-- 5. 飛控閉環控制迴圈 (20Hz)
-- ========================================================
local function flightControlLoop()
    while true do
        if state.mode == "HOLD_ALT" and altiSensor then
            local currentAlt = altiSensor.getHeight()
            local altError = state.targetAlt - currentAlt
            local pidAdjustment = altPID:update(altError)
            local finalSignal = state.baseThrottle + pidAdjustment
            broadcastThrottle(finalSignal)
        elseif state.mode == "CALIBRATING" then
            state.statusMsg = "Calibrating thrust..."
            drawDirectGPU_UI()
            local bestSignal = 0
            local minVspeed = 999

            for sig = 0, 15 do
                broadcastThrottle(sig)
                state.statusMsg = string.format("Testing throttle [%2d/15]...", sig)
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
-- 6. 螢幕繪製與觸控監聽
-- ========================================================
local function renderLoop()
    while true do
        drawDirectGPU_UI()
        sleep(0.05) -- 20 FPS 流暢高畫質更新
    end
end

local function touchEventLoop()
    while true do
        local event, id, x, y = os.pullEvent()
        -- 監聽 DirectGPU 螢幕觸控與點擊
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
