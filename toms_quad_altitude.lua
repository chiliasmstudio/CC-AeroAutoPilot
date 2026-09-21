--[[
    Create: Avionics & CC: Tweaked + Tom's Peripherals
    Tom's Peripherals Quad-Engine Flight Controller - A350 ECAM Edition (Tom's Peripherals 四軸有線 A350 ECAM 高畫質飛控大腦)
    Version: v3.5.0
    
    支援規格:
    - 專為 Tom's Peripherals 的 GPU 方塊 (tm_gpu / gpu) 與 Bitmap Monitors 打造。
    - 全面啟用 64x64 高畫質解析度 (gpu.setSize(64))。
    - 完整 32-bit ARGB (0xAARRGGBB) 顏色不透明度校正 (Alpha = 0xFF)，解決文字與刻度隱形問題。
    - A350 ECAM 全彩航空儀表：採用 GPU 平滑弧線 (lineS)、多色指針與數位讀數方框。
    - 穩健起飛偵測 (抗雜訊)：持續平穩爬坡加力，離地判定 (高度上升 >= 0.6m 或 垂直速度 >= 0.18m/s)。
    - 升力校準至 200m：平穩爬升至 200m，懸停自穩 5 秒自動鎖定並記錄懸停基準推力！
    - 自動平衡關閉 (純集體升力混控)，Gimbal 陀螺儀 Pitch / Roll 姿態完整即時呈現。
    - 支援 tm_monitor_touch 觸控點擊與全螢幕自動解析度適配。
--]]

local VERSION = "v3.5.0"
print(string.format("[TOMS-PERIPHERALS] Loading Avionics Master %s...", VERSION))

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
-- 2. Tom's Peripherals GPU 初始化與 32-bit ARGB 安全繪圖模組
-- ========================================================
local gpu = peripheral.find("tm_gpu") or peripheral.find("gpu")
if not gpu then
    error("未找到 Tom's Peripherals GPU 設備！請確保電腦相鄰放置了 GPU 方塊並連接 Bitmap Monitors。")
end

-- 啟用 64x64 高畫質點陣螢幕解析度
pcall(function()
    if gpu.setSize then
        gpu.setSize(64)
    end
end)
gpu.refreshSize()
sleep(0.05)

local screenW, screenH = 320, 240

local function updateScreenSize()
    local ok, w, h = pcall(function() return gpu.getSize() end)
    if ok and w and h and w > 0 and h > 0 then
        screenW, screenH = w, h
    end
end

updateScreenSize()

-- 32-bit ARGB 顏色轉換 (強制注入 100% Alpha 不透明度 0xFF000000)
local function toARGB(col)
    if type(col) ~= "number" then return 0xFFFFFFFF end
    if col <= 0x00FFFFFF then
        return col + 0xFF000000
    end
    return col
end

local function clampX(x)
    return math.max(1, math.min(screenW, math.floor(x + 0.5)))
end

local function clampY(y)
    return math.max(1, math.min(screenH, math.floor(y + 0.5)))
end

local function safeFill(color)
    local argb = toARGB(color)
    pcall(function()
        if gpu.fill then
            gpu.fill(argb)
        else
            gpu.filledRectangle(1, 1, screenW, screenH, argb)
        end
    end)
end

local function safeFilledRectangle(x, y, w, h, color)
    if w <= 0 or h <= 0 then return end
    x = clampX(x)
    y = clampY(y)
    w = math.max(1, math.min(screenW - x + 1, math.floor(w + 0.5)))
    h = math.max(1, math.min(screenH - y + 1, math.floor(h + 0.5)))
    local argb = toARGB(color)
    pcall(function() gpu.filledRectangle(x, y, w, h, argb) end)
end

local function safeRectangle(x, y, w, h, color)
    if w <= 0 or h <= 0 then return end
    x = clampX(x)
    y = clampY(y)
    w = math.max(1, math.min(screenW - x + 1, math.floor(w + 0.5)))
    h = math.max(1, math.min(screenH - y + 1, math.floor(h + 0.5)))
    local argb = toARGB(color)
    pcall(function() gpu.rectangle(x, y, w, h, argb) end)
end

local function safeLine(x1, y1, x2, y2, color)
    x1 = clampX(x1)
    y1 = clampY(y1)
    x2 = clampX(x2)
    y2 = clampY(y2)
    local argb = toARGB(color)
    pcall(function() gpu.line(x1, y1, x2, y2, argb) end)
end

local function safeLineS(x1, y1, x2, y2, color)
    x1 = clampX(x1)
    y1 = clampY(y1)
    x2 = clampX(x2)
    y2 = clampY(y2)
    local argb = toARGB(color)
    pcall(function()
        if gpu.lineS then
            gpu.lineS(x1, y1, x2, y2, argb)
        else
            gpu.line(x1, y1, x2, y2, argb)
        end
    end)
end

local function safeDrawText(x, y, text, textColor, bgColor, size)
    x = clampX(x)
    y = clampY(y)
    text = tostring(text or "")
    local fgARGB = toARGB(textColor or 0xFFFFFF)
    local bgARGB = bgColor and toARGB(bgColor) or nil
    pcall(function()
        if gpu.drawTextSmart then
            gpu.drawTextSmart(x, y, text, fgARGB, bgARGB, false, size or 1)
        elseif gpu.drawText then
            gpu.drawText(x, y, text, fgARGB, bgARGB, size or 1)
        end
    end)
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

-- GPU 幾何輔助繪圖 (圓形、平滑弧線)
local function drawArc(cx, cy, r, startDeg, endDeg, stepDeg, color)
    local prevX, prevY = nil, nil
    local step = (startDeg > endDeg) and -math.abs(stepDeg or 5) or math.abs(stepDeg or 5)
    for deg = startDeg, endDeg, step do
        local rad = math.rad(deg)
        local px = cx + r * math.cos(rad)
        local py = cy - r * math.sin(rad)
        if prevX and prevY then
            safeLineS(prevX, prevY, px, py, color)
        end
        prevX, prevY = px, py
    end
end

local function drawCircle(cx, cy, r, color, filled)
    if filled then
        for dy = -r, r do
            local dx = math.floor(math.sqrt(math.max(0, r * r - dy * dy)) + 0.5)
            safeLine(cx - dx, cy + dy, cx + dx, cy + dy, color)
        end
    else
        drawArc(cx, cy, r, 0, 360, 10, color)
    end
end

-- ========================================================
-- 3. 四角烏龜引擎節點掃描 (FL, FR, BL, BR)
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
-- 4. 紅石輸出（純類比輸出，排除正面，其餘 5 面）與集體升力混控
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

    -- 平衡關閉：四軸純集體升力
    deltaPitch = deltaPitch or 0
    deltaRoll  = deltaRoll or 0

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
    targetAlt = 200.0,        -- 預設目標高度 200m
    virtualAlt = 100.0,       -- 虛擬平滑軌跡高度
    baseThrottle = 1.0,       -- 支援浮點推力
    autoLevel = false,        -- 平衡關閉 (純集體升力)
    statusMsg = "System Ready",
    stableTimer = 0,          -- 穩定計時器 (秒)
    calibStartAlt = 0,        -- 校正啟動高度
    calibTargetAlt = 200.0,   -- 校正目標固定設定為 200m
    calibPhase = "GROUND_SEARCH", -- "GROUND_SEARCH", "CLIMB_TO_200", "STABILIZE_200"
    calibThrottle = 0.0,      -- 校正微調油門
    rampRate = 0.0001         -- 適應性加力速率
}

-- PID 控制器：高度修正限幅在 [-1.5, +1.5]
local altPID = PID.new(0.5, 0.02, 0.6, -1.5, 1.5)

local buttons = {}

local function addButton(x, y, w, h, text, bgCol, fgCol, action)
    table.insert(buttons, {
        x = x, y = y, w = w, h = h,
        text = text, bg = bgCol, fg = fgCol,
        action = action
    })
end

-- ========================================================
-- 6. Airbus A350 ECAM 引擎儀表全彩渲染函式 (Tom's Peripherals GPU)
-- ========================================================
local function drawA350EngineDial(cx, cy, r, val, maxVal, label, sig)
    -- 1. 儀表上方名稱標籤 (大字體)
    local labelSize = (screenW >= 280) and 2 or 1
    safeDrawText(cx - #label * 4 * labelSize, cy - r - 16, label, 0xC8E6FF, nil, labelSize)

    -- 2. 刻度弧 (210° ~ -30°)
    drawArc(cx, cy, r, 210, -30, 8, 0x8CA0B4)
    drawArc(cx, cy, r + 1, 210, -30, 8, 0x506478)

    -- 主刻度線 (0%, 50%, 100%)
    local ticks = {210, 90, -30}
    for _, deg in ipairs(ticks) do
        local rad = math.rad(deg)
        local t1x = cx + (r - 3) * math.cos(rad)
        local t1y = cy - (r - 3) * math.sin(rad)
        local t2x = cx + (r + 5) * math.cos(rad)
        local t2y = cy - (r + 5) * math.sin(rad)
        safeLine(t1x, t1y, t2x, t2y, 0xB4C8DC)
    end

    -- 3. 高推力紅線區 (13.0 ~ 15.0，對應 10° ~ -30°)
    drawArc(cx, cy, r, 10, -30, 5, 0xFF3737)
    drawArc(cx, cy, r + 1, 10, -30, 5, 0xFF3737)

    -- 紅線極限標記線
    local rTickRad = math.rad(-18)
    local rx1 = cx + (r - 3) * math.cos(rTickRad)
    local ry1 = cy - (r - 3) * math.sin(rTickRad)
    local rx2 = cx + (r + 7) * math.cos(rTickRad)
    local ry2 = cy - (r + 7) * math.sin(rTickRad)
    safeLine(rx1, ry1, rx2, ry2, 0xFF3232)

    -- 4. 綠色指針
    local ratio = math.min(1.0, math.max(0.0, val / (maxVal or 15.0)))
    local needleDeg = 210 - ratio * 240
    local nRad = math.rad(needleDeg)
    local nx = cx + (r - 2) * math.cos(nRad)
    local ny = cy - (r - 2) * math.sin(nRad)

    safeLineS(cx, cy, nx, ny, 0x32FF64)
    drawCircle(cx, cy, math.max(2, math.floor(r * 0.12)), 0xD2DCF0, true)

    -- 5. 底部數位連續推力讀數方框
    local boxW = math.max(40, math.floor(r * 1.5))
    local boxH = math.max(16, math.floor(r * 0.65))
    local boxX = cx - math.floor(boxW / 2)
    local boxY = cy + math.floor(r * 0.28)

    safeFilledRectangle(boxX, boxY, boxW, boxH, 0x0C121C)
    safeRectangle(boxX, boxY, boxW, boxH, 0x2896C8)

    local valStr = string.format("%4.1f", val)
    safeDrawText(boxX + 4, boxY + 2, valStr, 0x46FF78, nil, 1)

    -- 6. 離散紅石訊號 (如 2/15)
    local sigStr = string.format("%d/15", sig or 0)
    safeDrawText(cx - #sigStr * 3, boxY + boxH + 3, sigStr, 0x96BEE1, nil, 1)
end

-- ========================================================
-- 7. Tom's Peripherals GPU 全螢幕大字體響應式介面
-- ========================================================
local function drawTomsUI()
    updateScreenSize()

    -- 1. 背景清除 (深航空藍黑)
    safeFill(0x0F141E)

    -- 2. 頂部標題列 (1-indexed 邊界保護)
    local headerH = math.max(24, math.floor(screenH * 0.08))
    safeFilledRectangle(1, 1, screenW, headerH, 0x192841)
    local titleSize = (screenW >= 300) and 2 or 1
    safeDrawText(12, math.floor((headerH - 8 * titleSize) / 2) + 1, string.format("QUAD-ENGINE AVIONICS - TOMS GPU %s", VERSION), 0xF0F5FF, nil, titleSize)

    local currAlt = altiSensor and altiSensor.getHeight() or 0
    local currVspeed = altiSensor and altiSensor.getVerticalSpeed() or 0
    local currPitch, currRoll = getGimbalData()

    -- 3. 儀表主區塊垂直尺寸分配
    local instY = headerH + 6
    local instH = math.floor((screenH - headerH) * 0.52)
    local leftW = math.max(130, math.floor(screenW * 0.35))

    -- [左側 PFD / 高度與飛行狀態卡片]
    local pfdX = 6
    safeFilledRectangle(pfdX, instY, leftW, instH, 0x141A26)
    safeRectangle(pfdX, instY, leftW, instH, 0x283850)
    safeDrawText(pfdX + 8, instY + 6, "PRIMARY FLIGHT DISPLAY", 0xAAD2E6, nil, 1)

    -- PFD 圓形高度儀表
    local gR = math.min(32, math.floor(instH * 0.28))
    local gCX = pfdX + math.floor(leftW * 0.26)
    local gCY = instY + math.floor(instH * 0.54)

    drawCircle(gCX, gCY, gR, 0x1E2837, true)
    drawCircle(gCX, gCY, gR, 0x50AAF0, false)
    local altText = string.format("%.0f", currAlt)
    local altSize = (gR >= 26) and 2 or 1
    safeDrawText(gCX - math.floor(#altText * 3.5 * altSize), gCY - 4 * altSize, altText, 0xFFFFFF, nil, altSize)
    safeDrawText(gCX - 12, gCY + math.floor(gR * 0.36), "ALT(M)", 0xA0CDF0, nil, 1)

    -- 狀態文字讀數 (支援大字體)
    local textX = pfdX + math.floor(leftW * 0.52)
    local rowSpacing = math.floor((instH - 24) / 5)
    local textSize = (instH >= 120) and 2 or 1

    safeDrawText(textX, instY + 16, string.format("TGT: %.0fm", state.targetAlt), 0x50E6FF, nil, textSize)
    safeDrawText(textX, instY + 16 + rowSpacing, string.format("V.S: %+.2f", currVspeed), 0xFFCD4B, nil, textSize)
    
    local mCol = (state.mode == "HOLD_ALT") and 0x50FF78 or (state.mode == "CALIBRATING" and 0x50C8FF or 0xFF5050)
    safeDrawText(textX, instY + 16 + rowSpacing * 2, string.format("MODE: %s", state.mode:sub(1,7)), mCol, nil, textSize)

    -- 姿態與陀螺儀讀數
    safeDrawText(textX, instY + 16 + rowSpacing * 3, string.format("P:%+4.1f R:%+4.1f", currPitch, currRoll), 0xB4DCFF, nil, 1)
    if gimbalAvailable then
        safeDrawText(textX, instY + 16 + rowSpacing * 4, "GIMBAL: ACTIVE", 0x50FF78, nil, 1)
    else
        safeDrawText(textX, instY + 16 + rowSpacing * 4, "GIMBAL: NO SENSOR", 0xFF3C3C, nil, 1)
    end

    -- [右側 A350 ECAM 四軸引擎儀表區]
    local ecamX = pfdX + leftW + 6
    local ecamW = screenW - ecamX - 6
    safeFilledRectangle(ecamX, instY, ecamW, instH, 0x121822)
    safeRectangle(ecamX, instY, ecamW, instH, 0x283850)

    -- 渲染 4 具 A350 ECAM 引擎儀表 (FL, FR, BL, BR)
    local slotW = math.floor((ecamW - 8) / 4)
    local slots = {"FL", "FR", "BL", "BR"}

    local dialR = math.min(math.floor(slotW * 0.35), math.floor(instH * 0.30))
    local engCenterY = instY + math.floor(instH * 0.44)

    for i, slot in ipairs(slots) do
        local engCenterX = ecamX + 4 + math.floor((i - 0.5) * slotW)
        local vVal = virtualOutputs[slot] or 0.0
        local sig  = engineOutputs[slot] or 0
        local node = engines[slot]
        local lbl  = node and (node.label or node.name or slot) or slot
        if #lbl > 5 then lbl = lbl:sub(1, 5) end

        drawA350EngineDial(engCenterX, engCenterY, dialR, vVal, 15.0, lbl, sig)
    end

    -- 4. 狀態訊息橫幅
    local statY = instY + instH + 6
    local statH = math.max(18, math.floor(screenH * 0.06))
    safeFilledRectangle(pfdX, statY, screenW - 12, statH, 0x1E2432)
    safeRectangle(pfdX, statY, screenW - 12, statH, 0x3C4B64)
    safeDrawText(pfdX + 8, statY + math.floor((statH - 8) / 2), string.format("STATUS: %s", state.statusMsg), 0x50E6FF, nil, 1)

    -- 5. 觸控按鈕區 (響應式下半部排版)
    buttons = {}
    local btnAreaY = statY + statH + 6
    local btnAreaH = screenH - btnAreaY - 6
    local rowH = math.max(16, math.floor((btnAreaH - 12) / 3))

    -- Row 1: 目標高度調整 (+10m, +1m, -1m, -10m, SET CURR)
    local bY1 = btnAreaY
    local bW1 = math.floor((screenW - 12 - 16) / 5)

    addButton(pfdX, bY1, bW1, rowH, "+10m", 0x1B5E20, 0xE8F5E9, function()
        state.targetAlt = state.targetAlt + 10
    end)
    addButton(pfdX + bW1 + 4, bY1, bW1, rowH, "+1m", 0x2E7D32, 0xE8F5E9, function()
        state.targetAlt = state.targetAlt + 1
    end)
    addButton(pfdX + (bW1 + 4) * 2, bY1, bW1, rowH, "-1m", 0xE65100, 0xFFF3E0, function()
        state.targetAlt = math.max(0, state.targetAlt - 1)
    end)
    addButton(pfdX + (bW1 + 4) * 3, bY1, bW1, rowH, "-10m", 0xC62828, 0xFFEBEE, function()
        state.targetAlt = math.max(0, state.targetAlt - 10)
    end)
    addButton(pfdX + (bW1 + 4) * 4, bY1, bW1, rowH, "SET CURR", 0x00838F, 0xE0F7FA, function()
        local cur = altiSensor and altiSensor.getHeight() or 0
        state.targetAlt = math.floor(cur + 0.5)
        state.virtualAlt = cur
        state.statusMsg = string.format("Target locked to current: %.0fm", state.targetAlt)
    end)

    -- Row 2: 基準油門與校準 (BASE +, BASE -, RE-SCAN, CALIBRATE)
    local bY2 = bY1 + rowH + 6
    local bW2 = math.floor((screenW - 12 - 12) / 4)

    addButton(pfdX, bY2, bW2, rowH, "BASE +", 0x00695C, 0xE0F2F1, function()
        if state.baseThrottle < 1.0 then
            state.baseThrottle = math.min(15.0, math.floor((state.baseThrottle + 0.05) * 100 + 0.5) / 100)
        else
            state.baseThrottle = math.min(15.0, state.baseThrottle + 1.0)
        end
    end)
    addButton(pfdX + bW2 + 4, bY2, bW2, rowH, "BASE -", 0x37474F, 0xECEFF1, function()
        if state.baseThrottle <= 1.0 then
            state.baseThrottle = math.max(0.0, math.floor((state.baseThrottle - 0.05) * 100 + 0.5) / 100)
        else
            state.baseThrottle = math.max(0.0, state.baseThrottle - 1.0)
        end
    end)
    addButton(pfdX + (bW2 + 4) * 2, bY2, bW2, rowH, "RE-SCAN", 0x1565C0, 0xE3F2FD, function()
        scanQuadTurtles()
        state.statusMsg = "Hardware Re-scanned!"
    end)
    addButton(pfdX + (bW2 + 4) * 3, bY2, bW2, rowH, "CALIBRATE", 0x6A1B9A, 0xF3E5F5, function()
        scanQuadTurtles()
        local cur = altiSensor and altiSensor.getHeight() or 100
        state.calibStartAlt = cur
        state.calibTargetAlt = 200.0 -- 校正目標固定設定為 200m
        state.virtualAlt = cur
        state.targetAlt = 200.0
        state.calibPhase = "GROUND_SEARCH"
        state.calibThrottle = 0.0
        state.baseThrottle = 0.0
        state.rampRate = 0.0001
        state.mode = "CALIBRATING"
        state.stableTimer = 0
        altPID:reset()
        state.statusMsg = "Calib: Searching Lift to 200m..."
    end)

    -- Row 3: 主要飛控模式按鈕 (HOLD ALT / STOP)
    local bY3 = bY2 + rowH + 6
    local bW3 = math.floor((screenW - 12 - 6) / 2)
    local bH3 = math.max(rowH, screenH - bY3 - 6)

    local holdBg = (state.mode == "HOLD_ALT") and 0x2E7D32 or 0x1B5E20
    addButton(pfdX, bY3, bW3, bH3, " [ HOLD ALT ] ", holdBg, 0xFFFFFF, function()
        state.mode = "HOLD_ALT"
        local cur = altiSensor and altiSensor.getHeight() or state.targetAlt
        state.virtualAlt = cur
        altPID:reset()
        state.statusMsg = string.format("Holding Altitude: %.0fm", state.targetAlt)
    end)

    local stopBg = (state.mode == "IDLE") and 0xC62828 or 0xB71C1C
    addButton(pfdX + bW3 + 6, bY3, bW3, bH3, " [ STOP / IDLE ] ", stopBg, 0xFFFFFF, function()
        state.mode = "IDLE"
        altPID:reset()
        applyQuadThrust(0, 0, 0, 0)
        state.statusMsg = "All 4 Engines Stopped"
    end)

    -- 繪製所有按鈕
    local btnTextSize = (rowH >= 24) and 2 or 1
    for _, btn in ipairs(buttons) do
        safeFilledRectangle(btn.x, btn.y, btn.w, btn.h, btn.bg)
        safeRectangle(btn.x, btn.y, btn.w, btn.h, 0x8CA0B4)
        local tx = btn.x + math.max(2, math.floor((btn.w - #btn.text * 6 * btnTextSize) / 2))
        local ty = btn.y + math.floor((btn.h - 8 * btnTextSize) / 2)
        safeDrawText(tx, ty, btn.text, btn.fg, nil, btnTextSize)
    end

    -- 6. 同步至螢幕
    pcall(function()
        if gpu.sync then
            gpu.sync()
        end
    end)
end

-- ========================================================
-- 8. 飛控閉環控制迴圈 (20Hz)
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

        if state.mode == "HOLD_ALT" then
            if state.virtualAlt < state.targetAlt then
                state.virtualAlt = math.min(state.targetAlt, state.virtualAlt + 0.05)
            elseif state.virtualAlt > state.targetAlt then
                state.virtualAlt = math.max(state.targetAlt, state.virtualAlt - 0.05)
            end

            local altError = state.virtualAlt - currAlt
            local deltaAlt = altPID:update(altError)

            if currVspeed > 0.8 then deltaAlt = deltaAlt - 0.3 end
            if currVspeed < -0.8 then deltaAlt = deltaAlt + 0.3 end

            -- 平衡關閉：四軸純集體推力
            applyQuadThrust(state.baseThrottle, deltaAlt, 0, 0)

        elseif state.mode == "CALIBRATING" then
            local climbDist = currAlt - state.calibStartAlt

            if state.calibPhase == "GROUND_SEARCH" then
                -- 階段 1: 地面持續穩健加速加力
                -- 只要飛艇未確鑿離地 (爬升 < 0.6m 且 垂直速度 < 0.15m/s)，加力速率持續穩定爬坡，絕不因 0.01 輕微抖動而中斷！
                state.calibThrottle = math.min(15.0, state.calibThrottle + state.rampRate)
                state.baseThrottle = math.floor(state.calibThrottle * 1000 + 0.5) / 1000

                if climbDist < 0.6 and currVspeed < 0.15 then
                    state.rampRate = math.min(0.008, state.rampRate + 0.00003)
                end

                applyQuadThrust(state.baseThrottle, 0, 0, 0)
                state.statusMsg = string.format("Search: %4.2f/15 (R:%5.4f)", state.baseThrottle, state.rampRate * 20)

                -- 確鑿起飛離地判定 (高度上升 >= 0.6m 或 垂直速度確鑿 >= 0.18m/s)
                if climbDist >= 0.6 or currVspeed >= 0.18 then
                    state.rampRate = 0.0
                    state.virtualAlt = currAlt
                    state.calibPhase = "CLIMB_TO_200"
                    altPID:reset()
                    state.statusMsg = string.format("Airborne! Climbing to 200m (B:%4.2f)", state.baseThrottle)
                end

            elseif state.calibPhase == "CLIMB_TO_200" then
                -- 階段 2: 爬升至 200m (如果已經在上升則不增加功率)
                -- 速度不足 (V < 0.10 且高度 < 195m) 時才極微慢補充微小功率
                if currVspeed < 0.10 and currAlt < 195.0 and state.baseThrottle < 14 then
                    state.calibThrottle = math.min(15.0, state.calibThrottle + 0.0004)
                    state.baseThrottle = math.floor(state.calibThrottle * 1000 + 0.5) / 1000
                elseif currVspeed > 0.65 and state.baseThrottle > 0.02 then
                    -- 上升過快時微幅回調
                    state.calibThrottle = math.max(0.01, state.calibThrottle - 0.001)
                    state.baseThrottle = math.floor(state.calibThrottle * 1000 + 0.5) / 1000
                end

                state.virtualAlt = math.min(200.0, state.virtualAlt + 0.03)
                local altError = state.virtualAlt - currAlt
                local deltaAlt = altPID:update(altError)

                if currVspeed > 0.50 then deltaAlt = deltaAlt - 0.3 end

                applyQuadThrust(state.baseThrottle, deltaAlt, 0, 0)
                state.statusMsg = string.format("Climbing to 200m: %5.1f/200m (V:%+.2f)", currAlt, currVspeed)

                -- 到達 200m 附近 (高度 >= 198.5m)
                if currAlt >= 198.5 then
                    state.calibPhase = "STABILIZE_200"
                    state.virtualAlt = 200.0
                    state.targetAlt = 200.0
                    state.stableTimer = 0
                    altPID:reset()
                    state.statusMsg = "Reached 200m! Stabilizing & Recording..."
                end

            elseif state.calibPhase == "STABILIZE_200" then
                -- 階段 3: 在 200m 維持並記錄推力數據
                local altError = 200.0 - currAlt
                local deltaAlt = altPID:update(altError)

                if currVspeed > 0.20 then deltaAlt = deltaAlt - 0.2 end
                if currVspeed < -0.20 then deltaAlt = deltaAlt + 0.2 end

                applyQuadThrust(state.baseThrottle, deltaAlt, 0, 0)

                local altDiff = math.abs(altError)
                local vDiff = math.abs(currVspeed)

                -- 穩定標準: 高度誤差 < 0.6m, 垂直速度 < 0.15m/s
                if altDiff < 0.6 and vDiff < 0.15 then
                    state.stableTimer = state.stableTimer + 0.05
                    state.statusMsg = string.format("200m Hover Stabilizing (%.1f/5.0s)", state.stableTimer)

                    if state.stableTimer >= 5.0 then
                        -- 連續 5 秒維持 200m 穩定，記錄推力數據並切換定高！
                        state.mode = "HOLD_ALT"
                        state.targetAlt = 200.0
                        state.virtualAlt = 200.0
                        state.statusMsg = string.format("Calibrated at 200m! Hover Base: %.2f", state.baseThrottle)
                    end
                else
                    state.stableTimer = 0
                    if altError > 0.6 and state.baseThrottle < 14 and currVspeed < 0.10 then
                        state.baseThrottle = math.min(15.0, state.baseThrottle + 0.001)
                    elseif altError < -0.6 and state.baseThrottle > 0.01 and currVspeed > -0.10 then
                        state.baseThrottle = math.max(0.005, state.baseThrottle - 0.001)
                    end
                    state.statusMsg = string.format("200m Hover Trim (B:%4.2f)", state.baseThrottle)
                end
            end

        elseif state.mode == "IDLE" then
            applyQuadThrust(0, 0, 0, 0)
        end
        sleep(0.05)
    end
end

-- ========================================================
-- 9. 介面渲染迴圈與觸控事件監聽 (tm_monitor_touch)
-- ========================================================
local function renderLoop()
    while true do
        drawTomsUI()
        sleep(0.05)
    end
end

local function touchEventLoop()
    while true do
        local event, p1, p2, p3 = os.pullEvent()
        local clickX, clickY = nil, nil

        if event == "tm_monitor_touch" then
            clickX, clickY = p1, p2
        elseif event == "monitor_touch" then
            clickX, clickY = p2, p3
        elseif event == "mouse_click" then
            clickX, clickY = p2, p3
        end

        if clickX and clickY then
            for _, btn in ipairs(buttons) do
                if clickX >= btn.x and clickX < btn.x + btn.w and
                   clickY >= btn.y and clickY < btn.y + btn.h then
                    btn.action()
                    drawTomsUI()
                    break
                end
            end
        end
    end
end

parallel.waitForAll(flightControlLoop, renderLoop, touchEventLoop)
