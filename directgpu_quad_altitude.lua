--[[
    Create: Avionics & CC: Tweaked + DirectGPU
    DirectGPU Quad-Engine Wired Flight Controller (DirectGPU 四軸有線全彩 A350 ECAM 飛控大腦)
    Version: v3.5.0
    
    智慧校準與防暴衝姿態自穩邏輯:
    - 類比紅石純淨輸出：修正 setOutput 導致訊號被強制頂到 15 滿載暴衝的底層問題。
    - Airbus A350 ECAM 航太級全彩儀表：全螢幕自適應大尺寸渲染 4 軸引擎弧形刻度、極限紅線標記、空巴綠指針、數位讀數方框與離散訊號。
    - 智慧變速探索起飛推力：起步極慢 -> 持續無動作則穩健加速加力 (不卡死在低檔) -> 偵測到起飛立刻放緩加力並平滑控速爬升。
    - 虛擬軌跡巡航爬升：以 0.30 m/s 柔和定速引導爬升至 +10 格懸浮，消除階躍誤差暴衝。
    - 姿態即時反向加力配平：傾斜時立刻對低側加力、高側減力，維持水平。
    - 穩定收斂判定：高度誤差 < 0.5m、垂直速度 < 0.12m/s、姿態傾斜 < 1.0° 連續 5 秒自動鎖定最佳懸停基準！
--]]

local VERSION = "v3.5.0"
print(string.format("[DIRECTGPU] Loading Avionics Master %s...", VERSION))

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
-- 4. 飛控狀態與 PID 控制器
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

local function addButton(x, y, w, h, text, bg, fg, action)
    table.insert(buttons, {
        x = x, y = y, w = w, h = h,
        text = text, bg = bg, fg = fg,
        action = action
    })
end

-- ========================================================
-- 5. Airbus A350 ECAM 引擎儀表全彩渲染函式
-- ========================================================
local function drawA350EngineDial(cx, cy, r, val, maxVal, label, sig)
    -- 1. 儀表上方標題 (例如 1 FL 或 FL)
    local labelSize = math.max(9, math.floor(r * 0.45))
    gpu.drawText(displayId, label, cx - 12, cy - r - 14, 200, 230, 255, "Arial", labelSize, "bold")

    -- 2. 儀表圓弧背景與主刻度弧 (210° 到 -30°，共 240° 扇形)
    local arcPts = {}
    local arcPtsOuter = {}
    for deg = 210, -30, -10 do
        local rad = math.rad(deg)
        table.insert(arcPts, {
            math.floor(cx + r * math.cos(rad) + 0.5),
            math.floor(cy - r * math.sin(rad) + 0.5)
        })
        table.insert(arcPtsOuter, {
            math.floor(cx + (r + 1) * math.cos(rad) + 0.5),
            math.floor(cy - (r + 1) * math.sin(rad) + 0.5)
        })
    end
    gpu.drawPolylines(displayId, arcPts, 160, 180, 205)
    gpu.drawPolylines(displayId, arcPtsOuter, 120, 140, 165)

    -- 主刻度線 (0%, 50%, 100%)
    local ticks = {210, 90, -30}
    for _, deg in ipairs(ticks) do
        local rad = math.rad(deg)
        local t1x = math.floor(cx + (r - 2) * math.cos(rad) + 0.5)
        local t1y = math.floor(cy - (r - 2) * math.sin(rad) + 0.5)
        local t2x = math.floor(cx + (r + 5) * math.cos(rad) + 0.5)
        local t2y = math.floor(cy - (r + 5) * math.sin(rad) + 0.5)
        gpu.drawLine(displayId, t1x, t1y, t2x, t2y, 180, 200, 225)
    end

    -- 3. 高推力紅線區 (Redline zone: 13.0 ~ 15.0 滿推力紅色警示)
    local redPts = {}
    for deg = 10, -30, -8 do
        local rad = math.rad(deg)
        table.insert(redPts, {
            math.floor(cx + r * math.cos(rad) + 0.5),
            math.floor(cy - r * math.sin(rad) + 0.5)
        })
    end
    gpu.drawPolylines(displayId, redPts, 255, 55, 55)

    -- 紅線極限標記 (A350 Red Limit Flag / Tick)
    local rTickRad = math.rad(-18)
    local rx1 = math.floor(cx + (r - 3) * math.cos(rTickRad) + 0.5)
    local ry1 = math.floor(cy - (r - 3) * math.sin(rTickRad) + 0.5)
    local rx2 = math.floor(cx + (r + 7) * math.cos(rTickRad) + 0.5)
    local ry2 = math.floor(cy - (r + 7) * math.sin(rTickRad) + 0.5)
    gpu.drawLine(displayId, rx1, ry1, rx2, ry2, 255, 50, 50)
    gpu.drawLine(displayId, rx1 + 1, ry1, rx2 + 1, ry2, 255, 50, 50)

    -- 4. 空巴經典霓虹綠指針 (A350 Thick Neon Green Needle)
    local ratio = math.min(1.0, math.max(0.0, val / (maxVal or 15.0)))
    local needleDeg = 210 - ratio * 240
    local nRad = math.rad(needleDeg)
    local nx = math.floor(cx + (r - 3) * math.cos(nRad) + 0.5)
    local ny = math.floor(cy - (r - 3) * math.sin(nRad) + 0.5)

    -- 繪製加粗指針
    gpu.drawLine(displayId, cx, cy, nx, ny, 50, 255, 100)
    gpu.drawLine(displayId, cx + 1, cy, nx + 1, ny, 50, 255, 100)
    gpu.drawLine(displayId, cx, cy + 1, nx, ny + 1, 50, 255, 100)

    -- 中心圓形軸心 (Hub)
    gpu.drawCircle(displayId, cx, cy, math.max(2, math.floor(r * 0.14)), 200, 220, 240, true)

    -- 5. 底部空巴數位數值讀數框 (A350 Digital Box)
    local boxW = math.max(38, math.floor(r * 1.5))
    local boxH = math.max(16, math.floor(r * 0.65))
    local boxX = cx - math.floor(boxW / 2)
    local boxY = cy + math.floor(r * 0.28)

    gpu.fillRect(displayId, boxX, boxY, boxW, boxH, 12, 18, 28)
    gpu.drawPolylines(displayId, {
        {boxX, boxY},
        {boxX + boxW, boxY},
        {boxX + boxW, boxY + boxH},
        {boxX, boxY + boxH},
        {boxX, boxY}
    }, 40, 150, 200)

    local valStr = string.format("%4.1f", val)
    local numFontSize = math.max(9, math.floor(boxH * 0.68))
    local textOffset = math.floor((boxW - #valStr * (numFontSize * 0.6)) / 2)
    gpu.drawText(displayId, valStr, boxX + math.max(3, textOffset), boxY + 2, 70, 255, 120, "Arial", numFontSize, "bold")

    -- 6. 離散訊號標籤 (例如 2/15)
    local sigStr = string.format("%d/15", sig or 0)
    local sigFontSize = math.max(8, math.floor(boxH * 0.55))
    gpu.drawText(displayId, sigStr, cx - math.floor(#sigStr * 3), boxY + boxH + 3, 140, 180, 215, "Arial", sigFontSize, "plain")
end

-- ========================================================
-- 6. DirectGPU 全螢幕響應式渲染介面
-- ========================================================
local function drawDirectGPU_UI()
    local dispInfo = gpu.getDisplayInfo(displayId)
    local screenW = dispInfo.pixelWidth or 320
    local screenH = dispInfo.pixelHeight or 240

    gpu.clear(displayId, 15, 20, 30)

    -- 1. 頂部標題列 (比例自適應)
    local headerH = math.max(26, math.floor(screenH * 0.08))
    gpu.fillRect(displayId, 0, 0, screenW, headerH, 25, 40, 65)
    gpu.drawText(displayId, string.format("QUAD-ENGINE AVIONICS - A350 ECAM %s", VERSION), 12, math.floor((headerH - 12) / 2), 240, 245, 255, "Arial", math.max(11, math.floor(headerH * 0.45)), "bold")

    local currAlt = altiSensor and altiSensor.getHeight() or 0
    local currVspeed = altiSensor and altiSensor.getVerticalSpeed() or 0
    local currPitch, currRoll = getGimbalData()

    -- 2. 儀表主區塊垂直尺寸分配
    local instY = headerH + 6
    local instH = math.floor((screenH - headerH) * 0.52)
    local leftW = math.max(100, math.floor(screenW * 0.30))

    -- [左側 PFD / 高度與飛行狀態卡片]
    local pfdX = 6
    gpu.fillRect(displayId, pfdX, instY, leftW, instH, 20, 26, 38)

    -- PFD 圓形高度儀表 (依 instH 自適應尺寸)
    local gR = math.min(28, math.floor(instH * 0.25))
    local gCX = pfdX + math.floor(leftW * 0.28)
    local gCY = instY + math.floor(instH * 0.35)

    gpu.drawCircle(displayId, gCX, gCY, gR, 35, 45, 60, true)
    gpu.drawCircle(displayId, gCX, gCY, gR, 100, 180, 255, false)
    local altText = string.format("%.0f", currAlt)
    gpu.drawText(displayId, altText, gCX - math.floor(#altText * 4), gCY - 7, 255, 255, 255, "Arial", math.max(12, math.floor(gR * 0.55)), "bold")
    gpu.drawText(displayId, "ALT(M)", gCX - 14, gCY + 7, 160, 200, 230, "Arial", 8, "plain")

    -- 狀態文字讀數
    local textX = pfdX + math.floor(leftW * 0.56)
    gpu.drawText(displayId, string.format("TGT: %.0fm", state.targetAlt), textX, instY + 14, 80, 220, 255, "Arial", 10, "bold")
    gpu.drawText(displayId, string.format("V.S: %+.2fm/s", currVspeed), textX, instY + 32, 255, 200, 80, "Arial", 10, "bold")
    
    if gimbalAvailable then
        gpu.drawText(displayId, string.format("P:%+4.1f* R:%+4.1f*", currPitch, currRoll), pfdX + 10, instY + math.floor(instH * 0.68), 180, 220, 255, "Arial", 9, "plain")
        gpu.drawText(displayId, "GIMBAL: ACTIVE [OK]", pfdX + 10, instY + math.floor(instH * 0.82), 80, 255, 120, "Arial", 9, "bold")
    else
        gpu.drawText(displayId, "P: ---   R: ---", pfdX + 10, instY + math.floor(instH * 0.68), 150, 150, 150, "Arial", 9, "plain")
        gpu.drawText(displayId, "GIMBAL: NO SENSOR", pfdX + 10, instY + math.floor(instH * 0.82), 255, 60, 60, "Arial", 9, "bold")
    end
    
    local mCol = (state.mode == "HOLD_ALT") and {80, 255, 120} or (state.mode == "CALIBRATING" and {80, 200, 255} or {255, 80, 80})
    gpu.drawText(displayId, string.format("MODE: %s", state.mode), textX, instY + 50, mCol[1], mCol[2], mCol[3], "Arial", 10, "bold")

    -- [右側 A350 ECAM 四軸引擎儀表區]
    local ecamX = pfdX + leftW + 6
    local ecamW = screenW - ecamX - 6
    gpu.fillRect(displayId, ecamX, instY, ecamW, instH, 18, 24, 34)

    -- 渲染 4 顆大尺寸 A350 ECAM 引擎儀表 (FL, FR, BL, BR)
    local slotW = math.floor((ecamW - 8) / 4)
    local slots = {"FL", "FR", "BL", "BR"}

    -- 動態計算大尺寸半徑
    local dialR = math.min(math.floor(slotW * 0.36), math.floor(instH * 0.30))
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

    -- ECAM 儀表底部數值
    gpu.drawText(displayId, string.format("BASE: %4.3f", state.baseThrottle), ecamX + 10, instY + instH - 12, 200, 220, 255, "Arial", 9, "plain")
    local autoLvlCol = (state.autoLevel and gimbalAvailable) and {80, 255, 120} or {255, 90, 90}
    gpu.drawText(displayId, string.format("AUTO-LVL: %s", (state.autoLevel and gimbalAvailable) and "ON" or "OFF"), ecamX + ecamW - 85, instY + instH - 12, autoLvlCol[1], autoLvlCol[2], autoLvlCol[3], "Arial", 9, "bold")

    -- 3. 狀態訊息橫條 (自適應垂直高度)
    local statY = instY + instH + 6
    local statH = math.max(18, math.floor(screenH * 0.07))
    gpu.fillRect(displayId, 6, statY, screenW - 12, statH, 18, 24, 34)
    gpu.drawText(displayId, string.format("STATUS: %s", state.statusMsg), 12, statY + math.floor((statH - 10) / 2), 240, 240, 240, "Arial", math.max(9, math.floor(statH * 0.48)), "plain")

    -- 4. 底部觸控按鈕區 (填滿整個螢幕剩餘高度)
    buttons = {}
    local btnStartY = statY + statH + 6
    local btnAreaH = screenH - btnStartY - 6
    local rowH = math.max(22, math.floor((btnAreaH - 12) / 3))

    -- 第一排按鈕：目標高度調整 (+10m, +1m, -1m, -10m, SET CURR)
    local btnY1 = btnStartY
    local btnW1 = math.floor((screenW - 36) / 5)

    addButton(6, btnY1, btnW1, rowH, "+10m", {40, 120, 60}, {255, 255, 255}, function()
        state.targetAlt = state.targetAlt + 10
    end)
    addButton(12 + btnW1, btnY1, btnW1, rowH, "+1m", {60, 150, 80}, {255, 255, 255}, function()
        state.targetAlt = state.targetAlt + 1
    end)
    addButton(18 + btnW1*2, btnY1, btnW1, rowH, "-1m", {180, 110, 40}, {255, 255, 255}, function()
        state.targetAlt = math.max(0, state.targetAlt - 1)
    end)
    addButton(24 + btnW1*3, btnY1, btnW1, rowH, "-10m", {180, 60, 50}, {255, 255, 255}, function()
        state.targetAlt = math.max(0, state.targetAlt - 10)
    end)
    addButton(30 + btnW1*4, btnY1, btnW1, rowH, "SET CURR", {40, 110, 160}, {255, 255, 255}, function()
        local cur = altiSensor and altiSensor.getHeight() or 0
        state.targetAlt = math.floor(cur + 0.5)
        state.virtualAlt = cur
        state.statusMsg = string.format("Target locked to current: %.0fm", state.targetAlt)
    end)

    -- 第二排按鈕：基準油門微調、重新掃描、校正 (+10格懸浮自穩)
    local btnY2 = btnStartY + rowH + 6
    local btnW2 = math.floor((screenW - 30) / 4)

    addButton(6, btnY2, btnW2, rowH, "BASE +", {40, 90, 140}, {255, 255, 255}, function()
        if state.baseThrottle < 1.0 then
            state.baseThrottle = math.min(15.0, math.floor((state.baseThrottle + 0.05) * 100 + 0.5) / 100)
        else
            state.baseThrottle = math.min(15.0, state.baseThrottle + 1.0)
        end
    end)
    addButton(12 + btnW2, btnY2, btnW2, rowH, "BASE -", {50, 70, 120}, {255, 255, 255}, function()
        if state.baseThrottle <= 1.0 then
            state.baseThrottle = math.max(0.0, math.floor((state.baseThrottle - 0.05) * 100 + 0.5) / 100)
        else
            state.baseThrottle = math.max(0.0, state.baseThrottle - 1.0)
        end
    end)
    addButton(18 + btnW2*2, btnY2, btnW2, rowH, "RE-SCAN", {40, 120, 150}, {255, 255, 255}, function()
        scanQuadTurtles()
        state.statusMsg = "Hardware Re-scanned!"
    end)
    addButton(24 + btnW2*3, btnY2, btnW2, rowH, "CALIBRATE", {120, 60, 160}, {255, 255, 255}, function()
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

    -- 第三排按鈕：主要飛控模式 (高度更大更顯眼)
    local btnY3 = btnStartY + (rowH + 6) * 2
    local mainBW = math.floor((screenW - 20) / 2)
    local lastRowH = screenH - btnY3 - 6

    local holdBg = (state.mode == "HOLD_ALT") and {30, 180, 80} or {50, 80, 60}
    addButton(6, btnY3, mainBW, lastRowH, "HOLD ALT", holdBg, {255, 255, 255}, function()
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

    local stopBg = (state.mode == "IDLE") and {180, 40, 40} or {90, 40, 40}
    addButton(14 + mainBW, btnY3, mainBW, lastRowH, "STOP / IDLE", stopBg, {255, 255, 255}, function()
        state.mode = "IDLE"
        altPID:reset()
        applyQuadThrust(0, 0, 0, 0)
        state.statusMsg = "All 4 Engines Stopped"
    end)

    -- 繪製所有按鈕
    for _, btn in ipairs(buttons) do
        gpu.fillRect(displayId, btn.x, btn.y, btn.w, btn.h, btn.bg[1], btn.bg[2], btn.bg[3])
        local btnFontSize = math.max(10, math.floor(btn.h * 0.38))
        local tx = btn.x + math.max(2, math.floor((btn.w - #btn.text * (btnFontSize * 0.62)) / 2))
        local ty = btn.y + math.floor((btn.h - btnFontSize) / 2)
        gpu.drawText(displayId, btn.text, tx, ty, btn.fg[1], btn.fg[2], btn.fg[3], "Arial", btnFontSize, "bold")
    end

    gpu.updateDisplay(displayId)
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
-- 8. 畫面渲染與 DirectGPU 專屬事件監聽
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
            processClick(p2, p3)
        elseif event == "monitor_touch" then
            if not mon then mon = peripheral.find("monitor") end
            if mon then monCharW, monCharH = mon.getSize() end
            local clickX = math.floor(((p2 - 0.5) / monCharW) * screenW)
            local clickY = math.floor(((p3 - 0.5) / monCharH) * screenH)
            processClick(clickX, clickY)
        elseif event == "mouse_click" then
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
