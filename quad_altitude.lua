--[[
    Create: Avionics & CC: Tweaked
    Quad-Engine Flight Controller - A350 ECAM Edition (四軸有線 A350 ECAM 原生螢幕飛控大腦)
    Version: v3.5.0
    
    升力校準與定高邏輯:
    - 移植 A350 ECAM 引擎儀表：在一般螢幕/終端機上以彩色字元繪製弧形刻度錶、高溫紅線區、連續浮點推力讀數與離散紅石訊號。
    - 穩健起飛偵測 (抗雜訊)：徹底杜絕 V+0.01 輕微抖動就誤判或重置加速，持續加速加力直到飛艇確鑿離地 (高度上升 >= 0.6m 或 垂直速度 >= 0.18m/s)。
    - 升力校準至 200m：上升中不增加多餘功率，到達 200m 懸停自穩 5 秒並記錄最佳基準推力！
    - 自動平衡功能已關閉 (四軸純集體升力混控)，姿態與陀螺儀即時角度仍完整顯示於儀表。
    - 類比紅石純淨輸出，消除階躍暴衝。
--]]

local VERSION = "v3.5.0"
print(string.format("[AVIONICS] Loading Quad-Engine Flight Controller (A350 ECAM) %s...", VERSION))

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

    -- 平衡功能已關閉：四軸輸出純集體推力
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
    virtualAlt = 100.0,       -- 虛擬平滑軌跡高度 (防階躍暴衝)
    baseThrottle = 1.0,       -- 支援浮點推力 (0.01 ~ 15.0)
    autoLevel = false,        -- 平衡功能關閉 (純集體升力模式)
    statusMsg = "System Ready",
    stableTimer = 0,          -- 穩定計時器 (秒)
    calibStartAlt = 0,        -- 校正啟動高度
    calibTargetAlt = 200.0,   -- 校正目標固定為 200m
    calibPhase = "GROUND_SEARCH", -- "GROUND_SEARCH", "CLIMB_TO_200", "STABILIZE_200"
    calibThrottle = 0.0,      -- 校正微調油門
    rampRate = 0.0001         -- 適應性加力速率
}

-- PID 控制器：高度修正限幅在 [-1.5, +1.5]
local altPID = PID.new(0.5, 0.02, 0.6, -1.5, 1.5)

local buttons = {}

local function addButton(x, y, w, h, text, bgBlit, fgBlit, action)
    table.insert(buttons, {
        x = x, y = y, w = w, h = h,
        text = text, bg = bgBlit, fg = fgBlit,
        action = action
    })
end

-- ========================================================
-- 6. Airbus A350 ECAM 引擎文字儀表渲染模組
-- ========================================================
local function drawECAMDial(dx, dy, dw, dh, slot, label, val, sig)
    val = val or 0.0
    sig = sig or 0
    local ratio = math.max(0, math.min(1.0, val / 15.0))
    local lbl = label or slot
    if #lbl > 4 then lbl = lbl:sub(1, 4) end

    -- 繪製背景底框
    for row = 0, dh - 1 do
        safeBlit(dx, dy + row, string.rep(" ", dw), "0", "7")
    end

    if dh >= 6 then
        -- 【完整大尺寸 A350 ECAM 儀表】(>= 6 行)
        -- 行 1: 標題列 +-- FL --+
        local topBar = string.format("+--%s--+", lbl)
        if #topBar < dw then
            local padL = math.floor((dw - #topBar) / 2)
            local padR = dw - #topBar - padL
            topBar = string.rep("-", padL) .. topBar .. string.rep("-", padR)
        elseif #topBar > dw then
            topBar = topBar:sub(1, dw)
        end
        safeBlit(dx, dy, topBar, "9", "8")

        -- 行 2: 弧形刻度錶 (/====..\\)
        local arcLen = math.max(4, dw - 4)
        local fillCount = math.floor(ratio * (arcLen - 1) + 0.5)
        local arcChars = "/"
        local arcFg = "8"
        for k = 1, arcLen - 2 do
            if k <= fillCount then
                arcChars = arcChars .. "="
                if k > (arcLen - 2) * 0.85 then
                    arcFg = arcFg .. "e" -- 高推力紅線區
                elseif k > (arcLen - 2) * 0.65 then
                    arcFg = arcFg .. "4" -- 黃色警戒區
                else
                    arcFg = arcFg .. "5" -- 綠色正常區
                end
            else
                arcChars = arcChars .. "."
                arcFg = arcFg .. "7" -- 暗灰空刻度
            end
        end
        arcChars = arcChars .. "\\"
        arcFg = arcFg .. "e" -- 紅線極限標記

        local arcLine = "|" .. arcChars .. "|"
        if #arcLine < dw then
            local padL = math.floor((dw - #arcLine) / 2)
            local padR = dw - #arcLine - padL
            arcLine = string.rep(" ", padL) .. arcLine .. string.rep(" ", padR)
            arcFg = string.rep("0", padL) .. "8" .. arcFg .. "8" .. string.rep("0", padR)
        end
        safeBlit(dx, dy + 1, arcLine, arcFg, "7")

        -- 行 3: 連續浮點數值讀數方框 [ 2.30 ]
        local valText = string.format("[%4.1f]", val)
        local valPadL = math.max(0, math.floor((dw - #valText) / 2))
        local valPadR = math.max(0, dw - #valText - valPadL)
        local valLine = string.rep(" ", valPadL) .. valText .. string.rep(" ", valPadR)
        local valFg = string.rep("0", valPadL) .. "955559" .. string.rep("0", valPadR)
        safeBlit(dx, dy + 2, valLine, valFg, "f")

        -- 行 4: 離散紅石訊號 2/15
        local sigText = string.format("%2d/15", sig)
        local sigPadL = math.max(0, math.floor((dw - #sigText) / 2))
        local sigPadR = math.max(0, dw - #sigText - sigPadL)
        local sigLine = string.rep(" ", sigPadL) .. sigText .. string.rep(" ", sigPadR)
        local sigFg = string.rep("0", sigPadL) .. "33333" .. string.rep("0", sigPadR)
        safeBlit(dx, dy + 3, sigLine, sigFg, "7")

        -- 行 5: 底部框線
        local btmBar = "+" .. string.rep("-", math.max(0, dw - 2)) .. "+"
        safeBlit(dx, dy + dh - 1, btmBar:sub(1, dw), "8", "7")

    else
        -- 【緊湊型 A350 ECAM 儀表】(< 6 行)
        -- 行 1: 標籤與刻度 [FL] /==.\
        local arcLen = math.max(3, dw - 6)
        local fillCount = math.floor(ratio * arcLen + 0.5)
        local arcChars = ""
        local arcFg = ""
        for k = 1, arcLen do
            if k <= fillCount then
                arcChars = arcChars .. "="
                arcFg = arcFg .. ((k > arcLen * 0.85) and "e" or ((k > arcLen * 0.65) and "4" or "5"))
            else
                arcChars = arcChars .. "."
                arcFg = arcFg .. "8"
            end
        end
        local r1Text = string.format("%-3s /%s\\", lbl, arcChars):sub(1, dw)
        local r1Fg = "999" .. "8" .. arcFg .. "e"
        if #r1Fg < #r1Text then r1Fg = r1Fg .. string.rep("0", #r1Text - #r1Fg) end
        safeBlit(dx, dy, r1Text, r1Fg:sub(1, #r1Text), "8")

        -- 行 2: 數值與訊號 2.30 (2/15)
        local r2Text = string.format("%4.1f (%d/15)", val, sig):sub(1, dw)
        local r2Fg = "5555" .. "3" .. string.rep("3", math.max(0, #r2Text - 5))
        safeBlit(dx, dy + 1, r2Text, r2Fg:sub(1, #r2Text), "f")
    end
end

-- ========================================================
-- 7. 螢幕繪製主介面 (A350 ECAM 風格)
-- ========================================================
local function drawQuadUI()
    local w, h = display.getSize()
    display.setBackgroundColor(colors.black)
    display.clear()

    -- 1. 頂部標題列 (顯示 A350 ECAM 版本號)
    local titleText = string.format("  QUAD-ENGINE AVIONICS - A350 ECAM %s  ", VERSION)
    local padL = math.floor((w - #titleText) / 2)
    local padR = w - #titleText - padL
    local header = string.rep(" ", math.max(0, padL)) .. titleText .. string.rep(" ", math.max(0, padR))
    safeBlit(1, 1, header:sub(1, w), "0", "b")

    local currAlt = altiSensor and altiSensor.getHeight() or 0
    local currVspeed = altiSensor and altiSensor.getVerticalSpeed() or 0
    local currPitch, currRoll = getGimbalData()

    -- 2. 佈局尺寸分配 (左側 PFD 儀表 + 右側 4 軸 A350 引擎卡片)
    local cardH = math.max(6, math.min(9, h - 11))
    local leftW = math.max(20, math.floor(w * 0.38))
    local rightX = leftW + 3
    local rightW = w - rightX

    -- [左側 PFD / 飛行數據卡片]
    for y = 3, 2 + cardH do
        safeBlit(2, y, string.rep(" ", leftW), "0", "8")
    end
    safeBlit(3, 3, "PFD / TELEMETRY", "9", "8")
    
    local altStr = string.format("ALT:%5.1fm (TGT:%4.0f)", currAlt, state.targetAlt)
    if #altStr > leftW - 2 then altStr = string.format("A:%5.1f T:%4.0f", currAlt, state.targetAlt) end
    safeBlit(3, 4, altStr:sub(1, leftW - 2), "0", "8")
    
    local vspeedStr = string.format("V.SPD: %+5.2f m/s", currVspeed)
    safeBlit(3, 5, vspeedStr:sub(1, leftW - 2), "4", "8")

    -- 姿態角度與陀螺儀狀態 (保留顯示)
    local attStr = string.format("P:%+4.1f* R:%+4.1f*", currPitch, currRoll)
    safeBlit(3, 6, attStr:sub(1, leftW - 2), "3", "8")

    if gimbalAvailable then
        safeBlit(3, 7, "GIMBAL: ACTIVE [OK]", "5", "8")
    else
        safeBlit(3, 7, "GIMBAL: NO SENSOR!", "e", "8")
    end
    
    local modeFg = (state.mode == "HOLD_ALT") and "5" or (state.mode == "CALIBRATING" and "4" or "e")
    local modeRow = string.format("MODE:%-5s B:%4.2f", state.mode:sub(1,5), state.baseThrottle)
    safeBlit(3, 8, modeRow:sub(1, leftW - 2), modeFg, "8")

    -- [右側 A350 ECAM 四軸引擎儀表區]
    local slots = {"FL", "FR", "BL", "BR"}

    if rightW >= 40 then
        -- 橫向排布 4 具 A350 引擎儀表 (1x4)
        local dialW = math.floor((rightW - 3) / 4)
        for i, slot in ipairs(slots) do
            local dx = rightX + (i - 1) * (dialW + 1)
            local node = engines[slot]
            local lbl = node and (node.label or node.name or slot) or slot
            drawECAMDial(dx, 3, dialW, cardH, slot, lbl, virtualOutputs[slot], engineOutputs[slot])
        end
    else
        -- 2x2 矩陣排布 A350 引擎儀表 (FL, FR / BL, BR)
        local dialW = math.floor((rightW - 2) / 2)
        local dialH = math.max(3, math.floor(cardH / 2))
        
        -- 上排: FL, FR
        local flNode = engines.FL
        local frNode = engines.FR
        drawECAMDial(rightX, 3, dialW, dialH, "FL", flNode and flNode.label or "FL", virtualOutputs.FL, engineOutputs.FL)
        drawECAMDial(rightX + dialW + 1, 3, dialW, dialH, "FR", frNode and frNode.label or "FR", virtualOutputs.FR, engineOutputs.FR)

        -- 下排: BL, BR
        local blNode = engines.BL
        local brNode = engines.BR
        local dy2 = 3 + dialH
        drawECAMDial(rightX, dy2, dialW, dialH, "BL", blNode and blNode.label or "BL", virtualOutputs.BL, engineOutputs.BL)
        drawECAMDial(rightX + dialW + 1, dy2, dialW, dialH, "BR", brNode and brNode.label or "BR", virtualOutputs.BR, engineOutputs.BR)
    end

    -- 4. 狀態訊息列
    local statusY = 3 + cardH + 1
    local statusRow = string.format("STATUS: %-40s", state.statusMsg):sub(1, w - 2)
    safeBlit(2, statusY, statusRow, "0", "f")

    -- 5. 觸控按鈕區
    buttons = {}

    -- 第一排：目標高度調整 (+10m, +1m, -1m, -10m, SET CURR)
    local bY1 = statusY + 2
    local bW1 = math.floor((w - 6) / 5)
    local bH = (h >= 24) and 2 or 1

    addButton(2, bY1, bW1, bH, "+10m", "d", "0", function()
        state.targetAlt = state.targetAlt + 10
    end)
    addButton(3 + bW1, bY1, bW1, bH, "+1m", "5", "0", function()
        state.targetAlt = state.targetAlt + 1
    end)
    addButton(4 + bW1*2, bY1, bW1, bH, "-1m", "1", "0", function()
        state.targetAlt = math.max(0, state.targetAlt - 1)
    end)
    addButton(5 + bW1*3, bY1, bW1, bH, "-10m", "e", "0", function()
        state.targetAlt = math.max(0, state.targetAlt - 10)
    end)
    addButton(6 + bW1*4, bY1, bW1, bH, "SET CURR", "3", "0", function()
        local cur = altiSensor and altiSensor.getHeight() or 0
        state.targetAlt = math.floor(cur + 0.5)
        state.virtualAlt = cur
        state.statusMsg = string.format("Target locked to current: %.0fm", state.targetAlt)
    end)

    -- 第二排：基準油門微調、重新掃描、校正 (升力校準至 200m)
    local bY2 = bY1 + bH + 1
    local bW2 = math.floor((w - 5) / 4)
    addButton(2, bY2, bW2, bH, "BASE +", "3", "0", function()
        if state.baseThrottle < 1.0 then
            state.baseThrottle = math.min(15.0, math.floor((state.baseThrottle + 0.05) * 100 + 0.5) / 100)
        else
            state.baseThrottle = math.min(15.0, state.baseThrottle + 1.0)
        end
    end)
    addButton(3 + bW2, bY2, bW2, bH, "BASE -", "9", "0", function()
        if state.baseThrottle <= 1.0 then
            state.baseThrottle = math.max(0.0, math.floor((state.baseThrottle - 0.05) * 100 + 0.5) / 100)
        else
            state.baseThrottle = math.max(0.0, state.baseThrottle - 1.0)
        end
    end)
    addButton(4 + bW2*2, bY2, bW2, bH, "RE-SCAN", "b", "0", function()
        scanQuadTurtles()
        state.statusMsg = "Hardware Re-scanned!"
    end)
    addButton(5 + bW2*3, bY2, bW2, bH, "CALIBRATE", "a", "0", function()
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

    -- 第三排：主要飛控模式按鈕
    local bY3 = bY2 + bH + 1
    local mainBW = math.floor((w - 3) / 2)
    local mainBH = (h >= 24) and 3 or 2

    local holdBg = (state.mode == "HOLD_ALT") and "5" or "d"
    addButton(2, bY3, mainBW, mainBH, " [ HOLD ALT ] ", holdBg, "0", function()
        state.mode = "HOLD_ALT"
        local cur = altiSensor and altiSensor.getHeight() or state.targetAlt
        state.virtualAlt = cur
        altPID:reset()
        state.statusMsg = string.format("Holding Altitude: %.0fm", state.targetAlt)
    end)

    local stopBg = (state.mode == "IDLE") and "e" or "c"
    addButton(3 + mainBW, bY3, mainBW, mainBH, " [ STOP / IDLE ] ", stopBg, "0", function()
        state.mode = "IDLE"
        altPID:reset()
        applyQuadThrust(0, 0, 0, 0)
        state.statusMsg = "All 4 Engines Stopped"
    end)

    -- 繪製所有按鈕
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

            -- 平衡關閉：四軸集體推力
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
-- 9. 介面刷新與右鍵觸控監聽
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
