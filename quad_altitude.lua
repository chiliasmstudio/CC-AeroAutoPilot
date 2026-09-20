--[[
    Create: Avionics & CC: Tweaked
    Quad-Engine Wired Flight Controller (四軸有線飛控大腦 - 烏龜免跑程式)
    
    標籤命名支援:
    - 左前: "FL" 或 "FL_xxxx" (如 FL_FrontLeft, FL_Motor1)
    - 右前: "FR" 或 "FR_xxxx" (如 FR_EngineA)
    - 左後: "BL" 或 "BL_xxxx" (如 BL_Rotor)
    - 右後: "BR" 或 "BR_xxxx" (如 BR_BackRight)
    
    螢幕即時顯示各角連接的烏龜名稱與連線狀態！
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
    display.setPaletteColor(colors.black, 0x111622)      -- 深夜藍底色
    display.setPaletteColor(colors.gray, 0x1c2438)       -- 面板卡片深灰藍
    display.setPaletteColor(colors.lightGray, 0x5a6988)  -- 刻度與次要文字
    display.setPaletteColor(colors.blue, 0x2266cc)       -- 導航主藍色
    display.setPaletteColor(colors.cyan, 0x00d2ff)       -- 儀表亮青色
    display.setPaletteColor(colors.lime, 0x2ed573)       -- 啟動與推力綠
    display.setPaletteColor(colors.green, 0x1e824c)      -- 深綠按鈕
    display.setPaletteColor(colors.yellow, 0xffa502)     -- 警告與高亮金黃
    display.setPaletteColor(colors.red, 0xff4757)        -- 停機與警示紅
    display.setPaletteColor(colors.white, 0xf1f2f6)      -- 銳利白文字
end

local altiSensor   = peripheral.find("altitude_sensor")
local gimbalSensor = peripheral.find("gimbal_sensor")

-- 安全 blit 輔助函式
local function safeBlit(x, y, text, fgChar, bgChar)
    display.setCursorPos(x, y)
    local len = #text
    local fg = (type(fgChar) == "string" and #fgChar == len) and fgChar or string.rep(tostring(fgChar or "0"), len)
    local bg = (type(bgChar) == "string" and #bgChar == len) and bgChar or string.rep(tostring(bgChar or "f"), len)
    display.blit(text, fg, bg)
end

-- ========================================================
-- 3. 四角烏龜引擎節點掃描與辨識 (支援 FL, FL_xxx 等)
-- ========================================================
local engines = {
    FL = nil, -- 左前 (Front Left)
    FR = nil, -- 右前 (Front Right)
    BL = nil, -- 左後 (Back Left)
    BR = nil  -- 右後 (Back Right)
}

local function matchPrefix(label, prefix)
    local u = string.upper(label)
    -- 匹配 "FL" 或 "FL_xxxx" 或 "FL-xxxx" 或 "FL xxxx"
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
        if pType == "turtle" or pType == "computer" or pType == "redstone_relay" then
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

    -- 容錯：若標籤未設置，依序自動指派剩餘設備
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
-- 4. 四軸混控矩陣輸出 (Quad Mixer Matrix)
-- ========================================================
local engineOutputs = { FL = 0, FR = 0, BL = 0, BR = 0 }

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
-- 5. 飛控狀態與 PID 控制器
-- ========================================================
local state = {
    mode = "IDLE",       -- "IDLE", "HOLD_ALT", "CALIBRATING"
    targetAlt = 120.0,
    baseThrottle = 7,
    autoLevel = true,
    statusMsg = "System Ready"
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
-- 6. 螢幕繪製函式 (顯示名稱與四軸狀態)
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
    
    local modeFg = (state.mode == "HOLD_ALT") and "5" or "e"
    local modeRow = string.format("MODE:%-5s B:%2d", state.mode:sub(1,5), state.baseThrottle)
    safeBlit(3, 8, modeRow, modeFg, "8")

    -- 3. 右側四軸烏龜名稱與推力卡片 (Quad Names & Thrust)
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

    -- 顯示 4 個引擎的自訂 Label 名稱與當前輸出
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
    local bW2 = math.floor((w - 4) / 3)
    addButton(2, bY2, bW2, 2, "BASE +1", "3", "0", function()
        state.baseThrottle = math.min(15, state.baseThrottle + 1)
    end)
    addButton(3 + bW2, bY2, bW2, 2, "BASE -1", "9", "0", function()
        state.baseThrottle = math.max(0, state.baseThrottle - 1)
    end)
    addButton(4 + bW2*2, bY2, bW2, 2, "RE-SCAN", "a", "0", function()
        scanQuadTurtles()
        state.statusMsg = "Re-scanned all 4 engines!"
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
            drawQuadUI()
            local bestSignal = 0
            local minVspeed = 999

            for sig = 0, 15 do
                applyQuadThrust(sig, 0, 0, 0)
                state.statusMsg = string.format("Testing Quad [%2d/15]", sig)
                drawQuadUI()
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
