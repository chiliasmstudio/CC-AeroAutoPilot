--[[
    Create: Avionics & CC: Tweaked
    Quad-Engine Wired Flight Controller (四軸有線飛控大腦 - 烏龜免跑程式)
    
    硬體架構:
    - 主電腦 (Advanced Computer) 運行本程式，連接 Advanced Monitor。
    - 4 隻普通 Turtle 透過 Wired Modem + 網路纜線 (Cable) 連接到主電腦。
    - 4 隻烏龜【不需要開機、不跑任何程式】，由主電腦直接遠程控制紅石輸出！
    - 4 隻烏龜在遊戲中設定 Label 標籤: "FL", "FR", "BL", "BR"。
    - 飛艇上安裝 Altitude Sensor (高度計) 與 Gimbal Sensor (姿態陀螺儀 - 可選自穩)。
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

-- 安全 blit 輔助函式 (自動確保文字、前景色、背景色長度嚴格一致)
local function safeBlit(x, y, text, fgChar, bgChar)
    display.setCursorPos(x, y)
    local len = #text
    local fg = (type(fgChar) == "string" and #fgChar == len) and fgChar or string.rep(tostring(fgChar or "0"), len)
    local bg = (type(bgChar) == "string" and #bgChar == len) and bgChar or string.rep(tostring(bgChar or "f"), len)
    display.blit(text, fg, bg)
end

-- ========================================================
-- 3. 四角烏龜引擎節點掃描與辨識 (FL, FR, BL, BR)
-- ========================================================
local engines = {
    FL = nil, -- 左前 (Front Left)
    FR = nil, -- 右前 (Front Right)
    BL = nil, -- 左後 (Back Left)
    BR = nil  -- 右後 (Back Right)
}

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
-- 6. 螢幕繪製函式 (Quad Cockpit Monitor UI)
-- ========================================================
local function drawQuadUI()
    local w, h = display.getSize()
    display.setBackgroundColor(colors.black)
    display.clear()

    -- 1. 標題列
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
    for y = 3, 8 do
        safeBlit(2, y, string.rep(" ", cardW), "0", "8")
    end
    safeBlit(3, 3, "ALTITUDE & ATTITUDE", "9", "8")
    
    local altStr = string.format("ALT: %6.1f m", currAlt)
    safeBlit(3, 5, altStr, "0", "8")
    
    local vspeedStr = string.format("V.SPD: %+5.1f m/s", currVspeed)
    safeBlit(3, 6, vspeedStr, "4", "8")

    local attStr = string.format("P:%+4.1f* R:%+4.1f*", currPitch, currRoll)
    safeBlit(3, 7, attStr, "3", "8")

    -- 3. 右側四軸引擎狀態卡片
    local rightX = cardW + 4
    local rightW = w - rightX
    for y = 3, 8 do
        safeBlit(rightX, y, string.rep(" ", rightW), "0", "8")
    end
    safeBlit(rightX + 1, 3, "QUAD ENGINES (0-15)", "9", "8")

    local function getStatusCol(node) return node and "5" or "e" end
    
    local flText = string.format("FL:%2d", engineOutputs.FL)
    local frText = string.format("FR:%2d", engineOutputs.FR)
    local row1 = flText .. "  " .. frText
    local row1Fg = string.rep(getStatusCol(engines.FL), 5) .. "00" .. string.rep(getStatusCol(engines.FR), 5)
    safeBlit(rightX + 1, 5, row1, row1Fg, "8")

    local blText = string.format("BL:%2d", engineOutputs.BL)
    local brText = string.format("BR:%2d", engineOutputs.BR)
    local row2 = blText .. "  " .. brText
    local row2Fg = string.rep(getStatusCol(engines.BL), 5) .. "00" .. string.rep(getStatusCol(engines.BR), 5)
    safeBlit(rightX + 1, 6, row2, row2Fg, "8")

    local modeFg = (state.mode == "HOLD_ALT") and "5" or "e"
    local modeRow = string.format("MODE:%-5s B:%2d", state.mode:sub(1,5), state.baseThrottle)
    safeBlit(rightX + 1, 7, modeRow, modeFg, "8")

    -- 4. 狀態訊息列
    local statusRow = string.format("STATUS: %-30s", state.statusMsg):sub(1, w - 2)
    safeBlit(2, 9, statusRow, "0", "f")

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
    addButton(4 + bW2*2, bY2, bW2, 2, "CALIBRATE", "a", "0", function()
        scanQuadTurtles()
        state.mode = "CALIBRATING"
    end)

    local bY3 = 17
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
