--[[
    modules/drivers/tom.lua
    Tom's Peripherals GPU Driver (Full Color Bitmap / Vector Graphics)
--]]

local function safeRequire(mod)
    local ok, res = pcall(require, mod)
    if ok then return res end
    return nil
end

local FlightCore = package.loaded["modules.flight_core"] or safeRequire("modules.flight_core") or _G.FlightCore
local FONT_5X7 = package.loaded["modules.font_5x7"] or safeRequire("modules.font_5x7") or _G.FONT_5X7

local VIEW_TITLES = {
    OVERVIEW = "FLIGHT MONITOR",
    PFD      = "PRIMARY FLIGHT",
    ECAM     = "ECAM 2x2",
    CTRL     = "FLIGHT CONTROLS",
    NAV      = "NAV PRESETS",
    SYS      = "SYSTEM STATUS"
}

local Driver = {
    screens = {},
    refreshScreens = function(self)
        local existing = {}
        for _, scr in ipairs(self.screens) do
            if scr.id then existing[scr.id] = scr end
        end
        local newScreens = {}
        for _, name in ipairs(peripheral.getNames()) do
            local pType = peripheral.getType(name)
            if pType == "tm_gpu" or pType == "toms_gpu" or pType == "gpu" or pType == "tm_monitor" then
                local gpu = peripheral.wrap(name)
                if gpu and gpu.fill and gpu.filledRectangle then
                    -- 必須先 setSize + refreshSize 才能 getSize 得到正確像素解析度
                    pcall(function() if gpu.setSize then gpu.setSize(64) end end)
                    pcall(function() if gpu.refreshSize then gpu.refreshSize() end end)
                    local w, h = 320, 240
                    local ok, gw, gh = pcall(function() return gpu.getSize() end)
                    if ok and gw and gh and gw > 0 and gh > 0 then
                        if gw < 32 then gw = gw * 128 end
                        if gh < 32 then gh = gh * 128 end
                        w, h = gw, gh
                    end
                    local prev = existing[name]
                    table.insert(newScreens, {
                        id = name,
                        gpu = gpu,
                        screenW = w,
                        screenH = h,
                        currentView = prev and prev.currentView or "OVERVIEW",
                        isMenuOpen = prev and prev.isMenuOpen or false,
                        buttons = prev and prev.buttons or {}
                    })
                end
            end
        end
        self.screens = newScreens
        return #self.screens > 0
    end,
    init = function(self)
        -- 先做一次掃描讓 GPU setSize，等待硬體初始化完成後再重新讀取真實尺寸
        self:refreshScreens()
        sleep(0.1)
        return self:refreshScreens()
    end,
    toARGB = function(self, hexColor)
        if type(hexColor) == "table" then
            local r = hexColor[1] or 0
            local g = hexColor[2] or 0
            local b = hexColor[3] or 0
            local a = hexColor[4] or 255
            local val = a * 16777216 + r * 65536 + g * 256 + b
            if val >= 2147483648 then val = val - 4294967296 end
            return val
        elseif type(hexColor) == "number" then
            local a = 255
            local r = math.floor(hexColor / 65536) % 256
            local g = math.floor(hexColor / 256) % 256
            local b = hexColor % 256
            local val = a * 16777216 + r * 65536 + g * 256 + b
            if val >= 2147483648 then val = val - 4294967296 end
            return val
        end
        return -1
    end,
    drawText = function(self, scr, x, y, text, color, scale)
        scale = scale or 1
        local curX = x
        local argb = self:toARGB(color)
        local sw, sh = scr.screenW, scr.screenH
        local POW2 = { [0]=1, [1]=2, [2]=4, [3]=8, [4]=16, [5]=32, [6]=64, [7]=128 }

        for i = 1, #text do
            local ch = text:sub(i, i)
            local bitmap = FONT_5X7[ch] or FONT_5X7['?']

            if bitmap then
                for col = 1, 5 do
                    local byte = bitmap[col]
                    local px = curX + (col - 1) * scale
                    if px >= 1 and px <= sw then
                        local runStart = nil
                        local runLen = 0
                        for row = 0, 6 do
                            if (math.floor(byte / POW2[row]) % 2) == 1 then
                                if not runStart then runStart = row end
                                runLen = runLen + 1
                            else
                                if runStart then
                                    local py = y + runStart * scale
                                    local ph = runLen * scale
                                    if py <= sh then
                                        pcall(function() scr.gpu.filledRectangle(px, py, scale, ph, argb) end)
                                    end
                                    runStart = nil
                                    runLen = 0
                                end
                            end
                        end
                        if runStart then
                            local py = y + runStart * scale
                            local ph = runLen * scale
                            if py <= sh then
                                pcall(function() scr.gpu.filledRectangle(px, py, scale, ph, argb) end)
                            end
                        end
                    end
                end
            end
            curX = curX + 6 * scale
        end
    end,
    draw = function(self)
        for _, scr in ipairs(self.screens) do
            local ok, err = pcall(function() self:drawScreen(scr) end)
            if not ok then
                pcall(function() self:refreshScreens() end)
                break
            end
        end
    end,
    drawScreen = function(self, scr)
        local ok, w, h = pcall(function() return scr.gpu.getSize() end)
        -- Tom's GPU getSize() 返回像素大小；若回傳值疑似圖塊單位（< 32）則乘以 128 換算
        if ok and w and h and w > 0 and h > 0 then
            if w < 32 then w = w * 128 end
            if h < 32 then h = h * 128 end
            scr.screenW, scr.screenH = w, h
        end
        local sw, sh = scr.screenW, scr.screenH
        local isFull = (sw >= 190 and sh >= 190) -- 5x5 (640x640) 或以上為完整顯示，3x3/4x4/5x4/4x5 為精簡版

        local function toARGB(c) return self:toARGB(c) end
        local function sFill(c) pcall(function() scr.gpu.fill(toARGB(c)) end) end
        local function sFR(x, y, bw, bh, c)
            x, y = math.max(1, math.min(sw, x)), math.max(1, math.min(sh, y))
            bw, bh = math.max(1, math.min(sw - x + 1, bw)), math.max(1, math.min(sh - y + 1, bh))
            pcall(function() scr.gpu.filledRectangle(x, y, bw, bh, toARGB(c)) end)
        end
        local function sR(x, y, bw, bh, c)
            x, y = math.max(1, math.min(sw, x)), math.max(1, math.min(sh, y))
            bw, bh = math.max(1, math.min(sw - x + 1, bw)), math.max(1, math.min(sh - y + 1, bh))
            pcall(function() scr.gpu.rectangle(x, y, bw, bh, toARGB(c)) end)
        end
        local function sL(x1, y1, x2, y2, c)
            pcall(function() scr.gpu.line(math.max(1, math.min(sw, x1)), math.max(1, math.min(sh, y1)), math.max(1, math.min(sw, x2)), math.max(1, math.min(sh, y2)), toARGB(c)) end)
        end
        local function sLS(x1, y1, x2, y2, c)
            pcall(function()
                if scr.gpu.lineS then scr.gpu.lineS(math.max(1, math.min(sw, x1)), math.max(1, math.min(sh, y1)), math.max(1, math.min(sw, x2)), math.max(1, math.min(sh, y2)), toARGB(c))
                else scr.gpu.line(math.max(1, math.min(sw, x1)), math.max(1, math.min(sh, y1)), math.max(1, math.min(sw, x2)), math.max(1, math.min(sh, y2)), toARGB(c)) end
            end)
        end
        local function sTxt(x, y, txt, fc, sz)
            self:drawText(scr, x, y, txt, fc, sz or 1)
        end
        local function sArc(cx, cy, r, startD, endD, stepD, c)
            local px, py = nil, nil
            local st = (startD > endD) and -math.abs(stepD or 5) or math.abs(stepD or 5)
            for deg = startD, endD, st do
                local rad = math.rad(deg)
                local nx, ny = cx + r * math.cos(rad), cy - r * math.sin(rad)
                if px and py then sLS(px, py, nx, ny, c) end
                px, py = nx, ny
            end
        end

        local function addBtn(x, y, bw, bh, text, bg, fg, act)
            table.insert(scr.buttons, {x=x, y=y, w=bw, h=bh, text=text, bg=bg, fg=fg, action=act})
        end

        sFill(0x0F141E)
        scr.buttons = {}

        -- 1. 頂部狀態列
        local headerH = isFull and math.max(26, math.min(36, math.floor(sh * 0.06))) or (sh < 220 and 14 or 20)
        sFR(1, 1, sw, headerH, 0x192841)

        local menuBtnW = isFull and 54 or (sw < 220 and 18 or 24)
        local menuBtnH = headerH - 4
        local menuBtnX = sw - menuBtnW - 3
        local menuBtnY = 2
        if scr.isMenuOpen then
            addBtn(menuBtnX, menuBtnY, menuBtnW, menuBtnH, "[X]", 0x992222, 0xFFFFFF, function() scr.isMenuOpen = false end)
        else
            addBtn(menuBtnX, menuBtnY, menuBtnW, menuBtnH, isFull and "[MENU]" or "[=]", 0x224488, 0xFFFFFF, function() scr.isMenuOpen = true end)
        end

        local maxTitleW = menuBtnX - 12
        local viewTitle = scr.isMenuOpen and "SELECT VIEW" or (isFull and (VIEW_TITLES[scr.currentView] or scr.currentView) or scr.currentView)
        local titleSize = (isFull and #viewTitle * 12 <= maxTitleW and headerH >= 24) and 2 or 1
        sTxt(6, math.floor((headerH - 7 * titleSize) / 2) + 1, viewTitle, 0xF0F5FF, titleSize)

        -- 佈局高度自適應分配器
        local numBtnRows = isFull and 3 or 2
        local btnRowH = isFull and math.max(24, math.min(32, math.floor(sh * 0.08))) or (sh < 220 and 12 or (sh < 350 and 16 or 22))
        local btnSpacing = (sh < 220) and 2 or 3
        local totalBtnH = numBtnRows * btnRowH + (numBtnRows - 1) * btnSpacing
        local statH = isFull and 20 or (sh < 220 and 11 or 15)

        local topMargin = headerH + ((sh < 220) and 2 or 4)
        local statY = sh - totalBtnH - statH - ((sh < 220) and 3 or 5)
        local instY = topMargin
        local instH = statY - instY - ((sh < 220) and 2 or 4)
        local btnAreaY = statY + statH + ((sh < 220) and 2 or 3)

        -- 2. 視圖路由
        if scr.isMenuOpen then
            local cardW = math.floor((sw - 16) / 2)
            local cardH = math.floor((sh - headerH - 16) / 3)
            local startY = headerH + 4

            local function addMenuCard(col, row, title, targetView, color)
                local cx = 4 + (col - 1) * (cardW + 6)
                local cy = startY + (row - 1) * (cardH + 4)
                addBtn(cx, cy, cardW, cardH, title, color, 0xFFFFFF, function()
                    scr.currentView = targetView
                    scr.isMenuOpen = false
                end)
            end

            local shortTitle = cardW < 65
            addMenuCard(1, 1, isFull and "1. OVERVIEW (ALL)" or (shortTitle and "1. OVR" or "1. OVERVIEW"), "OVERVIEW", 0x1B3A60)
            addMenuCard(2, 1, isFull and "2. PFD (FLIGHT)" or (shortTitle and "2. PFD" or "2. PFD"), "PFD", 0x145A32)
            addMenuCard(1, 2, isFull and "3. ECAM (QUAD ENG)" or (shortTitle and "3. ECAM" or "3. ECAM"), "ECAM", 0x78281F)
            addMenuCard(2, 2, isFull and "4. CTRL (CONTROLS)" or (shortTitle and "4. CTRL" or "4. CTRL"), "CTRL", 0x117864)
            addMenuCard(1, 3, isFull and "5. NAV (PRESETS)" or (shortTitle and "5. NAV" or "5. NAV"), "NAV", 0x2471A3)
            addMenuCard(2, 3, isFull and "6. SYS (DIAGNOSE)" or (shortTitle and "6. SYS" or "6. SYS"), "SYS", 0x512E5F)

        elseif scr.currentView == "OVERVIEW" then
            local currAlt = FlightCore.altiSensor and FlightCore.altiSensor.getHeight() or 0
            local currVspeed = FlightCore.altiSensor and FlightCore.altiSensor.getVerticalSpeed() or 0
            local currPitch, currRoll = FlightCore.getGimbalData()

            if isFull then
                -- 5x5 或以上：完整航電儀表板 (Full Avionics Dashboard)
                local leftW = math.max(140, math.floor(sw * 0.38))
                local pfdX = 6
                local ecamX = pfdX + leftW + 6
                local ecamW = sw - ecamX - 6

                -- 左側卡片: 主飛行儀表 (PFD)
                sFR(pfdX, instY, leftW, instH, 0x141A26)
                sR(pfdX, instY, leftW, instH, 0x283850)
                sTxt(pfdX + 6, instY + 4, "PRIMARY FLIGHT", 0xAAD2E6, 1)

                local gR = math.max(16, math.min(36, math.floor(instH * 0.26), math.floor(leftW * 0.24)))
                local gCX = pfdX + math.floor(leftW * 0.25)
                local gCY = instY + math.floor(instH * 0.54)
                for dy = -gR, gR do
                    local dx = math.floor(math.sqrt(math.max(0, gR*gR - dy*dy)) + 0.5)
                    sL(gCX - dx, gCY + dy, gCX + dx, gCY + dy, 0x1E2837)
                end
                sArc(gCX, gCY, gR, 0, 360, 10, 0x50AAF0)
                local altText = string.format("%.0f", currAlt)
                local altSz = (gR >= 22) and 2 or 1
                sTxt(gCX - math.floor(#altText * 6 * altSz / 2), gCY - 3 * altSz - 1, altText, 0xFFFFFF, altSz)

                local textX = pfdX + math.floor(leftW * 0.52)
                local rowSpacing = math.max(12, math.floor((instH - 24) / 4))
                sTxt(textX, instY + 14, string.format("TGT: %.0fm", FlightCore.state.targetAlt), 0x50E6FF, 1)
                sTxt(textX, instY + 14 + rowSpacing, string.format("V.S: %+.2f", currVspeed), 0xFFCD4B, 1)
                local mCol = (FlightCore.state.mode == "HOLD_ALT") and 0x50FF78 or 0xFF5050
                sTxt(textX, instY + 14 + rowSpacing * 2, string.format("MOD: %s", FlightCore.state.mode:sub(1,6)), mCol, 1)
                sTxt(textX, instY + 14 + rowSpacing * 3, string.format("P:%+2.0f R:%+2.0f", currPitch, currRoll), 0xB4DCFF, 1)

                -- 右側卡片: 2x2 四象限發動機動力監控
                sFR(ecamX, instY, ecamW, instH, 0x121822)
                sR(ecamX, instY, ecamW, instH, 0x283850)

                local subW = math.floor((ecamW - 8) / 2)
                local subH = math.floor((instH - 8) / 2)
                local miniSlots = {
                    {slot="FL", col=1, row=1, name="FRONT-LEFT"},
                    {slot="FR", col=2, row=1, name="FRONT-RIGHT"},
                    {slot="BL", col=1, row=2, name="BACK-LEFT"},
                    {slot="BR", col=2, row=2, name="BACK-RIGHT"}
                }
                for _, ms in ipairs(miniSlots) do
                    local sx = ecamX + 3 + (ms.col - 1) * (subW + 2)
                    local sy = instY + 3 + (ms.row - 1) * (subH + 2)
                    local qH = FlightCore.getQuadHealth(ms.slot)
                    local val = FlightCore.virtualOutputs[ms.slot]
                    local online = qH.online > 0

                    sFR(sx, sy, subW, subH, 0x18202E)
                    sR(sx, sy, subW, subH, online and 0x284864 or 0x642828)
                    sTxt(sx + 4, sy + 3, string.format("[%s] %d ENG", ms.slot, qH.total), 0xC8E6FF, 1)
                    local onCol = online and 0x50FF78 or 0xFF5050
                    local onText = string.format("%d/%d ON", qH.online, qH.total)
                    sTxt(sx + subW - #onText * 6 - 4, sy + 3, onText, onCol, 1)

                    -- 動態油門條
                    local barY = sy + math.floor(subH * 0.38)
                    local barW = subW - 8
                    local barH = math.max(6, math.min(14, math.floor(subH * 0.20)))
                    sFR(sx + 4, barY, barW, barH, 0x0C121C)
                    local fillW = math.floor(barW * math.min(1.0, math.max(0.0, val / 15.0)))
                    local barCol = (val > 12) and 0xFF4500 or ((val > 8) and 0xFFD700 or 0x32CD32)
                    if fillW > 0 then sFR(sx + 4, barY, fillW, barH, barCol) end
                    sR(sx + 4, barY, barW, barH, 0x3C4B64)

                    -- 數值指示
                    sTxt(sx + 4, sy + subH - 10, string.format("THR: %4.1f/15", val), 0x46FF78, 1)
                    local pwmText = string.format("PWM:%d", FlightCore.engineOutputs[ms.slot] or 0)
                    sTxt(sx + subW - #pwmText * 6 - 4, sy + subH - 10, pwmText, 0x96BEE1, 1)
                end

                -- 中段狀態通報
                sFR(pfdX, statY, sw - 12, statH, 0x1E2432)
                sR(pfdX, statY, sw - 12, statH, 0x3C4B64)
                sTxt(pfdX + 6, statY + 4, string.format("STATUS: %s", FlightCore.state.statusMsg:sub(1, 45)), 0x50E6FF, 1)

                -- 底部 3 排控制按鈕群
                local bY1 = btnAreaY
                local bW1 = math.floor((sw - 12 - 20) / 6)
                addBtn(pfdX, bY1, bW1, btnRowH, "+50m", 0x145A32, 0xFFFFFF, function() FlightCore.adjustTargetAlt(50) end)
                addBtn(pfdX + (bW1+4), bY1, bW1, btnRowH, "+10m", 0x1B5E20, 0xFFFFFF, function() FlightCore.adjustTargetAlt(10) end)
                addBtn(pfdX + (bW1+4)*2, bY1, bW1, btnRowH, "+1m", 0x2E7D32, 0xFFFFFF, function() FlightCore.adjustTargetAlt(1) end)
                addBtn(pfdX + (bW1+4)*3, bY1, bW1, btnRowH, "-1m", 0xE65100, 0xFFFFFF, function() FlightCore.adjustTargetAlt(-1) end)
                addBtn(pfdX + (bW1+4)*4, bY1, bW1, btnRowH, "-10m", 0xC62828, 0xFFFFFF, function() FlightCore.adjustTargetAlt(-10) end)
                addBtn(pfdX + (bW1+4)*5, bY1, bW1, btnRowH, "LOCK", 0x00838F, 0xFFFFFF, function() FlightCore.lockCurrentAlt() end)

                local bY2 = bY1 + btnRowH + btnSpacing
                local bW2 = math.floor((sw - 12 - 12) / 4)
                addBtn(pfdX, bY2, bW2, btnRowH, "BASE +1", 0x00695C, 0xFFFFFF, function() FlightCore.adjustBaseThrottle(1.0) end)
                addBtn(pfdX + bW2 + 4, bY2, bW2, btnRowH, "BASE -1", 0x37474F, 0xFFFFFF, function() FlightCore.adjustBaseThrottle(-1.0) end)
                addBtn(pfdX + (bW2 + 4)*2, bY2, bW2, btnRowH, "RE-SCAN", 0x1565C0, 0xFFFFFF, function() FlightCore.scanQuadTurtles(); FlightCore.state.statusMsg="Re-scanned!" end)
                addBtn(pfdX + (bW2 + 4)*3, bY2, bW2, btnRowH, "CALIB", 0x6A1B9A, 0xFFFFFF, function() FlightCore.startCalibration() end)

                local bY3 = bY2 + btnRowH + btnSpacing
                local bW3 = math.floor((sw - 12 - 4) / 2)
                local holdBg = (FlightCore.state.mode == "HOLD_ALT") and 0x2E7D32 or 0x1B5E20
                addBtn(pfdX, bY3, bW3, btnRowH, "[ HOLD ALTITUDE ]", holdBg, 0xFFFFFF, function() FlightCore.holdAltitude() end)
                local stopBg = (FlightCore.state.mode == "IDLE") and 0xC62828 or 0xB71C1C
                addBtn(pfdX + bW3 + 4, bY3, bW3, btnRowH, "[ STOP / IDLE ]", stopBg, 0xFFFFFF, function() FlightCore.stopEngines() end)
            else
                -- 3x3 ~ 4x5 精簡模式 (<5x5 Compact Mode, 適配 384x384 到 512x640 及小螢幕)
                local colW = math.floor((sw - 12) / 2)
                local pfdX = 4
                local ecamX = pfdX + colW + 4

                if colW >= 85 then
                    -- 寬度充足精簡版 (4x4, 5x4): 圓形高度錶 + 2x2 四象限進度條
                    sFR(pfdX, instY, colW, instH, 0x141A26)
                    sR(pfdX, instY, colW, instH, 0x283850)
                    sTxt(pfdX + 4, instY + 3, "ALT/ATT", 0xAAD2E6, 1)

                    local gR = math.max(8, math.min(22, math.floor(colW * 0.18), math.floor(instH * 0.22)))
                    local gCX = pfdX + math.max(gR + 4, math.floor(colW * 0.24))
                    local gCY = instY + math.floor(instH * 0.54)
                    for dy = -gR, gR do
                        local dx = math.floor(math.sqrt(math.max(0, gR*gR - dy*dy)) + 0.5)
                        sL(gCX - dx, gCY + dy, gCX + dx, gCY + dy, 0x1E2837)
                    end
                    sArc(gCX, gCY, gR, 0, 360, 15, 0x50AAF0)
                    local altText = string.format("%.0f", currAlt)
                    sTxt(gCX - math.floor(#altText * 3), gCY - 3, altText, 0xFFFFFF, 1)

                    local textX = gCX + gR + 6
                    local rowSpacing = math.max(8, math.min(18, math.floor((instH - 18) / 4)))
                    sTxt(textX, instY + 10, string.format("T:%.0f", FlightCore.state.targetAlt), 0x50E6FF, 1)
                    sTxt(textX, instY + 10 + rowSpacing, string.format("V:%+.1f", currVspeed), 0xFFCD4B, 1)
                    local mCol = (FlightCore.state.mode == "HOLD_ALT") and 0x50FF78 or 0xFF5050
                    sTxt(textX, instY + 10 + rowSpacing * 2, string.format("M:%s", FlightCore.state.mode:sub(1,4)), mCol, 1)
                    sTxt(textX, instY + 10 + rowSpacing * 3, string.format("P:%+.0f", currPitch), 0xB4DCFF, 1)

                    -- 右側: 2x2 Mini Quad Engines (FL/FR/BL/BR)
                    sFR(ecamX, instY, colW, instH, 0x121822)
                    sR(ecamX, instY, colW, instH, 0x283850)

                    local subW = math.floor((colW - 6) / 2)
                    local subH = math.floor((instH - 6) / 2)
                    local miniSlots = {
                        {slot="FL", col=1, row=1},
                        {slot="FR", col=2, row=1},
                        {slot="BL", col=1, row=2},
                        {slot="BR", col=2, row=2}
                    }
                    for _, ms in ipairs(miniSlots) do
                        local sx = ecamX + 2 + (ms.col - 1) * (subW + 2)
                        local sy = instY + 2 + (ms.row - 1) * (subH + 2)
                        local qH = FlightCore.getQuadHealth(ms.slot)
                        local val = FlightCore.virtualOutputs[ms.slot]
                        local online = qH.online > 0

                        sFR(sx, sy, subW, subH, 0x18202E)
                        sR(sx, sy, subW, subH, online and 0x284864 or 0x642828)
                        sTxt(sx + 2, sy + 2, string.format("[%s] %dE", ms.slot, qH.total), 0xC8E6FF, 1)

                        -- 動態微型進度條 (帶底框和邊框)
                        local barY = sy + math.floor(subH * 0.40)
                        local barW = math.max(4, subW - 6)
                        local barH = math.max(4, math.min(10, math.floor(subH * 0.18)))
                        sFR(sx + 3, barY, barW, barH, 0x0A1018)
                        local fillW = math.floor(barW * math.min(1.0, math.max(0.0, val / 15.0)))
                        local barCol = (val > 12) and 0xFF4500 or ((val > 8) and 0xFFD700 or 0x32CD32)
                        if fillW > 0 then sFR(sx + 3, barY, fillW, barH, barCol) end
                        sR(sx + 3, barY, barW, barH, 0x384860)

                        -- 數值指示
                        local btmLineY = sy + subH - 9
                        sTxt(sx + 2, btmLineY, string.format("%4.1f", val), 0x46FF78, 1)
                        local pwmStr = string.format("P:%d", FlightCore.engineOutputs[ms.slot] or 0)
                        sTxt(sx + subW - #pwmStr * 6 - 2, btmLineY, pwmStr, 0x96BEE1, 1)
                    end
                else
                    -- 極致窄螢幕 (3x3 螢幕, colW < 85px): 乾淨直向數據流，零碰撞
                    -- 左側: PFD 飛行姿態垂直數據堆疊
                    sFR(pfdX, instY, colW, instH, 0x141A26)
                    sR(pfdX, instY, colW, instH, 0x283850)
                    sTxt(pfdX + 3, instY + 2, "ALT/ATT", 0xAAD2E6, 1)

                    local rowSp = math.max(8, math.min(12, math.floor((instH - 14) / 4)))
                    local yBase = instY + 13
                    sTxt(pfdX + 3, yBase, string.format("ALT:%4.0f", currAlt), 0xFFFFFF, 1)
                    sTxt(pfdX + 3, yBase + rowSp, string.format("TGT:%4.0f", FlightCore.state.targetAlt), 0x50E6FF, 1)
                    sTxt(pfdX + 3, yBase + rowSp * 2, string.format("V.S:%+4.1f", currVspeed), 0xFFCD4B, 1)
                    local mCol = (FlightCore.state.mode == "HOLD_ALT") and 0x50FF78 or 0xFF5050
                    sTxt(pfdX + 3, yBase + rowSp * 3, string.format("MOD:%s", FlightCore.state.mode:sub(1, 4)), mCol, 1)

                    -- 右側: ECAM 四象限發動機直向數據堆疊 (FL, FR, BL, BR 逐行排布)
                    sFR(ecamX, instY, colW, instH, 0x121822)
                    sR(ecamX, instY, colW, instH, 0x283850)
                    sTxt(ecamX + 3, instY + 2, "ECAM 4Q", 0xAAD2E6, 1)

                    local qSlots = {"FL", "FR", "BL", "BR"}
                    for idx, slot in ipairs(qSlots) do
                        local qH = FlightCore.getQuadHealth(slot)
                        local val = FlightCore.virtualOutputs[slot]
                        local sig = FlightCore.engineOutputs[slot] or 0
                        local yPos = yBase + (idx - 1) * rowSp
                        local colVal = (val > 12) and 0xFF4500 or ((val > 8) and 0xFFD700 or 0x50FF78)
                        if qH.online == 0 then colVal = 0xFF5050 end
                        sTxt(ecamX + 3, yPos, string.format("%s%4.1f P%d", slot, val, sig), colVal, 1)
                    end
                end

                -- 底部狀態列
                sFR(pfdX, statY, sw - 8, statH, 0x1E2432)
                sR(pfdX, statY, sw - 8, statH, 0x3C4B64)
                local maxStatChars = math.max(6, math.floor((sw - 20) / 6))
                sTxt(pfdX + 4, statY + 3, string.format("STATUS: %s", FlightCore.state.statusMsg:sub(1, maxStatChars)), 0x50E6FF, 1)

                -- 底部 2 排按鈕 (高度充足防重疊)
                local bW4 = math.floor((sw - 8 - 9) / 4)
                addBtn(pfdX, btnAreaY, bW4, btnRowH, "+10", 0x1B5E20, 0xFFFFFF, function() FlightCore.adjustTargetAlt(10) end)
                addBtn(pfdX + bW4 + 3, btnAreaY, bW4, btnRowH, "-10", 0xC62828, 0xFFFFFF, function() FlightCore.adjustTargetAlt(-10) end)
                addBtn(pfdX + (bW4 + 3)*2, btnAreaY, bW4, btnRowH, "B+", 0x00695C, 0xFFFFFF, function() FlightCore.adjustBaseThrottle(1.0) end)
                addBtn(pfdX + (bW4 + 3)*3, btnAreaY, bW4, btnRowH, "B-", 0x37474F, 0xFFFFFF, function() FlightCore.adjustBaseThrottle(-1.0) end)

                local bY2 = btnAreaY + btnRowH + btnSpacing
                local bW2 = math.floor((sw - 8 - 3) / 2)
                local holdBg = (FlightCore.state.mode == "HOLD_ALT") and 0x2E7D32 or 0x1B5E20
                addBtn(pfdX, bY2, bW2, btnRowH, "HOLD", holdBg, 0xFFFFFF, function() FlightCore.holdAltitude() end)
                local stopBg = (FlightCore.state.mode == "IDLE") and 0xC62828 or 0xB71C1C
                addBtn(pfdX + bW2 + 3, bY2, bW2, btnRowH, "STOP", stopBg, 0xFFFFFF, function() FlightCore.stopEngines() end)
            end

        elseif scr.currentView == "PFD" then
            local currAlt = FlightCore.altiSensor and FlightCore.altiSensor.getHeight() or 0
            local currVspeed = FlightCore.altiSensor and FlightCore.altiSensor.getVerticalSpeed() or 0
            local currPitch, currRoll = FlightCore.getGimbalData()

            local mainH = sh - headerH - btnRowH - 12
            local mainY = headerH + 4

            local leftW = math.floor(sw * 0.52)
            sFR(6, mainY, leftW, mainH, 0x141A26)
            sR(6, mainY, leftW, mainH, 0x283850)

            local hCX = 6 + math.floor(leftW / 2)
            local hCY = mainY + math.floor(mainH / 2)
            local hR = math.min(math.floor(leftW * 0.38), math.floor(mainH * 0.40))
            sArc(hCX, hCY, hR, 0, 360, 10, 0x50AAF0)

            local rollRad = math.rad(-currRoll)
            local lx1 = math.floor(hCX - hR * 0.85 * math.cos(rollRad) + 0.5)
            local ly1 = math.floor(hCY - hR * 0.85 * math.sin(rollRad) + 0.5)
            local lx2 = math.floor(hCX + hR * 0.85 * math.cos(rollRad) + 0.5)
            local ly2 = math.floor(hCY + hR * 0.85 * math.sin(rollRad) + 0.5)
            sLS(lx1, ly1, lx2, ly2, 0xFFD700)
            if leftW >= 80 then
                sTxt(10, mainY + mainH - 12, string.format("P:%+3.0f* R:%+3.0f*", currPitch, currRoll), 0xC8E6FF, 1)
            end

            local rightX = 6 + leftW + 5
            local rightW = sw - rightX - 6
            sFR(rightX, mainY, rightW, mainH, 0x18202E)
            sR(rightX, mainY, rightW, mainH, 0x283850)

            sTxt(rightX + 4, mainY + 4, rightW < 55 and "ALT" or "CURRENT ALT", 0x8CA0B4, 1)
            local altStr = string.format("%.0fm", currAlt)
            local altSz = (isFull and rightW >= 60) and 2 or 1
            sTxt(rightX + 4, mainY + 16, altStr, 0xFFFFFF, altSz)

            local midY = mainY + (isFull and 36 or 28)
            sTxt(rightX + 4, midY, rightW < 55 and string.format("T:%.0f", FlightCore.state.targetAlt) or string.format("TGT: %.0fm", FlightCore.state.targetAlt), 0x50E6FF, 1)
            sTxt(rightX + 4, midY + 11, rightW < 55 and string.format("V:%+.1f", currVspeed) or string.format("V.S: %+.2f", currVspeed), 0xFFCD4B, 1)

            local bY = mainY + mainH + 4
            local bW = math.floor((sw - 12 - 12) / 4)
            addBtn(6, bY, bW, btnRowH, isFull and "+10m" or "+10", 0x1E642D, 0xFFFFFF, function() FlightCore.adjustTargetAlt(10) end)
            addBtn(6 + (bW+4), bY, bW, btnRowH, isFull and "-10m" or "-10", 0xBE2828, 0xFFFFFF, function() FlightCore.adjustTargetAlt(-10) end)
            local holdBg = (FlightCore.state.mode == "HOLD_ALT") and 0x2D8C3C or 0x1E5A28
            addBtn(6 + (bW+4)*2, bY, bW, btnRowH, "HOLD", holdBg, 0xFFFFFF, function() FlightCore.holdAltitude() end)
            local stopBg = (FlightCore.state.mode == "IDLE") and 0xC82D2D or 0xA01E1E
            addBtn(6 + (bW+4)*3, bY, bW, btnRowH, "STOP", stopBg, 0xFFFFFF, function() FlightCore.stopEngines() end)

        elseif scr.currentView == "ECAM" then
            local topY = headerH + 3
            local btmH = isFull and 22 or 14
            local areaH = sh - topY - btmH - 4
            local quadW = math.floor((sw - 12) / 2)
            local quadH = math.floor((areaH - 3) / 2)

            local quadDefs = {
                {slot="FL", col=1, row=1, title="[FL] FRONT-LEFT"},
                {slot="FR", col=2, row=1, title="[FR] FRONT-RIGHT"},
                {slot="BL", col=1, row=2, title="[BL] BACK-LEFT"},
                {slot="BR", col=2, row=2, title="[BR] BACK-RIGHT"}
            }

            for _, q in ipairs(quadDefs) do
                local qx = 4 + (q.col - 1) * (quadW + 4)
                local qy = topY + (q.row - 1) * (quadH + 3)
                sFR(qx, qy, quadW, quadH, 0x121822)
                sR(qx, qy, quadW, quadH, 0x283850)

                local qH = FlightCore.getQuadHealth(q.slot)
                local online = qH.online > 0
                local val = FlightCore.virtualOutputs[q.slot]

                if isFull and (quadW >= 110) then
                    local titleCol = online and 0xB4E6FF or 0xFF7878
                    sTxt(qx + 6, qy + 4, q.title, titleCol, 1)

                    local engCountStr = string.format("ENG:%d", qH.total)
                    sTxt(qx + quadW - #engCountStr * 6 - 6, qy + 4, engCountStr, 0x96BEE6, 1)

                    -- 圓形刻度盤 (Tom's GPU)
                    local dialR = math.min(math.floor(quadW * 0.28), math.floor(quadH * 0.28))
                    local engCenterX = qx + math.floor(quadW / 2)
                    local engCenterY = qy + math.floor(quadH * 0.52)
                    sArc(engCenterX, engCenterY, dialR, 135, 405, 10, 0x384860)

                    local angle = 135 + (val / 15.0) * 270
                    local rad = math.rad(angle)
                    local px = engCenterX + math.floor(dialR * 0.8 * math.cos(rad) + 0.5)
                    local py = engCenterY + math.floor(dialR * 0.8 * math.sin(rad) + 0.5)
                    sLS(engCenterX, engCenterY, px, py, 0x50FF78)

                    local thrValStr = string.format("%.1f", val)
                    sTxt(engCenterX - math.floor(#thrValStr * 3), engCenterY - 3, thrValStr, 0xFFFFFF, 1)

                    local statText = string.format("ACT:%d/%d", qH.online, qH.total)
                    if qH.total == 0 then statText = "NO ENG" end
                    local statCol = (qH.online > 0) and 0x50FF78 or 0xFF5050
                    sTxt(qx + 6, qy + quadH - 12, statText, statCol, 1)

                    local pwmText = string.format("PWM:%d", FlightCore.engineOutputs[q.slot] or 0)
                    sTxt(qx + quadW - #pwmText * 6 - 6, qy + quadH - 12, pwmText, 0x96BEE1, 1)
                else
                    -- 緊湊/直立螢幕版
                    local qTitle = string.format("[%s] %dE", q.slot, qH.total)
                    local titleCol = online and 0xB4E6FF or 0xFF7878
                    sTxt(qx + 3, qy + 3, qTitle, titleCol, 1)

                    local barY = qy + math.floor(quadH * 0.40)
                    local barW = math.max(4, quadW - 6)
                    local barH = math.max(4, math.min(10, math.floor(quadH * 0.18)))
                    sFR(qx + 3, barY, barW, barH, 0x0A1018)
                    local fillW = math.floor(barW * math.min(1.0, math.max(0.0, val / 15.0)))
                    local barCol = (val > 12) and 0xFF4500 or ((val > 8) and 0xFFD700 or 0x32CD32)
                    if fillW > 0 then sFR(qx + 3, barY, fillW, barH, barCol) end
                    sR(qx + 3, barY, barW, barH, 0x384860)

                    local btmLineY = qy + quadH - 9
                    sTxt(qx + 3, btmLineY, string.format("%.1f", val), 0x46FF78, 1)
                    local actStr = (quadW < 65) and string.format("P:%d", FlightCore.engineOutputs[q.slot] or 0) or string.format("P:%d %d/%d", FlightCore.engineOutputs[q.slot] or 0, qH.online, qH.total)
                    local actCol = (qH.online > 0) and 0x50FF78 or 0xFF5050
                    sTxt(qx + quadW - #actStr * 6 - 3, btmLineY, actStr, actCol, 1)
                end
            end

            local statY = topY + areaH + 2
            sFR(4, statY, sw - 8, btmH, 0x19202D)
            sR(4, statY, sw - 8, btmH, 0x3C4B64)
            if isFull then
                sTxt(8, statY + 4, string.format("BASE: %4.2f/15 | BALANCED | STATUS: %s", FlightCore.state.baseThrottle, FlightCore.state.statusMsg:sub(1, 26)), 0x50E6FF, 1)
            else
                local statChars = math.max(6, math.floor((sw - 60) / 6))
                sTxt(6, statY + 3, string.format("B:%4.2f | %s", FlightCore.state.baseThrottle, FlightCore.state.statusMsg:sub(1, statChars)), 0x50E6FF, 1)
            end

        elseif scr.currentView == "CTRL" then
            local currAlt = FlightCore.altiSensor and FlightCore.altiSensor.getHeight() or 0
            local currVspeed = FlightCore.altiSensor and FlightCore.altiSensor.getVerticalSpeed() or 0
            local currPitch, currRoll = FlightCore.getGimbalData()

            if isFull then
                -- 大型螢幕 >=5x5 專業飛控管理中心 (Full Avionics Flight Management System)
                local cardW = math.floor((sw - 16) / 2)
                local c1X = 6
                local c2X = c1X + cardW + 4

                -- 卡片 1: 飛行高度與升降剖面
                sFR(c1X, instY, cardW, instH, 0x141E2B)
                sR(c1X, instY, cardW, instH, 0x284864)
                sTxt(c1X + 6, instY + 4, "ALTITUDE PROFILE", 0x8CA0B4, 1)
                sTxt(c1X + 8, instY + 18, string.format("%.1fm", currAlt), 0xFFFFFF, 2)

                local textOffY = instY + 36
                local rowSp = math.max(12, math.floor((instH - 42) / 3))
                sTxt(c1X + 8, textOffY, string.format("TARGET: %5.1fm (DIFF:%+5.1fm)", FlightCore.state.targetAlt, FlightCore.state.targetAlt - currAlt), 0x50E6FF, 1)
                local vsCol = (math.abs(currVspeed) < 0.2) and 0x50FF78 or 0xFFCD4B
                sTxt(c1X + 8, textOffY + rowSp, string.format("V.SPEED: %+5.2f m/s", currVspeed), vsCol, 1)
                local mCol = (FlightCore.state.mode == "HOLD_ALT") and 0x50FF78 or ((FlightCore.state.mode == "CALIBRATING") and 0xFFCD4B or 0xFF5050)
                sTxt(c1X + 8, textOffY + rowSp * 2, string.format("MODE: [%s]", FlightCore.state.mode), mCol, 1)

                -- 卡片 2: 姿態平衡與推力總覽
                sFR(c2X, instY, cardW, instH, 0x141E2B)
                sR(c2X, instY, cardW, instH, 0x284864)
                sTxt(c2X + 6, instY + 4, "ATTITUDE & PROPULSION", 0x8CA0B4, 1)
                sTxt(c2X + 8, instY + 18, string.format("BASE: %4.2f / 15.0", FlightCore.state.baseThrottle), 0x50FF78, 2)
                sTxt(c2X + 8, textOffY, string.format("GYRO: P:%+4.1f*  R:%+4.1f*", currPitch, currRoll), 0xB4DCFF, 1)

                local quadSummary = string.format("FL:%.1f FR:%.1f BL:%.1f BR:%.1f", FlightCore.virtualOutputs.FL, FlightCore.virtualOutputs.FR, FlightCore.virtualOutputs.BL, FlightCore.virtualOutputs.BR)
                sTxt(c2X + 8, textOffY + rowSp, quadSummary, 0xFFCD4B, 1)
                sTxt(c2X + 8, textOffY + rowSp * 2, "SYSTEM: BALANCED & SYNCED", 0x50E6FF, 1)

                -- 中段狀態列
                sFR(6, statY, sw - 12, statH, 0x1E2432)
                sR(6, statY, sw - 12, statH, 0x3C4B64)
                sTxt(10, statY + 4, string.format("STATUS: %s", FlightCore.state.statusMsg:sub(1, 45)), 0x50E6FF, 1)

                -- 下半部控制按鈕群 (3 組分類)
                local bW1 = math.floor((sw - 12 - 20) / 6)
                addBtn(6, btnAreaY, bW1, btnRowH, "+50m", 0x145A32, 0xFFFFFF, function() FlightCore.adjustTargetAlt(50) end)
                addBtn(6 + (bW1+4), btnAreaY, bW1, btnRowH, "+10m", 0x1B5E20, 0xFFFFFF, function() FlightCore.adjustTargetAlt(10) end)
                addBtn(6 + (bW1+4)*2, btnAreaY, bW1, btnRowH, "+1m", 0x2E7D32, 0xFFFFFF, function() FlightCore.adjustTargetAlt(1) end)
                addBtn(6 + (bW1+4)*3, btnAreaY, bW1, btnRowH, "-1m", 0xE65100, 0xFFFFFF, function() FlightCore.adjustTargetAlt(-1) end)
                addBtn(6 + (bW1+4)*4, btnAreaY, bW1, btnRowH, "-10m", 0xC62828, 0xFFFFFF, function() FlightCore.adjustTargetAlt(-10) end)
                addBtn(6 + (bW1+4)*5, btnAreaY, bW1, btnRowH, "LOCK", 0x00838F, 0xFFFFFF, function() FlightCore.lockCurrentAlt() end)

                local r2Y = btnAreaY + btnRowH + btnSpacing
                local bW2 = math.floor((sw - 12 - 12) / 4)
                addBtn(6, r2Y, bW2, btnRowH, "BASE +1.0", 0x00695C, 0xFFFFFF, function() FlightCore.adjustBaseThrottle(1.0) end)
                addBtn(6 + (bW2+4), r2Y, bW2, btnRowH, "BASE -1.0", 0x37474F, 0xFFFFFF, function() FlightCore.adjustBaseThrottle(-1.0) end)
                addBtn(6 + (bW2+4)*2, r2Y, bW2, btnRowH, "BASE +0.1", 0x00897B, 0xFFFFFF, function() FlightCore.adjustBaseThrottle(0.1) end)
                addBtn(6 + (bW2+4)*3, r2Y, bW2, btnRowH, "BASE -0.1", 0x455A64, 0xFFFFFF, function() FlightCore.adjustBaseThrottle(-0.1) end)

                local r3Y = r2Y + btnRowH + btnSpacing
                local bW3 = math.floor((sw - 12 - 12) / 4)
                local holdBg = (FlightCore.state.mode == "HOLD_ALT") and 0x2E7D32 or 0x1B5E20
                addBtn(6, r3Y, bW3, btnRowH, "[ HOLD ALT ]", holdBg, 0xFFFFFF, function() FlightCore.holdAltitude() end)
                addBtn(6 + (bW3+4), r3Y, bW3, btnRowH, "[ CALIB 200m ]", 0x6A1B9A, 0xFFFFFF, function() FlightCore.startCalibration() end)
                addBtn(6 + (bW3+4)*2, r3Y, bW3, btnRowH, "[ RE-SCAN HW ]", 0x1565C0, 0xFFFFFF, function() FlightCore.scanQuadTurtles(); FlightCore.state.statusMsg="Re-scanned!" end)
                local stopBg = (FlightCore.state.mode == "IDLE") and 0xC62828 or 0xB71C1C
                addBtn(6 + (bW3+4)*3, r3Y, bW3, btnRowH, "[ STOP IDLE ]", stopBg, 0xFFFFFF, function() FlightCore.stopEngines() end)
            else
                -- 緊湊模式 (<5x5 Compact Mode)
                local topH = math.max(16, math.min(22, math.floor(instH * 0.28)))
                sFR(6, instY, sw - 12, topH, 0x141C2A)
                sR(6, instY, sw - 12, topH, 0x284864)
                local mCol = (FlightCore.state.mode == "HOLD_ALT") and 0x50FF78 or 0xFF5050
                if sw < 160 then
                    sTxt(8, instY + 3, string.format("[%s] A:%.0f->%.0f", FlightCore.state.mode:sub(1,4), currAlt, FlightCore.state.targetAlt), 0xC8E6FF, 1)
                else
                    sTxt(10, instY + 4, string.format("[%s]", FlightCore.state.mode:sub(1,6)), mCol, 1)
                    sTxt(65, instY + 4, string.format("ALT:%.0f->%.0f | B:%.2f", currAlt, FlightCore.state.targetAlt, FlightCore.state.baseThrottle), 0xC8E6FF, 1)
                end

                local ctrlRowH = math.max(12, math.min(24, math.floor((sh - instY - topH - 12) / 3)))
                local gridY = instY + topH + 4

                -- Row 1: Target Altitude
                local bW1 = math.floor((sw - 12 - 12) / 4)
                addBtn(6, gridY, bW1, ctrlRowH, "+10m", 0x1E642D, 0xFFFFFF, function() FlightCore.adjustTargetAlt(10) end)
                addBtn(6 + (bW1+4), gridY, bW1, ctrlRowH, "+1m", 0x2D8237, 0xFFFFFF, function() FlightCore.adjustTargetAlt(1) end)
                addBtn(6 + (bW1+4)*2, gridY, bW1, ctrlRowH, "-1m", 0xD25F14, 0xFFFFFF, function() FlightCore.adjustTargetAlt(-1) end)
                addBtn(6 + (bW1+4)*3, gridY, bW1, ctrlRowH, "-10m", 0xBE2828, 0xFFFFFF, function() FlightCore.adjustTargetAlt(-10) end)

                -- Row 2: Base Throttle
                local r2Y = gridY + ctrlRowH + 3
                local bW2 = math.floor((sw - 12 - 12) / 4)
                addBtn(6, r2Y, bW2, ctrlRowH, "B +1", 0x006E5F, 0xFFFFFF, function() FlightCore.adjustBaseThrottle(1.0) end)
                addBtn(6 + (bW2+4), r2Y, bW2, ctrlRowH, "B -1", 0x374650, 0xFFFFFF, function() FlightCore.adjustBaseThrottle(-1.0) end)
                addBtn(6 + (bW2+4)*2, r2Y, bW2, ctrlRowH, "B +.1", 0x14786E, 0xFFFFFF, function() FlightCore.adjustBaseThrottle(0.1) end)
                addBtn(6 + (bW2+4)*3, r2Y, bW2, ctrlRowH, "B -.1", 0x46505A, 0xFFFFFF, function() FlightCore.adjustBaseThrottle(-0.1) end)

                -- Row 3: Flight Ops
                local r3Y = r2Y + ctrlRowH + 3
                local bW3 = math.floor((sw - 12 - 12) / 4)
                local holdBg = (FlightCore.state.mode == "HOLD_ALT") and 0x2D8C3C or 0x1E5A28
                addBtn(6, r3Y, bW3, ctrlRowH, "HOLD", holdBg, 0xFFFFFF, function() FlightCore.holdAltitude() end)
                addBtn(6 + (bW3+4), r3Y, bW3, ctrlRowH, "CALIB", 0x6E1E9B, 0xFFFFFF, function() FlightCore.startCalibration() end)
                addBtn(6 + (bW3+4)*2, r3Y, bW3, ctrlRowH, "RE-SCAN", 0x1964BE, 0xFFFFFF, function() FlightCore.scanQuadTurtles(); FlightCore.state.statusMsg="Re-scanned!" end)
                local stopBg = (FlightCore.state.mode == "IDLE") and 0xC82D2D or 0xA01E1E
                addBtn(6 + (bW3+4)*3, r3Y, bW3, ctrlRowH, "STOP", stopBg, 0xFFFFFF, function() FlightCore.stopEngines() end)
            end

        elseif scr.currentView == "NAV" then
            local currAlt = FlightCore.altiSensor and FlightCore.altiSensor.getHeight() or 0
            local currVspeed = FlightCore.altiSensor and FlightCore.altiSensor.getVerticalSpeed() or 0

            local statNavH = isFull and 26 or 16
            sFR(6, headerH + 4, sw - 12, statNavH, 0x141C28)
            sR(6, headerH + 4, sw - 12, statNavH, 0x284864)
            if sw < 160 then
                sTxt(8, headerH + 6, string.format("ALT:%.0f TGT:%.0f V:%+.1f", currAlt, FlightCore.state.targetAlt, currVspeed), 0x50E6FF, 1)
            else
                sTxt(10, headerH + 6, string.format("ALT: %.0fm -> TGT: %.0fm (V.S: %+.1f)", currAlt, FlightCore.state.targetAlt, currVspeed), 0x50E6FF, 1)
            end

            local gridY = headerH + 4 + statNavH + 4
            local btnAreaH = sh - gridY - 4
            local rowH = math.floor((btnAreaH - 8) / 3)
            local colW = math.floor((sw - 12 - 6) / 2)

            addBtn(6, gridY, colW, rowH, isFull and "[ 0m LANDING ]" or "0m LAND", 0xA03232, 0xFFFFFF, function() FlightCore.setTargetAlt(0) end)
            addBtn(6 + colW + 6, gridY, colW, rowH, isFull and "[ 80m TREETOP ]" or "80m TREE", 0x236E3C, 0xFFFFFF, function() FlightCore.setTargetAlt(80) end)

            addBtn(6, gridY + rowH + 4, colW, rowH, isFull and "[ 150m CRUISE ]" or "150m CRZ", 0x195A8C, 0xFFFFFF, function() FlightCore.setTargetAlt(150) end)
            addBtn(6 + colW + 6, gridY + rowH + 4, colW, rowH, isFull and "[ 200m CALIBRATE ]" or "200m CAL", 0x64288C, 0xFFFFFF, function() FlightCore.setTargetAlt(200) end)

            addBtn(6, gridY + (rowH + 4)*2, colW, rowH, isFull and "[ 300m HIGH-ALT ]" or "300m HIGH", 0x146E96, 0xFFFFFF, function() FlightCore.setTargetAlt(300) end)
            addBtn(6 + colW + 6, gridY + (rowH + 4)*2, colW, rowH, isFull and "[ LOCK CURRENT ]" or "LOCK CURR", 0x825A14, 0xFFFFFF, function() FlightCore.lockCurrentAlt() end)

        elseif scr.currentView == "SYS" then
            local cardW = math.floor((sw - 18) / 2)
            local btmSysH = btnRowH
            local btmY = sh - btmSysH - 4
            local cardH = math.floor((btmY - topMargin - 6) / 2)
            local startY = topMargin

            local slots = {
                {slot="FL", col=1, row=1, name="FL Quad"},
                {slot="FR", col=2, row=1, name="FR Quad"},
                {slot="BL", col=1, row=2, name="BL Quad"},
                {slot="BR", col=2, row=2, name="BR Quad"}
            }

            for _, s in ipairs(slots) do
                local cx = 6 + (s.col - 1) * (cardW + 6)
                local cy = startY + (s.row - 1) * (cardH + 4)
                local qH = FlightCore.getQuadHealth(s.slot)
                local online = qH.online > 0
                sFR(cx, cy, cardW, cardH, online and 0x14231E or 0x281414)
                sR(cx, cy, cardW, cardH, online and 0x28643C or 0x642828)

                local tagCol = online and 0x50FF78 or 0xFF5050
                sTxt(cx + 4, cy + 4, string.format("[%s] %d ENG", s.slot, qH.total), 0xDCEDFF, 1)
                local actStr = (cardW < 75) and string.format("ON:%d P:%d", qH.online, FlightCore.engineOutputs[s.slot] or 0) or string.format("ACT:%d/%d | P:%d", qH.online, qH.total, FlightCore.engineOutputs[s.slot] or 0)
                sTxt(cx + 4, cy + math.max(10, math.floor(cardH * 0.45)), actStr, tagCol, 1)
            end

            local btmW = math.floor((sw - 16) / 2)
            addBtn(6, btmY, btmW, btmSysH, isFull and "[ RE-SCAN HARDWARE ]" or "RE-SCAN HW", 0x1964BE, 0xFFFFFF, function() FlightCore.scanQuadTurtles(); FlightCore.state.statusMsg="Re-scanned!" end)
            addBtn(6 + btmW + 4, btmY, btmW, btmSysH, isFull and "[ STOP ALL ENGINES ]" or "STOP ALL", 0xBE2828, 0xFFFFFF, function() FlightCore.stopEngines() end)
        end

        local btnTextSize = 1
        for _, btn in ipairs(scr.buttons) do
            sFR(btn.x, btn.y, btn.w, btn.h, btn.bg)
            sR(btn.x, btn.y, btn.w, btn.h, 0x8CA0B4)
            local tx = btn.x + math.max(1, math.floor((btn.w - #btn.text * 6 * btnTextSize) / 2))
            local ty = btn.y + math.floor((btn.h - 7 * btnTextSize) / 2)
            sTxt(tx, ty, btn.text, btn.fg, btnTextSize)
        end

        pcall(function() if scr.gpu.sync then scr.gpu.sync() end end)
    end,
    handleEvent = function(self, event, p1, p2, p3, p4)
        if event == "peripheral" or event == "peripheral_detach" or event == "monitor_resize" or event == "tm_monitor_resize" then
            self:refreshScreens()
            return
        end

        local targetScreen = nil
        local clickX, clickY = nil, nil

        if event == "tm_monitor_touch" then
            for _, scr in ipairs(self.screens) do
                if scr.id == p1 then targetScreen = scr; break end
            end
            if targetScreen and type(p2) == "number" and type(p3) == "number" then
                clickX, clickY = p2, p3
            elseif type(p1) == "number" and type(p2) == "number" then
                targetScreen = self.screens[1]
                clickX, clickY = p1, p2
            end

        elseif event == "monitor_touch" then
            for _, scr in ipairs(self.screens) do
                if scr.id == p1 then targetScreen = scr; break end
            end
            if not targetScreen and #self.screens > 0 then targetScreen = self.screens[1] end
            if targetScreen and type(p2) == "number" and type(p3) == "number" then
                local mw, mh = 50, 19
                local mon = peripheral.wrap(p1)
                if mon and mon.getSize then
                    local ok, w, h = pcall(function() return mon.getSize() end)
                    if ok and w and h and w > 0 and h > 0 then mw, mh = w, h end
                end
                clickX = math.floor(((p2 - 0.5) / mw) * (targetScreen.screenW or 320))
                clickY = math.floor(((p3 - 0.5) / mh) * (targetScreen.screenH or 240))
            end

        elseif event == "mouse_click" then
            targetScreen = self.screens[1]
            if targetScreen and type(p2) == "number" and type(p3) == "number" then
                local tw, th = term.getSize()
                clickX = math.floor(((p2 - 0.5) / tw) * (targetScreen.screenW or 320))
                clickY = math.floor(((p3 - 0.5) / th) * (targetScreen.screenH or 240))
            end
        end

        if targetScreen and clickX and clickY then
            for _, btn in ipairs(targetScreen.buttons) do
                if clickX >= btn.x and clickX <= btn.x + btn.w and clickY >= btn.y and clickY <= btn.y + btn.h then
                    pcall(btn.action)
                    self:drawScreen(targetScreen)
                    break
                end
            end
        end
    end
}

return Driver
