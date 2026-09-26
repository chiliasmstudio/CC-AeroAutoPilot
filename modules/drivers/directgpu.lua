--[[
    modules/drivers/directgpu.lua
    CC-DirectGPU-Mod Driver (High-Performance Hardware-Accelerated RGB Renderer)
--]]

local function safeRequire(mod)
    local ok, res = pcall(require, mod)
    if ok then return res end
    return nil
end

local FlightCore = package.loaded["modules.flight_core"] or safeRequire("modules.flight_core") or _G.FlightCore

local VIEW_TITLES = {
    OVERVIEW = "FLIGHT MONITOR",
    PFD      = "PRIMARY FLIGHT",
    ECAM     = "ECAM 2x2",
    CTRL     = "FLIGHT CONTROLS",
    NAV      = "HEADING & NAV",
    SYS      = "SYSTEM STATUS"
}

local Driver = {
    screens = {},
    refreshScreens = function(self)
        local existing = {}
        for _, scr in ipairs(self.screens) do
            if scr.key then existing[scr.key] = scr end
        end
        local newScreens = {}
        for _, name in ipairs(peripheral.getNames()) do
            if peripheral.getType(name) == "directgpu" then
                local gpu = peripheral.wrap(name)
                if gpu then
                    local count = 1
                    pcall(function() if gpu.getDisplayCount then count = gpu.getDisplayCount() end end)
                    for dId = 0, count - 1 do
                        local key = name .. "#" .. tostring(dId)
                        local info = {pixelWidth=320, pixelHeight=240}
                        pcall(function() if gpu.getDisplayInfo then info = gpu.getDisplayInfo(dId) or info end end)
                        local prev = existing[key]
                        table.insert(newScreens, {
                            key = key,
                            name = name,
                            gpu = gpu,
                            displayId = dId,
                            screenW = info.pixelWidth or 320,
                            screenH = info.pixelHeight or 240,
                            currentView = prev and prev.currentView or "OVERVIEW",
                            isMenuOpen = prev and prev.isMenuOpen or false,
                            buttons = prev and prev.buttons or {}
                        })
                    end
                end
            end
        end
        self.screens = newScreens
        return #self.screens > 0
    end,
    init = function(self)
        return self:refreshScreens()
    end,
    drawA350Dial = function(self, scr, cx, cy, r, val, maxVal, label, sig, slot)
        local gpu = scr.gpu
        local dispId = scr.displayId
        local labelSize = math.max(9, math.floor(r * 0.44))
        local lblOffset = math.floor(#label * (labelSize * 0.32))
        gpu.drawText(dispId, label, cx - lblOffset, cy - r - math.floor(labelSize * 1.1), 200, 230, 255, "Arial", labelSize, "bold")

        local arcPts, arcPtsOuter = {}, {}
        for deg = 210, -30, -10 do
            local rad = math.rad(deg)
            table.insert(arcPts, { math.floor(cx + r * math.cos(rad) + 0.5), math.floor(cy - r * math.sin(rad) + 0.5) })
            table.insert(arcPtsOuter, { math.floor(cx + (r + 1) * math.cos(rad) + 0.5), math.floor(cy - (r + 1) * math.sin(rad) + 0.5) })
        end
        gpu.drawPolylines(dispId, arcPts, 160, 180, 205)
        gpu.drawPolylines(dispId, arcPtsOuter, 120, 140, 165)

        for _, deg in ipairs({210, 90, -30}) do
            local rad = math.rad(deg)
            gpu.drawLine(dispId, math.floor(cx + (r - 3) * math.cos(rad) + 0.5), math.floor(cy - (r - 3) * math.sin(rad) + 0.5),
                                           math.floor(cx + (r + 5) * math.cos(rad) + 0.5), math.floor(cy - (r + 5) * math.sin(rad) + 0.5), 180, 200, 225)
        end

        local redPts = {}
        for deg = 10, -30, -8 do
            local rad = math.rad(deg)
            table.insert(redPts, { math.floor(cx + r * math.cos(rad) + 0.5), math.floor(cy - r * math.sin(rad) + 0.5) })
        end
        gpu.drawPolylines(dispId, redPts, 255, 55, 55)

        local ratio = math.min(1.0, math.max(0.0, val / (maxVal or 15.0)))
        local nRad = math.rad(210 - ratio * 240)
        local nx = math.floor(cx + (r - 3) * math.cos(nRad) + 0.5)
        local ny = math.floor(cy - (r - 3) * math.sin(nRad) + 0.5)
        gpu.drawLine(dispId, cx, cy, nx, ny, 50, 255, 100)
        gpu.drawCircle(dispId, cx, cy, math.max(2, math.floor(r * 0.14)), 200, 220, 240, true)

        local boxW = math.max(28, math.floor(r * 1.50))
        local boxH = math.max(12, math.floor(r * 0.58))
        local boxX = cx - math.floor(boxW / 2)
        local boxY = cy + math.floor(r * 0.28)
        gpu.fillRect(dispId, boxX, boxY, boxW, boxH, 12, 18, 28)
        gpu.drawPolylines(dispId, {{boxX, boxY}, {boxX+boxW, boxY}, {boxX+boxW, boxY+boxH}, {boxX, boxY+boxH}, {boxX, boxY}}, 40, 150, 200)

        local valStr = string.format("%4.1f", val)
        local numFontSize = math.max(9, math.floor(boxH * 0.70))
        gpu.drawText(dispId, valStr, boxX + 3, boxY + 2, 70, 255, 120, "Arial", numFontSize, "bold")

        local sigStr = string.format("PWM:%d", sig or 0)
        local sigFontSize = math.max(8, math.floor(boxH * 0.50))
        gpu.drawText(dispId, sigStr, cx - math.floor(#sigStr * 2.8), boxY + boxH + 2, 150, 190, 225, "Arial", sigFontSize, "plain")
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
        local gpu = scr.gpu
        local dispId = scr.displayId
        local ok, dispInfo = pcall(function() return gpu.getDisplayInfo(dispId) end)
        if ok and dispInfo and dispInfo.pixelWidth and dispInfo.pixelHeight and dispInfo.pixelWidth > 0 and dispInfo.pixelHeight > 0 then
            scr.screenW = dispInfo.pixelWidth
            scr.screenH = dispInfo.pixelHeight
        end
        local sw = scr.screenW or 320
        local sh = scr.screenH or 240
        local isFull = (sw >= 750 and sh >= 750) -- 5x5 (820x820) 或以上為完整顯示，3x3/4x4/5x4/4x5 (<=656) 為精簡版

        gpu.clear(dispId, 15, 20, 30)
        scr.buttons = {}

        local function addBtn(x, y, w, h, text, bg, fg, act)
            table.insert(scr.buttons, {x=x, y=y, w=w, h=h, text=text, bg=bg, fg=fg, action=act})
        end

        local headerH = isFull and math.max(26, math.min(36, math.floor(sh * 0.06))) or (sh < 220 and 14 or 20)
        gpu.fillRect(dispId, 0, 0, sw, headerH, 25, 40, 65)

        local menuBtnW = isFull and 54 or (sw < 220 and 18 or 24)
        local menuBtnH = headerH - 4
        local menuBtnX = sw - menuBtnW - 3
        local menuBtnY = 2
        if scr.isMenuOpen then
            addBtn(menuBtnX, menuBtnY, menuBtnW, menuBtnH, "[X]", {190, 45, 45}, {255, 255, 255}, function() scr.isMenuOpen = false end)
        else
            addBtn(menuBtnX, menuBtnY, menuBtnW, menuBtnH, isFull and "[MENU]" or "[=]", {35, 80, 150}, {255, 255, 255}, function() scr.isMenuOpen = true end)
        end

        local titleFontSize = isFull and math.max(10, math.min(14, math.floor(headerH * 0.48))) or (sh < 220 and 8 or 9)
        local viewTitle = scr.isMenuOpen and "SELECT VIEW" or (isFull and (VIEW_TITLES[scr.currentView] or scr.currentView) or scr.currentView)
        gpu.drawText(dispId, viewTitle, 8, math.floor((headerH - titleFontSize) / 2), 240, 245, 255, "Arial", titleFontSize, "bold")

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

        if scr.isMenuOpen then
            local cardW = math.floor((sw - 20) / 2)
            local cardH = math.floor((sh - headerH - 20) / 3)
            local startY = headerH + 5

            local function addMenuCard(col, row, title, targetView, color)
                local cx = 6 + (col - 1) * (cardW + 8)
                local cy = startY + (row - 1) * (cardH + 5)
                addBtn(cx, cy, cardW, cardH, title, color, {255, 255, 255}, function()
                    scr.currentView = targetView
                    scr.isMenuOpen = false
                end)
            end

            local shortTitle = cardW < 65
            addMenuCard(1, 1, isFull and "1. OVERVIEW (ALL)" or (shortTitle and "1. OVR" or "1. OVERVIEW"), "OVERVIEW", {25, 60, 95})
            addMenuCard(2, 1, isFull and "2. PFD (FLIGHT)" or (shortTitle and "2. PFD" or "2. PFD"), "PFD", {20, 90, 50})
            addMenuCard(1, 2, isFull and "3. ECAM (QUAD ENG)" or (shortTitle and "3. ECAM" or "3. ECAM"), "ECAM", {120, 40, 30})
            addMenuCard(2, 2, isFull and "4. CTRL (CONTROLS)" or (shortTitle and "4. CTRL" or "4. CTRL"), "CTRL", {18, 120, 100})
            addMenuCard(1, 3, isFull and "5. NAV (HEADING)" or (shortTitle and "5. NAV" or "5. NAV"), "NAV", {35, 115, 165})
            addMenuCard(2, 3, isFull and "6. SYS (DIAGNOSE)" or (shortTitle and "6. SYS" or "6. SYS"), "SYS", {80, 45, 95})

        elseif scr.currentView == "OVERVIEW" then
            local currAlt = FlightCore.altiSensor and FlightCore.altiSensor.getHeight() or 0
            local currVspeed = FlightCore.altiSensor and FlightCore.altiSensor.getVerticalSpeed() or 0
            local currPitch, currRoll = FlightCore.getGimbalData()

            if isFull then
                -- 5x5 或以上：超大高解析度完整儀表 (Full Avionics Dashboard)
                local leftW = math.max(140, math.floor(sw * 0.38))
                local pfdX = 6
                local ecamX = pfdX + leftW + 6
                local ecamW = sw - ecamX - 6

                -- 左側卡片: 主飛行儀表 (PFD)
                gpu.fillRect(dispId, pfdX, instY, leftW, instH, 20, 26, 38)
                gpu.drawText(dispId, "PRIMARY FLIGHT", pfdX + 6, instY + 4, 170, 210, 230, "Arial", 11, "bold")

                local gR = math.max(16, math.min(36, math.floor(instH * 0.26), math.floor(leftW * 0.24)))
                local gCX = pfdX + math.floor(leftW * 0.25)
                local gCY = instY + math.floor(instH * 0.54)
                gpu.drawCircle(dispId, gCX, gCY, gR, 80, 170, 240, true)
                local altStr = string.format("%.0f", currAlt)
                local altSz = (gR >= 22) and 13 or 10
                gpu.drawText(dispId, altStr, gCX - math.floor(#altStr * 4), gCY - 6, 255, 255, 255, "Arial", altSz, "bold")

                local textX = pfdX + math.floor(leftW * 0.52)
                local rowSpacing = math.max(12, math.floor((instH - 24) / 4))
                gpu.drawText(dispId, string.format("TGT: %.0fm", FlightCore.state.targetAlt), textX, instY + 14, 80, 230, 255, "Arial", 10, "bold")
                gpu.drawText(dispId, string.format("V.S: %+.2f", currVspeed), textX, instY + 14 + rowSpacing, 255, 205, 75, "Arial", 10, "bold")
                local mCol = (FlightCore.state.mode == "HOLD_ALT") and {80, 255, 120} or {255, 80, 80}
                gpu.drawText(dispId, string.format("MOD: %s", FlightCore.state.mode:sub(1,6)), textX, instY + 14 + rowSpacing * 2, mCol[1], mCol[2], mCol[3], "Arial", 10, "bold")
                gpu.drawText(dispId, string.format("P:%+2.0f R:%+2.0f", currPitch, currRoll), textX, instY + 14 + rowSpacing * 3, 180, 220, 255, "Arial", 9, "plain")

                -- 右側卡片: 2x2 四象限發動機動力監控
                gpu.fillRect(dispId, ecamX, instY, ecamW, instH, 18, 24, 34)

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

                    gpu.fillRect(dispId, sx, sy, subW, subH, 24, 32, 46)
                    local slotTitle = (subW < 100) and string.format("[%s]", ms.slot) or string.format("[%s] %d ENG", ms.slot, qH.total)
                    gpu.drawText(dispId, slotTitle, sx + 4, sy + 3, 200, 230, 255, "Arial", 9, "bold")
                    local onCol = online and {80, 255, 120} or {255, 80, 80}
                    local onText = (subW < 80) and string.format("%d/%d", qH.online, qH.total) or string.format("%d/%d ON", qH.online, qH.total)
                    gpu.drawText(dispId, onText, sx + subW - math.floor(#onText * 6) - 4, sy + 3, onCol[1], onCol[2], onCol[3], "Arial", 9, "bold")

                    -- 動態油門條
                    local barY = sy + math.floor(subH * 0.38)
                    local barW = subW - 8
                    local barH = math.max(6, math.min(14, math.floor(subH * 0.20)))
                    gpu.fillRect(dispId, sx + 4, barY, barW, barH, 10, 18, 28)
                    local fillW = math.floor(barW * math.min(1.0, math.max(0.0, val / 15.0)))
                    local barCol = (val > 12) and {255, 69, 0} or ((val > 8) and {255, 215, 0} or {50, 205, 50})
                    if fillW > 0 then gpu.fillRect(dispId, sx + 4, barY, fillW, barH, barCol[1], barCol[2], barCol[3]) end

                    -- 數值指示
                    local thrText = (subW < 90) and string.format("%.1f", val) or string.format("THR: %.1f", val)
                    gpu.drawText(dispId, thrText, sx + 4, sy + subH - 11, 70, 255, 120, "Arial", 9, "bold")
                    local pwmText = string.format("PWM:%d", FlightCore.engineOutputs[ms.slot] or 0)
                    gpu.drawText(dispId, pwmText, sx + subW - math.floor(#pwmText * 6) - 4, sy + subH - 11, 150, 190, 225, "Arial", 8, "plain")
                end

                -- 中段狀態列 (包含導航簡報)
                gpu.fillRect(dispId, pfdX, statY, sw - 12, statH, 30, 36, 50)
                local navInfo = ""
                if FlightCore.nav.x then
                    navInfo = string.format("POS:[%s] X:%.0f Y:%.0f Z:%.0f | HDG:%03d* | SPD:%.1fm/s | ", FlightCore.nav.source, FlightCore.nav.x, FlightCore.nav.y or currAlt, FlightCore.nav.z, math.floor(FlightCore.nav.yaw), FlightCore.nav.speed)
                else
                    navInfo = string.format("HDG:%03d* | ", math.floor(FlightCore.nav.yaw))
                end
                gpu.drawText(dispId, navInfo .. string.format("STATUS: %s", FlightCore.state.statusMsg:sub(1,35)), pfdX + 8, statY + 4, 80, 230, 255, "Arial", 10, "bold")

                -- 底部 3 排控制按鈕群
                local bY1 = btnAreaY
                local bW1 = math.floor((sw - 12 - 20) / 6)
                addBtn(pfdX, bY1, bW1, btnRowH, "+50m", {20, 90, 50}, {255, 255, 255}, function() FlightCore.adjustTargetAlt(50) end)
                addBtn(pfdX + (bW1 + 4), bY1, bW1, btnRowH, "+10m", {27, 94, 32}, {255, 255, 255}, function() FlightCore.adjustTargetAlt(10) end)
                addBtn(pfdX + (bW1 + 4)*2, bY1, bW1, btnRowH, "+1m", {46, 125, 50}, {255, 255, 255}, function() FlightCore.adjustTargetAlt(1) end)
                addBtn(pfdX + (bW1 + 4)*3, bY1, bW1, btnRowH, "-1m", {230, 81, 0}, {255, 255, 255}, function() FlightCore.adjustTargetAlt(-1) end)
                addBtn(pfdX + (bW1 + 4)*4, bY1, bW1, btnRowH, "-10m", {198, 40, 40}, {255, 255, 255}, function() FlightCore.adjustTargetAlt(-10) end)
                addBtn(pfdX + (bW1 + 4)*5, bY1, bW1, btnRowH, "LOCK", {0, 131, 143}, {255, 255, 255}, function() FlightCore.lockCurrentAlt() end)

                local bY2 = bY1 + btnRowH + btnSpacing
                local bW2 = math.floor((sw - 12 - 12) / 4)
                addBtn(pfdX, bY2, bW2, btnRowH, "BASE +1", {0, 105, 92}, {255, 255, 255}, function() FlightCore.adjustBaseThrottle(1.0) end)
                addBtn(pfdX + bW2 + 4, bY2, bW2, btnRowH, "BASE -1", {55, 71, 79}, {255, 255, 255}, function() FlightCore.adjustBaseThrottle(-1.0) end)
                addBtn(pfdX + (bW2 + 4)*2, bY2, bW2, btnRowH, "RE-SCAN", {21, 101, 192}, {255, 255, 255}, function() FlightCore.scanQuadTurtles(); FlightCore.state.statusMsg="Re-scanned!" end)
                addBtn(pfdX + (bW2 + 4)*3, bY2, bW2, btnRowH, "CALIB", {106, 27, 154}, {255, 255, 255}, function() FlightCore.startCalibration() end)

                local bY3 = bY2 + btnRowH + btnSpacing
                local bW3 = math.floor((sw - 12 - 4) / 2)
                local holdBg = (FlightCore.state.mode == "HOLD_ALT") and {46, 125, 50} or {27, 94, 32}
                addBtn(pfdX, bY3, bW3, btnRowH, "[ HOLD ALTITUDE ]", holdBg, {255, 255, 255}, function() FlightCore.holdAltitude() end)
                local stopBg = (FlightCore.state.mode == "IDLE") and {198, 40, 40} or {183, 28, 28}
                addBtn(pfdX + bW3 + 4, bY3, bW3, btnRowH, "[ STOP / IDLE ]", stopBg, {255, 255, 255}, function() FlightCore.stopEngines() end)
            else
                -- 3x3 ~ 4x5 精簡模式 (<5x5 Compact Mode, 適配 384x384 到 512x640 及小螢幕)
                local colW = math.floor((sw - 12) / 2)
                local pfdX = 4
                local ecamX = pfdX + colW + 4

                if colW >= 85 then
                    -- 寬度充足精簡版 (4x4, 5x4): 圓形高度錶 + 2x2 四象限進度條
                    gpu.fillRect(dispId, pfdX, instY, colW, instH, 20, 26, 38)
                    gpu.drawText(dispId, "ALT/ATT", pfdX + 4, instY + 3, 170, 210, 230, "Arial", 9, "bold")

                    local gR = math.max(8, math.min(22, math.floor(colW * 0.18), math.floor(instH * 0.22)))
                    local gCX = pfdX + math.max(gR + 4, math.floor(colW * 0.24))
                    local gCY = instY + math.floor(instH * 0.54)
                    gpu.drawCircle(dispId, gCX, gCY, gR, 80, 170, 240, true)
                    local altStr = string.format("%.0f", currAlt)
                    gpu.drawText(dispId, altStr, gCX - math.floor(#altStr * 4), gCY - 4, 255, 255, 255, "Arial", 9, "bold")

                    local textX = gCX + gR + 6
                    local rowSpacing = math.max(8, math.min(18, math.floor((instH - 18) / 4)))
                    gpu.drawText(dispId, string.format("T:%.0f", FlightCore.state.targetAlt), textX, instY + 10, 80, 230, 255, "Arial", 9, "bold")
                    gpu.drawText(dispId, string.format("V:%+.1f", currVspeed), textX, instY + 10 + rowSpacing, 255, 205, 75, "Arial", 9, "bold")
                    local mCol = (FlightCore.state.mode == "HOLD_ALT") and {80, 255, 120} or {255, 80, 80}
                    gpu.drawText(dispId, string.format("M:%s", FlightCore.state.mode:sub(1,4)), textX, instY + 10 + rowSpacing * 2, mCol[1], mCol[2], mCol[3], "Arial", 9, "bold")
                    gpu.drawText(dispId, string.format("P:%+.0f", currPitch), textX, instY + 10 + rowSpacing * 3, 180, 220, 255, "Arial", 8, "plain")

                    -- 右側: 2x2 Mini Quad Engines
                    gpu.fillRect(dispId, ecamX, instY, colW, instH, 18, 24, 34)
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

                        gpu.fillRect(dispId, sx, sy, subW, subH, 24, 32, 46)
                        gpu.drawText(dispId, string.format("[%s] %dE", ms.slot, qH.total), sx + 2, sy + 2, 200, 230, 255, "Arial", 8, "bold")

                        -- 動態微型進度條 (帶清晰邊框)
                        local barY = sy + math.floor(subH * 0.40)
                        local barW = math.max(4, subW - 6)
                        local barH = math.max(4, math.min(10, math.floor(subH * 0.18)))
                        gpu.fillRect(dispId, sx + 3, barY, barW, barH, 10, 16, 24)
                        local fillW = math.floor(barW * math.min(1.0, math.max(0.0, val / 15.0)))
                        local barCol = (val > 12) and {255, 69, 0} or ((val > 8) and {255, 215, 0} or {50, 205, 50})
                        if fillW > 0 then gpu.fillRect(dispId, sx + 3, barY, fillW, barH, barCol[1], barCol[2], barCol[3]) end

                        -- 數值指示
                        local btmLineY = sy + subH - 9
                        gpu.drawText(dispId, string.format("%4.1f", val), sx + 2, btmLineY, 70, 255, 120, "Arial", 8, "plain")
                        local pwmStr = string.format("P:%d", FlightCore.engineOutputs[ms.slot] or 0)
                        gpu.drawText(dispId, pwmStr, sx + subW - math.floor(#pwmStr * 5) - 2, btmLineY, 150, 190, 225, "Arial", 8, "plain")
                    end
                else
                    -- 極致窄螢幕 (3x3 螢幕, colW < 85px): 乾淨直向數據流，零碰撞
                    -- 左側: PFD 飛行姿態垂直數據堆疊
                    gpu.fillRect(dispId, pfdX, instY, colW, instH, 20, 26, 38)
                    gpu.drawText(dispId, "ALT/ATT", pfdX + 3, instY + 2, 170, 210, 230, "Arial", 8, "bold")

                    local rowSp = math.max(8, math.min(12, math.floor((instH - 14) / 4)))
                    local yBase = instY + 13
                    gpu.drawText(dispId, string.format("ALT:%4.0f", currAlt), pfdX + 3, yBase, 255, 255, 255, "Arial", 8, "bold")
                    gpu.drawText(dispId, string.format("TGT:%4.0f", FlightCore.state.targetAlt), pfdX + 3, yBase + rowSp, 80, 230, 255, "Arial", 8, "bold")
                    gpu.drawText(dispId, string.format("V.S:%+4.1f", currVspeed), pfdX + 3, yBase + rowSp * 2, 255, 205, 75, "Arial", 8, "bold")
                    local mCol = (FlightCore.state.mode == "HOLD_ALT") and {80, 255, 120} or {255, 80, 80}
                    gpu.drawText(dispId, string.format("MOD:%s", FlightCore.state.mode:sub(1, 4)), pfdX + 3, yBase + rowSp * 3, mCol[1], mCol[2], mCol[3], "Arial", 8, "bold")

                    -- 右側: ECAM 四象限發動機直向數據堆疊 (FL, FR, BL, BR 逐行排布)
                    gpu.fillRect(dispId, ecamX, instY, colW, instH, 18, 24, 34)
                    gpu.drawText(dispId, "ECAM 4Q", ecamX + 3, instY + 2, 170, 210, 230, "Arial", 8, "bold")

                    local qSlots = {"FL", "FR", "BL", "BR"}
                    for idx, slot in ipairs(qSlots) do
                        local qH = FlightCore.getQuadHealth(slot)
                        local val = FlightCore.virtualOutputs[slot]
                        local sig = FlightCore.engineOutputs[slot] or 0
                        local yPos = yBase + (idx - 1) * rowSp
                        local colVal = (val > 12) and {255, 69, 0} or ((val > 8) and {255, 215, 0} or {80, 255, 120})
                        if qH.online == 0 then colVal = {255, 80, 80} end
                        gpu.drawText(dispId, string.format("%s%4.1f P%d", slot, val, sig), ecamX + 3, yPos, colVal[1], colVal[2], colVal[3], "Arial", 8, "plain")
                    end
                end

                -- 底部狀態 (包含導航簡報)
                gpu.fillRect(dispId, pfdX, statY, sw - 8, statH, 30, 36, 50)
                local navShort = ""
                if FlightCore.nav.x then
                    navShort = string.format("[%s] X:%.0f Z:%.0f H:%03d* | ", FlightCore.nav.source, FlightCore.nav.x, FlightCore.nav.z, math.floor(FlightCore.nav.yaw))
                else
                    navShort = string.format("H:%03d* | ", math.floor(FlightCore.nav.yaw))
                end
                local maxStatChars = math.max(6, math.floor((sw - 20) / 6) - #navShort)
                gpu.drawText(dispId, navShort .. FlightCore.state.statusMsg:sub(1, maxStatChars), pfdX + 4, statY + 3, 80, 230, 255, "Arial", 8, "bold")

                -- 底部按鈕 (2 排按鈕，嚴格鎖定高度防重疊)
                local bW4 = math.floor((sw - 8 - 9) / 4)
                addBtn(pfdX, btnAreaY, bW4, btnRowH, "+10", {27, 94, 32}, {255, 255, 255}, function() FlightCore.adjustTargetAlt(10) end)
                addBtn(pfdX + bW4 + 3, btnAreaY, bW4, btnRowH, "-10", {198, 40, 40}, {255, 255, 255}, function() FlightCore.adjustTargetAlt(-10) end)
                addBtn(pfdX + (bW4 + 3)*2, btnAreaY, bW4, btnRowH, "B+", {0, 105, 92}, {255, 255, 255}, function() FlightCore.adjustBaseThrottle(1.0) end)
                addBtn(pfdX + (bW4 + 3)*3, btnAreaY, bW4, btnRowH, "B-", {55, 71, 79}, {255, 255, 255}, function() FlightCore.adjustBaseThrottle(-1.0) end)

                local bY2 = btnAreaY + btnRowH + btnSpacing
                local bW2 = math.floor((sw - 8 - 3) / 2)
                local holdBg = (FlightCore.state.mode == "HOLD_ALT") and {46, 125, 50} or {27, 94, 32}
                addBtn(pfdX, bY2, bW2, btnRowH, "HOLD", holdBg, {255, 255, 255}, function() FlightCore.holdAltitude() end)
                local stopBg = (FlightCore.state.mode == "IDLE") and {198, 40, 40} or {183, 28, 28}
                addBtn(pfdX + bW2 + 3, bY2, bW2, btnRowH, "STOP", stopBg, {255, 255, 255}, function() FlightCore.stopEngines() end)
            end

        elseif scr.currentView == "PFD" then
            local currAlt = FlightCore.altiSensor and FlightCore.altiSensor.getHeight() or 0
            local currVspeed = FlightCore.altiSensor and FlightCore.altiSensor.getVerticalSpeed() or 0
            local currPitch, currRoll = FlightCore.getGimbalData()

            local mainH = sh - headerH - btnRowH - 12
            local mainY = headerH + 4

            local leftW = math.floor(sw * 0.52)
            gpu.fillRect(dispId, 6, mainY, leftW, mainH, 20, 26, 38)

            local hCX = 6 + math.floor(leftW / 2)
            local hCY = mainY + math.floor(mainH / 2)
            local hR = math.min(math.floor(leftW * 0.38), math.floor(mainH * 0.40))
            gpu.drawCircle(dispId, hCX, hCY, hR, 80, 170, 240, false)

            local rollRad = math.rad(-currRoll)
            local lx1 = math.floor(hCX - hR * 0.85 * math.cos(rollRad) + 0.5)
            local ly1 = math.floor(hCY - hR * 0.85 * math.sin(rollRad) + 0.5)
            local lx2 = math.floor(hCX + hR * 0.85 * math.cos(rollRad) + 0.5)
            local ly2 = math.floor(hCY + hR * 0.85 * math.sin(rollRad) + 0.5)
            gpu.drawLine(dispId, lx1, ly1, lx2, ly2, 255, 215, 0)
            gpu.drawCircle(dispId, hCX, hCY, 3, 255, 80, 80, true)
            if leftW >= 80 then
                gpu.drawText(dispId, string.format("P:%+3.0f* R:%+3.0f*", currPitch, currRoll), 10, mainY + mainH - 12, 200, 230, 255, "Arial", 9, "bold")
            end

            local rightX = 6 + leftW + 5
            local rightW = sw - rightX - 6
            gpu.fillRect(dispId, rightX, mainY, rightW, mainH, 24, 32, 46)

            gpu.drawText(dispId, rightW < 55 and "ALT" or "CURRENT ALT", rightX + 4, mainY + 4, 140, 160, 180, "Arial", 8, "plain")
            local altStr = string.format("%.0fm", currAlt)
            local altSz = (isFull and rightW >= 60) and 14 or 10
            gpu.drawText(dispId, altStr, rightX + 4, mainY + 16, 255, 255, 255, "Arial", altSz, "bold")

            local midY = mainY + (isFull and 36 or 28)
            gpu.drawText(dispId, rightW < 55 and string.format("T:%.0f", FlightCore.state.targetAlt) or string.format("TGT: %.0fm", FlightCore.state.targetAlt), rightX + 4, midY, 80, 230, 255, "Arial", 8, "bold")
            gpu.drawText(dispId, rightW < 55 and string.format("V:%+.1f", currVspeed) or string.format("V.S: %+.2f", currVspeed), rightX + 4, midY + 11, 255, 205, 75, "Arial", 8, "bold")

            local bY = mainY + mainH + 4
            local bW = math.floor((sw - 12 - 12) / 4)
            addBtn(6, bY, bW, btnRowH, "+10m", {30, 100, 45}, {255, 255, 255}, function() FlightCore.adjustTargetAlt(10) end)
            addBtn(6 + (bW+4), bY, bW, btnRowH, "-10m", {190, 40, 40}, {255, 255, 255}, function() FlightCore.adjustTargetAlt(-10) end)
            local holdBg = (FlightCore.state.mode == "HOLD_ALT") and {45, 140, 60} or {30, 90, 40}
            addBtn(6 + (bW+4)*2, bY, bW, btnRowH, "HOLD", holdBg, {255, 255, 255}, function() FlightCore.holdAltitude() end)
            local stopBg = (FlightCore.state.mode == "IDLE") and {200, 45, 45} or {160, 30, 30}
            addBtn(6 + (bW+4)*3, bY, bW, btnRowH, "STOP", stopBg, {255, 255, 255}, function() FlightCore.stopEngines() end)

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
                gpu.fillRect(dispId, qx, qy, quadW, quadH, 18, 24, 34)

                local qH = FlightCore.getQuadHealth(q.slot)
                local online = qH.online > 0

                if isFull and (quadW >= 110) then
                    local titleCol = online and {180, 230, 255} or {255, 120, 120}
                    local titleStr = (quadW < 160) and string.format("[%s] QUAD", q.slot) or q.title
                    gpu.drawText(dispId, titleStr, qx + 6, qy + 4, titleCol[1], titleCol[2], titleCol[3], "Arial", 10, "bold")

                    local engCountStr = (quadW < 130) and string.format("%dE", qH.total) or string.format("ENG:%d", qH.total)
                    gpu.drawText(dispId, engCountStr, qx + quadW - math.floor(#engCountStr * 6) - 6, qy + 4, 150, 190, 230, "Arial", 9, "plain")

                    local dialR = math.min(math.floor(quadW * 0.28), math.floor(quadH * 0.28))
                    local engCenterX = qx + math.floor(quadW / 2)
                    local engCenterY = qy + math.floor(quadH * 0.52)
                    self:drawA350Dial(scr, engCenterX, engCenterY, dialR, FlightCore.virtualOutputs[q.slot], 15.0, q.slot, FlightCore.engineOutputs[q.slot], q.slot)

                    local statText = string.format("ACT:%d/%d", qH.online, qH.total)
                    if qH.total == 0 then statText = "NO ENG" end
                    local statCol = (qH.online > 0) and {80, 255, 120} or {255, 80, 80}
                    gpu.drawText(dispId, statText, qx + 6, qy + quadH - 12, statCol[1], statCol[2], statCol[3], "Arial", 9, "bold")
                else
                    -- 緊湊/直立螢幕版
                    local qTitle = string.format("[%s] %dE", q.slot, qH.total)
                    local titleCol = online and {180, 230, 255} or {255, 120, 120}
                    gpu.drawText(dispId, qTitle, qx + 3, qy + 3, titleCol[1], titleCol[2], titleCol[3], "Arial", 8, "bold")

                    local barY = qy + math.floor(quadH * 0.40)
                    local barW = math.max(4, quadW - 6)
                    local barH = math.max(4, math.min(10, math.floor(quadH * 0.18)))
                    gpu.fillRect(dispId, qx + 3, barY, barW, barH, 10, 16, 24)
                    local fillW = math.floor(barW * math.min(1.0, math.max(0.0, FlightCore.virtualOutputs[q.slot] / 15.0)))
                    local barCol = (FlightCore.virtualOutputs[q.slot] > 12) and {255, 69, 0} or ((FlightCore.virtualOutputs[q.slot] > 8) and {255, 215, 0} or {50, 205, 50})
                    if fillW > 0 then gpu.fillRect(dispId, qx + 3, barY, fillW, barH, barCol[1], barCol[2], barCol[3]) end

                    local btmLineY = qy + quadH - 9
                    gpu.drawText(dispId, string.format("%.1f", FlightCore.virtualOutputs[q.slot]), qx + 3, btmLineY, 70, 255, 120, "Arial", 8, "bold")
                    local actStr = (quadW < 65) and string.format("P:%d", FlightCore.engineOutputs[q.slot] or 0) or string.format("P:%d %d/%d", FlightCore.engineOutputs[q.slot] or 0, qH.online, qH.total)
                    local actCol = (qH.online > 0) and {80, 255, 120} or {255, 80, 80}
                    gpu.drawText(dispId, actStr, qx + quadW - math.floor(#actStr * 5) - 3, btmLineY, actCol[1], actCol[2], actCol[3], "Arial", 8, "plain")
                end
            end

            local statY = topY + areaH + 2
            gpu.fillRect(dispId, 4, statY, sw - 8, btmH, 25, 32, 45)
            if isFull then
                gpu.drawText(dispId, string.format("BASE: %4.2f/15 | BALANCED | STATUS: %s", FlightCore.state.baseThrottle, FlightCore.state.statusMsg:sub(1, 26)), 8, statY + 4, 80, 230, 255, "Arial", 9, "bold")
            else
                local statChars = math.max(6, math.floor((sw - 60) / 6))
                gpu.drawText(dispId, string.format("B:%4.2f | %s", FlightCore.state.baseThrottle, FlightCore.state.statusMsg:sub(1, statChars)), 6, statY + 3, 80, 230, 255, "Arial", 8, "bold")
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
                gpu.fillRect(dispId, c1X, instY, cardW, instH, 20, 30, 43)
                gpu.drawText(dispId, "ALTITUDE PROFILE", c1X + 6, instY + 4, 140, 160, 180, "Arial", 9, "bold")

                local altStr = string.format("%.1fm", currAlt)
                gpu.drawText(dispId, altStr, c1X + 8, instY + 18, 255, 255, 255, "Arial", 14, "bold")

                local textOffY = instY + 36
                local rowSp = math.max(12, math.floor((instH - 42) / 3))
                local tgtStr = (cardW < 170) and string.format("TGT:%.0fm (DIFF:%+.0fm)", FlightCore.state.targetAlt, FlightCore.state.targetAlt - currAlt) or string.format("TARGET: %5.1fm (DIFF:%+5.1fm)", FlightCore.state.targetAlt, FlightCore.state.targetAlt - currAlt)
                gpu.drawText(dispId, tgtStr, c1X + 8, textOffY, 80, 230, 255, "Arial", 9, "bold")
                local vsCol = (math.abs(currVspeed) < 0.2) and {80, 255, 120} or {255, 205, 75}
                gpu.drawText(dispId, string.format("V.SPEED: %+5.2f m/s", currVspeed), c1X + 8, textOffY + rowSp, vsCol[1], vsCol[2], vsCol[3], "Arial", 9, "bold")
                local mCol = (FlightCore.state.mode == "HOLD_ALT") and {80, 255, 120} or ((FlightCore.state.mode == "CALIBRATING") and {255, 205, 75} or {255, 80, 80})
                gpu.drawText(dispId, string.format("MODE: [%s]", FlightCore.state.mode), c1X + 8, textOffY + rowSp * 2, mCol[1], mCol[2], mCol[3], "Arial", 9, "bold")

                -- 卡片 2: 姿態平衡與推力總覽
                gpu.fillRect(dispId, c2X, instY, cardW, instH, 20, 30, 43)
                gpu.drawText(dispId, "ATTITUDE & PROPULSION", c2X + 6, instY + 4, 140, 160, 180, "Arial", 9, "bold")

                local baseStr = (cardW < 170) and string.format("BASE: %4.2f", FlightCore.state.baseThrottle) or string.format("BASE: %4.2f / 15.0", FlightCore.state.baseThrottle)
                gpu.drawText(dispId, baseStr, c2X + 8, instY + 18, 80, 255, 120, "Arial", 14, "bold")
                gpu.drawText(dispId, string.format("GYRO: P:%+4.1f*  R:%+4.1f*", currPitch, currRoll), c2X + 8, textOffY, 180, 220, 255, "Arial", 9, "bold")

                local quadSummary = (cardW < 170) and string.format("F:%.1f,%.1f B:%.1f,%.1f", FlightCore.virtualOutputs.FL, FlightCore.virtualOutputs.FR, FlightCore.virtualOutputs.BL, FlightCore.virtualOutputs.BR) or string.format("FL:%.1f FR:%.1f BL:%.1f BR:%.1f", FlightCore.virtualOutputs.FL, FlightCore.virtualOutputs.FR, FlightCore.virtualOutputs.BL, FlightCore.virtualOutputs.BR)
                gpu.drawText(dispId, quadSummary, c2X + 8, textOffY + rowSp, 255, 205, 75, "Arial", 9, "bold")
                local sysSummary = (cardW < 170) and "SYS: BALANCED & SYNCED" or "SYSTEM: BALANCED & SYNCED"
                gpu.drawText(dispId, sysSummary, c2X + 8, textOffY + rowSp * 2, 80, 230, 255, "Arial", 9, "bold")

                -- 中段狀態列 (包含導航簡報)
                gpu.fillRect(dispId, 6, statY, sw - 12, statH, 30, 36, 50)
                local navInfo = ""
                if FlightCore.nav.x then
                    navInfo = string.format("POS:[%s] X:%.0f Y:%.0f Z:%.0f | HDG:%03d* | ", FlightCore.nav.source, FlightCore.nav.x, FlightCore.nav.y or currAlt, FlightCore.nav.z, math.floor(FlightCore.nav.yaw))
                else
                    navInfo = string.format("HDG:%03d* | ", math.floor(FlightCore.nav.yaw))
                end
                gpu.drawText(dispId, navInfo .. string.format("STATUS: %s", FlightCore.state.statusMsg:sub(1, 35)), 10, statY + 4, 80, 230, 255, "Arial", 9, "bold")

                -- 下半部控制按鈕群 (3 組分類)
                local bW1 = math.floor((sw - 12 - 20) / 6)
                addBtn(6, btnAreaY, bW1, btnRowH, "+50m", {20, 90, 50}, {255, 255, 255}, function() FlightCore.adjustTargetAlt(50) end)
                addBtn(6 + (bW1+4), btnAreaY, bW1, btnRowH, "+10m", {27, 94, 32}, {255, 255, 255}, function() FlightCore.adjustTargetAlt(10) end)
                addBtn(6 + (bW1+4)*2, btnAreaY, bW1, btnRowH, "+1m", {46, 125, 50}, {255, 255, 255}, function() FlightCore.adjustTargetAlt(1) end)
                addBtn(6 + (bW1+4)*3, btnAreaY, bW1, btnRowH, "-1m", {230, 81, 0}, {255, 255, 255}, function() FlightCore.adjustTargetAlt(-1) end)
                addBtn(6 + (bW1+4)*4, btnAreaY, bW1, btnRowH, "-10m", {198, 40, 40}, {255, 255, 255}, function() FlightCore.adjustTargetAlt(-10) end)
                addBtn(6 + (bW1+4)*5, btnAreaY, bW1, btnRowH, "LOCK", {0, 131, 143}, {255, 255, 255}, function() FlightCore.lockCurrentAlt() end)

                local r2Y = btnAreaY + btnRowH + btnSpacing
                local bW2 = math.floor((sw - 12 - 12) / 4)
                addBtn(6, r2Y, bW2, btnRowH, "BASE +1.0", {0, 105, 92}, {255, 255, 255}, function() FlightCore.adjustBaseThrottle(1.0) end)
                addBtn(6 + (bW2+4), r2Y, bW2, btnRowH, "BASE -1.0", {55, 71, 79}, {255, 255, 255}, function() FlightCore.adjustBaseThrottle(-1.0) end)
                addBtn(6 + (bW2+4)*2, r2Y, bW2, btnRowH, "BASE +0.1", {0, 137, 123}, {255, 255, 255}, function() FlightCore.adjustBaseThrottle(0.1) end)
                addBtn(6 + (bW2+4)*3, r2Y, bW2, btnRowH, "BASE -0.1", {69, 90, 100}, {255, 255, 255}, function() FlightCore.adjustBaseThrottle(-0.1) end)

                local r3Y = r2Y + btnRowH + btnSpacing
                local bW3 = math.floor((sw - 12 - 12) / 4)
                local holdBg = (FlightCore.state.mode == "HOLD_ALT") and {46, 125, 50} or {27, 94, 32}
                addBtn(6, r3Y, bW3, btnRowH, "[ HOLD ALT ]", holdBg, {255, 255, 255}, function() FlightCore.holdAltitude() end)
                addBtn(6 + (bW3+4), r3Y, bW3, btnRowH, "[ CALIB 200m ]", {106, 27, 154}, {255, 255, 255}, function() FlightCore.startCalibration() end)
                addBtn(6 + (bW3+4)*2, r3Y, bW3, btnRowH, "[ RE-SCAN HW ]", {21, 101, 192}, {255, 255, 255}, function() FlightCore.scanQuadTurtles(); FlightCore.state.statusMsg="Re-scanned!" end)
                local stopBg = (FlightCore.state.mode == "IDLE") and {198, 40, 40} or {183, 28, 28}
                addBtn(6 + (bW3+4)*3, r3Y, bW3, btnRowH, "[ STOP IDLE ]", stopBg, {255, 255, 255}, function() FlightCore.stopEngines() end)
            else
                -- 緊湊模式 (<5x5 Compact Mode)
                local topH = math.max(16, math.min(22, math.floor(instH * 0.28)))
                gpu.fillRect(dispId, 6, instY, sw - 12, topH, 20, 28, 42)
                local mCol = (FlightCore.state.mode == "HOLD_ALT") and {80, 255, 120} or {255, 80, 80}
                if sw < 160 then
                    gpu.drawText(dispId, string.format("[%s] A:%.0f H:%03d*", FlightCore.state.mode:sub(1,4), currAlt, math.floor(FlightCore.nav.yaw)), 8, instY + 3, 200, 230, 255, "Arial", 8, "plain")
                else
                    gpu.drawText(dispId, string.format("[%s]", FlightCore.state.mode:sub(1,6)), 10, instY + 4, mCol[1], mCol[2], mCol[3], "Arial", 9, "bold")
                    local navCompact = FlightCore.nav.x and string.format("X:%.0f Z:%.0f H:%03d*", FlightCore.nav.x, FlightCore.nav.z, math.floor(FlightCore.nav.yaw)) or string.format("ALT:%.0f->%.0f H:%03d*", currAlt, FlightCore.state.targetAlt, math.floor(FlightCore.nav.yaw))
                    gpu.drawText(dispId, navCompact, 65, instY + 4, 200, 230, 255, "Arial", 9, "plain")
                end

                local ctrlRowH = math.max(12, math.min(24, math.floor((sh - instY - topH - 12) / 3)))
                local gridY = instY + topH + 4

                -- Row 1: Target Altitude
                local bW1 = math.floor((sw - 12 - 12) / 4)
                addBtn(6, gridY, bW1, ctrlRowH, "+10m", {30, 100, 45}, {240, 255, 240}, function() FlightCore.adjustTargetAlt(10) end)
                addBtn(6 + (bW1+4), gridY, bW1, ctrlRowH, "+1m", {45, 130, 55}, {240, 255, 240}, function() FlightCore.adjustTargetAlt(1) end)
                addBtn(6 + (bW1+4)*2, gridY, bW1, ctrlRowH, "-1m", {210, 95, 20}, {255, 245, 230}, function() FlightCore.adjustTargetAlt(-1) end)
                addBtn(6 + (bW1+4)*3, gridY, bW1, ctrlRowH, "-10m", {190, 40, 40}, {255, 235, 235}, function() FlightCore.adjustTargetAlt(-10) end)

                -- Row 2: Base Throttle
                local r2Y = gridY + ctrlRowH + 3
                local bW2 = math.floor((sw - 12 - 12) / 4)
                addBtn(6, r2Y, bW2, ctrlRowH, "B +1", {0, 110, 95}, {230, 250, 245}, function() FlightCore.adjustBaseThrottle(1.0) end)
                addBtn(6 + (bW2+4), r2Y, bW2, ctrlRowH, "B -1", {55, 70, 80}, {235, 240, 245}, function() FlightCore.adjustBaseThrottle(-1.0) end)
                addBtn(6 + (bW2+4)*2, r2Y, bW2, ctrlRowH, "B +.1", {20, 120, 110}, {230, 255, 250}, function() FlightCore.adjustBaseThrottle(0.1) end)
                addBtn(6 + (bW2+4)*3, r2Y, bW2, ctrlRowH, "B -.1", {70, 80, 90}, {235, 240, 245}, function() FlightCore.adjustBaseThrottle(-0.1) end)

                -- Row 3: Flight Ops
                local r3Y = r2Y + ctrlRowH + 3
                local bW3 = math.floor((sw - 12 - 12) / 4)
                local holdBg = (FlightCore.state.mode == "HOLD_ALT") and {45, 140, 60} or {30, 90, 40}
                addBtn(6, r3Y, bW3, ctrlRowH, "HOLD", holdBg, {255, 255, 255}, function() FlightCore.holdAltitude() end)
                addBtn(6 + (bW3+4), r3Y, bW3, ctrlRowH, "CALIB", {110, 30, 155}, {250, 230, 255}, function() FlightCore.startCalibration() end)
                addBtn(6 + (bW3+4)*2, r3Y, bW3, ctrlRowH, "RE-SCAN", {25, 100, 190}, {230, 245, 255}, function() FlightCore.scanQuadTurtles(); FlightCore.state.statusMsg="Re-scanned!" end)
                local stopBg = (FlightCore.state.mode == "IDLE") and {200, 45, 45} or {160, 30, 30}
                addBtn(6 + (bW3+4)*3, r3Y, bW3, ctrlRowH, "STOP", stopBg, {255, 255, 255}, function() FlightCore.stopEngines() end)
            end

        elseif scr.currentView == "NAV" then
            local currAlt = FlightCore.altiSensor and FlightCore.altiSensor.getHeight() or 0
            local currVspeed = FlightCore.altiSensor and FlightCore.altiSensor.getVerticalSpeed() or 0
            local currPitch, currRoll = FlightCore.getGimbalData()

            local statNavH = isFull and 36 or 22
            gpu.fillRect(dispId, 6, headerH + 4, sw - 12, statNavH, 20, 28, 40)

            local holdStr = FlightCore.nav.headingHold and "ON" or "OFF"

            if isFull then
                local posStr = ""
                if FlightCore.nav.x then
                    posStr = string.format("POS:[%s] X:%+6.0f Y:%4.0f Z:%+6.0f | SPD: %4.1f m/s", FlightCore.nav.source, FlightCore.nav.x, FlightCore.nav.y or currAlt, FlightCore.nav.z, FlightCore.nav.speed)
                else
                    posStr = string.format("POS:[%s] ALT: %4.0fm | GYRO: P:%+2.0f* R:%+2.0f*", FlightCore.nav.source, currAlt, currPitch, currRoll)
                end
                gpu.drawText(dispId, posStr, 10, headerH + 7, 140, 160, 180, "Arial", 10, "plain")
                local hdgStr = string.format("HDG: %03d*  |  TARGET HDG: %03d*  |  HEADING HOLD: [%s]", math.floor(FlightCore.nav.yaw), FlightCore.nav.targetHeading, holdStr)
                gpu.drawText(dispId, hdgStr, 10, headerH + 21, 80, 230, 255, "Arial", 10, "bold")
            else
                if FlightCore.nav.x and sw >= 180 then
                    gpu.drawText(dispId, string.format("[%s] X:%.0f Z:%.0f S:%.1f", FlightCore.nav.source, FlightCore.nav.x, FlightCore.nav.z, FlightCore.nav.speed), 8, headerH + 6, 140, 160, 180, "Arial", 8, "plain")
                else
                    gpu.drawText(dispId, string.format("ALT:%.0f (V:%+.1f)", currAlt, currVspeed), 8, headerH + 6, 140, 160, 180, "Arial", 8, "plain")
                end
                gpu.drawText(dispId, string.format("H:%03d* -> T:%03d* [%s]", math.floor(FlightCore.nav.yaw), FlightCore.nav.targetHeading, holdStr), 8, headerH + 15, 80, 230, 255, "Arial", 8, "bold")
            end

            local gridY = headerH + 4 + statNavH + 4
            local btnAreaH = sh - gridY - 4
            local rowH = math.floor((btnAreaH - 8) / 3)

            -- Row 1: 航向微調/步進調整 (Heading Step Adjustments)
            local bW4 = math.floor((sw - 12 - 12) / 4)
            addBtn(6, gridY, bW4, rowH, isFull and "[ HDG -45* ]" or "-45*", {20, 70, 100}, {255, 255, 255}, function() FlightCore.adjustTargetHeading(-45) end)
            addBtn(6 + (bW4+4), gridY, bW4, rowH, isFull and "[ HDG -5* ]" or "-5*", {30, 90, 120}, {255, 255, 255}, function() FlightCore.adjustTargetHeading(-5) end)
            addBtn(6 + (bW4+4)*2, gridY, bW4, rowH, isFull and "[ HDG +5* ]" or "+5*", {30, 90, 120}, {255, 255, 255}, function() FlightCore.adjustTargetHeading(5) end)
            addBtn(6 + (bW4+4)*3, gridY, bW4, rowH, isFull and "[ HDG +45* ]" or "+45*", {20, 70, 100}, {255, 255, 255}, function() FlightCore.adjustTargetHeading(45) end)

            -- Row 2: 四大主方位快速鎖定 (Cardinal Direction Presets)
            local r2Y = gridY + rowH + 4
            addBtn(6, r2Y, bW4, rowH, isFull and "[ 000* NORTH ]" or "0* N", {27, 94, 32}, {255, 255, 255}, function() FlightCore.setTargetHeading(0) end)
            addBtn(6 + (bW4+4), r2Y, bW4, rowH, isFull and "[ 090* EAST ]" or "90* E", {46, 125, 50}, {255, 255, 255}, function() FlightCore.setTargetHeading(90) end)
            addBtn(6 + (bW4+4)*2, r2Y, bW4, rowH, isFull and "[ 180* SOUTH ]" or "180* S", {0, 131, 143}, {255, 255, 255}, function() FlightCore.setTargetHeading(180) end)
            addBtn(6 + (bW4+4)*3, r2Y, bW4, rowH, isFull and "[ 270* WEST ]" or "270* W", {0, 105, 92}, {255, 255, 255}, function() FlightCore.setTargetHeading(270) end)

            -- Row 3: 自駕儀航向保持與快速功能 (Heading Hold Autopilot & Ops)
            local r3Y = r2Y + rowH + 4
            local holdBg = FlightCore.nav.headingHold and {46, 125, 50} or {55, 71, 79}
            local holdText = isFull and (FlightCore.nav.headingHold and "[ HDG HOLD: ON ]" or "[ HDG HOLD: OFF ]") or (FlightCore.nav.headingHold and "HOLD:ON" or "HOLD:OFF")
            addBtn(6, r3Y, bW4, rowH, holdText, holdBg, {255, 255, 255}, function() FlightCore.toggleHeadingHold() end)
            addBtn(6 + (bW4+4), r3Y, bW4, rowH, isFull and "[ SYNC HDG ]" or "SYNC HDG", {21, 101, 192}, {255, 255, 255}, function() FlightCore.syncHeading() end)
            local altHoldBg = (FlightCore.state.mode == "HOLD_ALT") and {46, 125, 50} or {27, 94, 32}
            addBtn(6 + (bW4+4)*2, r3Y, bW4, rowH, isFull and "[ HOLD ALT ]" or "HOLD ALT", altHoldBg, {255, 255, 255}, function() FlightCore.holdAltitude() end)
            local stopBg = (FlightCore.state.mode == "IDLE") and {198, 40, 40} or {183, 28, 28}
            addBtn(6 + (bW4+4)*3, r3Y, bW4, rowH, isFull and "[ STOP IDLE ]" or "STOP", stopBg, {255, 255, 255}, function() FlightCore.stopEngines() end)

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
                local bg = online and {20, 35, 30} or {40, 20, 20}
                gpu.fillRect(dispId, cx, cy, cardW, cardH, bg[1], bg[2], bg[3])

                local tagCol = online and {80, 255, 120} or {255, 80, 80}
                gpu.drawText(dispId, string.format("[%s] %d ENG", s.slot, qH.total), cx + 4, cy + 4, 220, 235, 255, "Arial", 9, "bold")
                local actStr = (cardW < 75) and string.format("ON:%d P:%d", qH.online, FlightCore.engineOutputs[s.slot] or 0) or string.format("ACT:%d/%d | P:%d", qH.online, qH.total, FlightCore.engineOutputs[s.slot] or 0)
                gpu.drawText(dispId, actStr, cx + 4, cy + math.max(10, math.floor(cardH * 0.45)), tagCol[1], tagCol[2], tagCol[3], "Arial", 8, "bold")
            end

            local btmW = math.floor((sw - 16) / 2)
            addBtn(6, btmY, btmW, btmSysH, isFull and "[ RE-SCAN HARDWARE ]" or "RE-SCAN HW", {25, 100, 190}, {255, 255, 255}, function() FlightCore.scanQuadTurtles(); FlightCore.state.statusMsg="Re-scanned!" end)
            addBtn(6 + btmW + 4, btmY, btmW, btmSysH, isFull and "[ STOP ALL ENGINES ]" or "STOP ALL", {190, 40, 40}, {255, 255, 255}, function() FlightCore.stopEngines() end)
        end

        for _, btn in ipairs(scr.buttons) do
            gpu.fillRect(dispId, btn.x, btn.y, btn.w, btn.h, btn.bg[1], btn.bg[2], btn.bg[3])
            local btnFontSize = math.max(8, math.min(13, math.floor(btn.h * 0.46)))
            local tx = btn.x + math.max(1, math.floor((btn.w - #btn.text * (btnFontSize * 0.58)) / 2))
            local ty = btn.y + math.floor((btn.h - btnFontSize) / 2)
            gpu.drawText(dispId, btn.text, tx, ty, btn.fg[1], btn.fg[2], btn.fg[3], "Arial", btnFontSize, "bold")
        end

        gpu.updateDisplay(dispId)
    end,
    handleEvent = function(self, event, p1, p2, p3, p4)
        if event == "peripheral" or event == "peripheral_detach" or event == "monitor_resize" or event == "directgpu_resize" then
            self:refreshScreens()
            return
        end

        local targetScreen = nil
        local clickX, clickY = nil, nil

        if event == "directgpu_touch" then
            if type(p1) == "string" and type(p2) == "number" and type(p3) == "number" and type(p4) == "number" then
                for _, scr in ipairs(self.screens) do
                    if scr.name == p1 and scr.displayId == p2 then targetScreen = scr; break end
                end
                clickX, clickY = p3, p4
            elseif type(p1) == "number" and type(p2) == "number" and type(p3) == "number" then
                for _, scr in ipairs(self.screens) do
                    if scr.displayId == p1 then targetScreen = scr; break end
                end
                clickX, clickY = p2, p3
            elseif type(p1) == "number" and type(p2) == "number" then
                targetScreen = self.screens[1]
                clickX, clickY = p1, p2
            end

        elseif event == "monitor_touch" then
            for _, scr in ipairs(self.screens) do
                if scr.name == p1 then targetScreen = scr; break end
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
