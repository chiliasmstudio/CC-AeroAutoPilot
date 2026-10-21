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

local Views = {
    OVERVIEW = safeRequire("modules.drivers.directgpu.overview"),
    PFD = safeRequire("modules.drivers.directgpu.pfd"),
    ECAM = safeRequire("modules.drivers.directgpu.ecam"),
    CTRL = safeRequire("modules.drivers.directgpu.ctrl"),
    NAV = safeRequire("modules.drivers.directgpu.nav"),
    SYS = safeRequire("modules.drivers.directgpu.sys"),
}

local VIEW_TITLES = {
    OVERVIEW = "FLIGHT MONITOR",
    PFD      = "PRIMARY FLIGHT",
    ECAM     = "ECAM 2x2",
    CTRL     = "FLIGHT CONTROLS",
    NAV      = "HEADING & NAV",
    SYS      = "SYSTEM STATUS"
}

local Driver = {
    views = Views,
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

        local viewContext = {
            FlightCore = FlightCore,
            addBtn = addBtn,
            btnAreaY = btnAreaY,
            btnRowH = btnRowH,
            btnSpacing = btnSpacing,
            dispId = dispId,
            gpu = gpu,
            headerH = headerH,
            instH = instH,
            instY = instY,
            isFull = isFull,
            sh = sh,
            statH = statH,
            statY = statY,
            sw = sw,
            topMargin = topMargin,
        }

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
            self.views.OVERVIEW(self, scr, viewContext)
        elseif scr.currentView == "PFD" then
            self.views.PFD(self, scr, viewContext)
        elseif scr.currentView == "ECAM" then
            self.views.ECAM(self, scr, viewContext)
        elseif scr.currentView == "CTRL" then
            self.views.CTRL(self, scr, viewContext)
        elseif scr.currentView == "NAV" then
            self.views.NAV(self, scr, viewContext)
        elseif scr.currentView == "SYS" then
            self.views.SYS(self, scr, viewContext)
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
