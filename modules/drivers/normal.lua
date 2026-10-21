--[[
    modules/drivers/normal.lua
    CC: Tweaked Native Monitor & Terminal Driver
--]]

local function safeRequire(mod)
    local ok, res = pcall(require, mod)
    if ok then return res end
    return nil
end

local FlightCore = package.loaded["modules.flight_core"] or safeRequire("modules.flight_core") or _G.FlightCore

local Views = {
    OVERVIEW = safeRequire("modules.drivers.normal.overview"),
    PFD = safeRequire("modules.drivers.normal.pfd"),
    ECAM = safeRequire("modules.drivers.normal.ecam"),
    CTRL = safeRequire("modules.drivers.normal.ctrl"),
    NAV = safeRequire("modules.drivers.normal.nav"),
    SYS = safeRequire("modules.drivers.normal.sys"),
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
            if scr.id then existing[scr.id] = scr end
        end
        local newScreens = {}
        for _, name in ipairs(peripheral.getNames()) do
            if peripheral.getType(name) == "monitor" then
                local mon = peripheral.wrap(name)
                if mon then
                    pcall(function() mon.setTextScale(0.5) end)
                    local mw, mh = mon.getSize()
                    local prev = existing[name]
                    table.insert(newScreens, {
                        id = name,
                        device = mon,
                        isMonitor = true,
                        screenW = mw,
                        screenH = mh,
                        currentView = prev and prev.currentView or "OVERVIEW",
                        isMenuOpen = prev and prev.isMenuOpen or false,
                        buttons = prev and prev.buttons or {}
                    })
                end
            end
        end

        if #newScreens == 0 then
            local tw, th = term.getSize()
            local prev = existing["terminal"]
            table.insert(newScreens, {
                id = "terminal",
                device = term.current(),
                isMonitor = false,
                screenW = tw,
                screenH = th,
                currentView = prev and prev.currentView or "OVERVIEW",
                isMenuOpen = prev and prev.isMenuOpen or false,
                buttons = prev and prev.buttons or {}
            })
        end
        self.screens = newScreens
        return true
    end,
    init = function(self)
        return self:refreshScreens()
    end,
    safeBlit = function(self, scr, x, y, text, fg, bg)
        local t = scr.device
        local w, h = scr.screenW, scr.screenH
        if y < 1 or y > h or x > w then return end
        if x < 1 then
            local cut = 1 - x
            if cut >= #text then return end
            text = text:sub(cut + 1)
            fg = fg:sub(cut + 1)
            bg = bg:sub(cut + 1)
            x = 1
        end
        if x + #text - 1 > w then
            local maxLen = w - x + 1
            text = text:sub(1, maxLen)
            fg = fg:sub(1, maxLen)
            bg = bg:sub(1, maxLen)
        end
        if #text > 0 then
            t.setCursorPos(x, y)
            t.blit(text, fg, bg)
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
        local t = scr.device
        local w, h = t.getSize()
        scr.screenW, scr.screenH = w, h
        local isFull = (w >= 140 and h >= 58) -- 5x5 (approx 145x60 chars at textScale 0.5) 或以上為完整顯示，3x3/4x4/5x4/4x5 為精簡版

        t.setBackgroundColor(colors.black)
        t.clear()
        scr.buttons = {}

        local function addBtn(x, y, bw, bh, text, fg, bg, act)
            table.insert(scr.buttons, {x=x, y=y, w=bw, h=bh, text=text, fg=fg, bg=bg, action=act})
        end

        -- 1. 頂部導航列 (無版本號，僅顯示當前視圖名稱)
        local menuBtnW = isFull and 8 or 5
        local menuBtnX = w - menuBtnW + 1
        self:safeBlit(scr, 1, 1, string.rep(" ", w), "0", "b")

        local viewTitle = scr.isMenuOpen and "SELECT VIEW" or (isFull and (VIEW_TITLES[scr.currentView] or scr.currentView) or scr.currentView)
        self:safeBlit(scr, 2, 1, viewTitle, "0", "b")

        if scr.isMenuOpen then
            addBtn(menuBtnX, 1, menuBtnW, 1, isFull and "[CLOSE]" or "[X]", "0", "e", function() scr.isMenuOpen = false end)
        else
            addBtn(menuBtnX, 1, menuBtnW, 1, isFull and "[ MENU ]" or "[=]", "0", "9", function() scr.isMenuOpen = true end)
        end

        local viewContext = {
            FlightCore = FlightCore,
            addBtn = addBtn,
            h = h,
            isFull = isFull,
            w = w,
        }

        -- 2. 視圖分流
        if scr.isMenuOpen then
            local startY = 3
            local cardW = math.floor((w - 3) / 2)
            local cardH = math.max(2, math.floor((h - startY - 2) / 3))

            local function addMenuCard(col, row, title, targetView, fg, bg)
                local cx = 2 + (col - 1) * (cardW + 1)
                local cy = startY + (row - 1) * (cardH + 1)
                addBtn(cx, cy, cardW, cardH, title, fg, bg, function()
                    scr.currentView = targetView
                    scr.isMenuOpen = false
                end)
            end

            addMenuCard(1, 1, isFull and "1. OVERVIEW (ALL)" or "1. OVERVIEW", "OVERVIEW", "0", "9")
            addMenuCard(2, 1, isFull and "2. PFD (FLIGHT)" or "2. PFD", "PFD", "0", "5")
            addMenuCard(1, 2, isFull and "3. ECAM (QUAD ENG)" or "3. ECAM", "ECAM", "0", "e")
            addMenuCard(2, 2, isFull and "4. CTRL (CONTROLS)" or "4. CTRL", "CTRL", "0", "3")
            addMenuCard(1, 3, isFull and "5. NAV (HEADING)" or "5. NAV", "NAV", "0", "b")
            addMenuCard(2, 3, isFull and "6. SYS (DIAGNOSE)" or "6. SYS", "SYS", "0", "a")

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
            for dy = 0, btn.h - 1 do
                self:safeBlit(scr, btn.x, btn.y + dy, string.rep(" ", btn.w), btn.fg, btn.bg)
            end
            local textX = btn.x + math.max(0, math.floor((btn.w - #btn.text) / 2))
            local textY = btn.y + math.floor((btn.h - 1) / 2)
            self:safeBlit(scr, textX, textY, btn.text, btn.fg, btn.bg)
        end
    end,
    handleEvent = function(self, event, p1, p2, p3, p4)
        if event == "peripheral" or event == "peripheral_detach" or event == "monitor_resize" then
            self:refreshScreens()
            return
        end
        for _, scr in ipairs(self.screens) do
            local clickX, clickY = nil, nil
            if event == "monitor_touch" then
                if scr.isMonitor and peripheral.getName(scr.device) == p1 then
                    clickX, clickY = p2, p3
                end
            elseif event == "mouse_click" then
                if not scr.isMonitor then
                    clickX, clickY = p2, p3
                end
            end

            if clickX and clickY then
                for _, btn in ipairs(scr.buttons) do
                    if clickX >= btn.x and clickX <= btn.x + btn.w - 1 and clickY >= btn.y and clickY <= btn.y + btn.h - 1 then
                        pcall(btn.action)
                        self:drawScreen(scr)
                        break
                    end
                end
            end
        end
    end
}

return Driver
