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
local FONT_5X7 = package.loaded["modules.bitmap_font"] or package.loaded["modules.font_5x7"] or safeRequire("modules.bitmap_font") or safeRequire("modules.font_5x7") or _G.FONT_5X7

local Views = {
    OVERVIEW = safeRequire("modules.drivers.tom.overview"),
    PFD = safeRequire("modules.drivers.tom.pfd"),
    ECAM = safeRequire("modules.drivers.tom.ecam"),
    CTRL = safeRequire("modules.drivers.tom.ctrl"),
    NAV = safeRequire("modules.drivers.tom.nav"),
    SYS = safeRequire("modules.drivers.tom.sys"),
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
            local pType = peripheral.getType(name)
            if pType == "tm_gpu" or pType == "toms_gpu" or pType == "gpu" or pType == "tm_monitor" then
                local gpu = peripheral.wrap(name)
                if gpu and gpu.fill and gpu.filledRectangle then
                    -- 必須先 setSize + refreshSize 才能 getSize 得到正確像素解析度
                    pcall(function() if gpu.setSize then gpu.setSize(64) end end)
                    pcall(function() if gpu.refreshSize then gpu.refreshSize() end end)
                    local w, h = 192, 192
                    local ok, gw, gh = pcall(function() return gpu.getSize() end)
                    if ok and gw and gh and gw > 0 and gh > 0 then
                        if gw < 32 then gw = gw * 64 end
                        if gh < 32 then gh = gh * 64 end
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
        -- Tom's GPU getSize() 返回像素大小；若回傳值為圖塊單位（< 32）則以 64px (setSize 64) 換算
        if ok and w and h and w > 0 and h > 0 then
            if w < 32 then w = w * 64 end
            if h < 32 then h = h * 64 end
            scr.screenW, scr.screenH = w, h
        end
        local sw, sh = scr.screenW or 192, scr.screenH or 192
        local isFull = (sw >= 300 and sh >= 300) -- 5x5 (320x320) 或以上為完整顯示，3x3/4x4/5x4/4x5 (<=256) 為精簡版

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

        local viewContext = {
            FlightCore = FlightCore,
            addBtn = addBtn,
            btnAreaY = btnAreaY,
            btnRowH = btnRowH,
            btnSpacing = btnSpacing,
            headerH = headerH,
            instH = instH,
            instY = instY,
            isFull = isFull,
            sArc = sArc,
            sFR = sFR,
            sL = sL,
            sLS = sLS,
            sR = sR,
            sTxt = sTxt,
            sh = sh,
            statH = statH,
            statY = statY,
            sw = sw,
            topMargin = topMargin,
        }

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
            if not targetScreen and #self.screens > 0 then targetScreen = self.screens[1] end
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
                clickX, clickY = p2, p3
            end

        elseif event == "mouse_click" then
            targetScreen = self.screens[1]
            if targetScreen and type(p2) == "number" and type(p3) == "number" then
                local tw, th = term.getSize()
                clickX = math.floor(((p2 - 0.5) / tw) * (targetScreen.screenW or 192))
                clickY = math.floor(((p3 - 0.5) / th) * (targetScreen.screenH or 192))
            end
        end

        if targetScreen and clickX and clickY then
            -- 1. 嘗試直接像素座標匹配
            local hit = false
            for _, btn in ipairs(targetScreen.buttons) do
                if clickX >= btn.x and clickX <= btn.x + btn.w and clickY >= btn.y and clickY <= btn.y + btn.h then
                    pcall(btn.action)
                    self:drawScreen(targetScreen)
                    hit = true
                    break
                end
            end

            -- 2. 若直接像素未命中且座標疑似為字元格（< 60），嘗試字元格縮放換算匹配
            if not hit and clickX < 60 and (targetScreen.screenW or 192) >= 80 then
                local mw, mh = 29, 19
                local mon = peripheral.wrap(p1)
                if mon and mon.getSize then
                    local ok, w, h = pcall(function() return mon.getSize() end)
                    if ok and w and h and w > 0 and h > 0 then
                        if w < 60 then mw, mh = w, h end
                    end
                end
                local mappedX = math.floor(((clickX - 0.5) / mw) * (targetScreen.screenW or 192))
                local mappedY = math.floor(((clickY - 0.5) / mh) * (targetScreen.screenH or 192))
                for _, btn in ipairs(targetScreen.buttons) do
                    if mappedX >= btn.x and mappedX <= btn.x + btn.w and mappedY >= btn.y and mappedY <= btn.y + btn.h then
                        pcall(btn.action)
                        self:drawScreen(targetScreen)
                        break
                    end
                end
            end
        end
    end
}

return Driver
