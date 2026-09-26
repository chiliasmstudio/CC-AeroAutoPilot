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
            local currAlt = FlightCore.altiSensor and FlightCore.altiSensor.getHeight() or 0
            local currVspeed = FlightCore.altiSensor and FlightCore.altiSensor.getVerticalSpeed() or 0
            local currPitch, currRoll = FlightCore.getGimbalData()

            if isFull then
                -- 5x5 或以上：完整大儀表板 (Full Avionics Dashboard)
                local navInfo = ""
                if FlightCore.nav.x then
                    navInfo = string.format("POS:[%s] X:%+6.0f Y:%4.0f Z:%+6.0f | HDG:%03d* | SPD: %4.1f m/s", FlightCore.nav.source, FlightCore.nav.x, FlightCore.nav.y or currAlt, FlightCore.nav.z, math.floor(FlightCore.nav.yaw), FlightCore.nav.speed)
                else
                    navInfo = string.format("ALT: %6.1fm | TARGET: %6.1fm | V.S: %+5.2f m/s | HDG: %03d*", currAlt, FlightCore.state.targetAlt, currVspeed, math.floor(FlightCore.nav.yaw))
                end
                self:safeBlit(scr, 2, 3, navInfo, "0", "b")
                self:safeBlit(scr, 2, 4, string.format("ATTITUDE: PITCH %+4.1f* | ROLL %+4.1f* | BASE THROTTLE: %4.2f / 15.0", currPitch, currRoll, FlightCore.state.baseThrottle), "0", "8")

                local gridY = 6
                local quadW = math.floor((w - 3) / 2)
                local quadH = math.max(4, math.floor((h - gridY - 12) / 2))

                local quadDefs = {
                    {slot="FL", col=1, row=1, title="[FL] FRONT-LEFT QUADRANT"},
                    {slot="FR", col=2, row=1, title="[FR] FRONT-RIGHT QUADRANT"},
                    {slot="BL", col=1, row=2, title="[BL] BACK-LEFT QUADRANT"},
                    {slot="BR", col=2, row=2, title="[BR] BACK-RIGHT QUADRANT"}
                }

                for _, q in ipairs(quadDefs) do
                    local qx = 2 + (q.col - 1) * (quadW + 1)
                    local qy = gridY + (q.row - 1) * (quadH + 1)
                    local qH = FlightCore.getQuadHealth(q.slot)
                    local val = FlightCore.virtualOutputs[q.slot]
                    local online = qH.online > 0

                    for r = 0, quadH - 1 do
                        self:safeBlit(scr, qx, qy + r, string.rep(" ", quadW), "0", "7")
                    end
                    self:safeBlit(scr, qx + 1, qy, string.format("%s (ENGINES: %d | ACT: %d)", q.title, qH.total, qH.online), online and "9" or "e", "8")

                    local barLen = math.max(6, quadW - 24)
                    local ratio = math.min(1.0, math.max(0.0, val / 15.0))
                    local filled = math.floor(ratio * barLen + 0.5)
                    local barStr = "[" .. string.rep("=", filled) .. string.rep(" ", barLen - filled) .. "]"
                    self:safeBlit(scr, qx + 1, qy + 1, string.format("THR: %4.1f/15.0 %s", val, barStr), "5", "7")
                    self:safeBlit(scr, qx + 1, qy + 2, string.format("PWM OUTPUT: %2d / 15 | STATUS: %s", FlightCore.engineOutputs[q.slot] or 0, online and "HEALTHY" or "OFFLINE"), online and "3" or "e", "7")
                end

                local statY = gridY + quadH * 2 + 2
                self:safeBlit(scr, 2, statY, string.format("STATUS: %s | BASE: %4.2f/15", FlightCore.state.statusMsg, FlightCore.state.baseThrottle), "0", "b")

                -- 3 排按鈕
                local bY1 = statY + 2
                local bW1 = math.floor((w - 7) / 6)
                addBtn(2, bY1, bW1, 2, "+50m", "0", "5", function() FlightCore.adjustTargetAlt(50) end)
                addBtn(3 + bW1, bY1, bW1, 2, "+10m", "0", "5", function() FlightCore.adjustTargetAlt(10) end)
                addBtn(4 + bW1*2, bY1, bW1, 2, "+1m", "0", "5", function() FlightCore.adjustTargetAlt(1) end)
                addBtn(5 + bW1*3, bY1, bW1, 2, "-1m", "0", "e", function() FlightCore.adjustTargetAlt(-1) end)
                addBtn(6 + bW1*4, bY1, bW1, 2, "-10m", "0", "e", function() FlightCore.adjustTargetAlt(-10) end)
                addBtn(7 + bW1*5, bY1, bW1, 2, "LOCK", "0", "3", function() FlightCore.lockCurrentAlt() end)

                local bY2 = bY1 + 3
                local bW2 = math.floor((w - 5) / 4)
                addBtn(2, bY2, bW2, 2, "BASE +1.0", "3", "0", function() FlightCore.adjustBaseThrottle(1.0) end)
                addBtn(3 + bW2, bY2, bW2, 2, "BASE -1.0", "9", "0", function() FlightCore.adjustBaseThrottle(-1.0) end)
                addBtn(4 + bW2*2, bY2, bW2, 2, "RE-SCAN HW", "b", "0", function() FlightCore.scanQuadTurtles(); FlightCore.state.statusMsg="Re-scanned!" end)
                addBtn(5 + bW2*3, bY2, bW2, 2, "CALIBRATE", "a", "0", function() FlightCore.startCalibration() end)

                local bY3 = bY2 + 3
                local bW3 = math.floor((w - 3) / 2)
                local holdBg = (FlightCore.state.mode == "HOLD_ALT") and "5" or "3"
                addBtn(2, bY3, bW3, 2, "[ HOLD ALTITUDE ]", "0", holdBg, function() FlightCore.holdAltitude() end)
                addBtn(3 + bW3, bY3, bW3, 2, "[ STOP / IDLE ]", "0", stopBg, function() FlightCore.stopEngines() end)
            else
                -- 3x3 ~ 4x5 精簡版 (Compact Edition)
                local navShort = ""
                if FlightCore.nav.x then
                    navShort = string.format("X:%.0f Z:%.0f H:%03d* | A:%.0f", FlightCore.nav.x, FlightCore.nav.z, math.floor(FlightCore.nav.yaw), currAlt)
                else
                    navShort = string.format("ALT:%5.1fm | H:%03d* | V:%+4.1f", currAlt, math.floor(FlightCore.nav.yaw), currVspeed)
                end
                self:safeBlit(scr, 2, 2, string.format("%s | [%s]", navShort, FlightCore.state.mode:sub(1,4)), "0", "b")

                local gridY = 4
                local quadW = math.floor((w - 3) / 2)
                local quadH = math.max(2, math.floor((h - gridY - 5) / 2))

                local quadDefs = {
                    {slot="FL", col=1, row=1, title="FL"},
                    {slot="FR", col=2, row=1, title="FR"},
                    {slot="BL", col=1, row=2, title="BL"},
                    {slot="BR", col=2, row=2, title="BR"}
                }

                for _, q in ipairs(quadDefs) do
                    local qx = 2 + (q.col - 1) * (quadW + 1)
                    local qy = gridY + (q.row - 1) * (quadH + 1)
                    local qH = FlightCore.getQuadHealth(q.slot)
                    local val = FlightCore.virtualOutputs[q.slot]
                    local online = qH.online > 0

                    for r = 0, quadH - 1 do
                        self:safeBlit(scr, qx, qy + r, string.rep(" ", quadW), "0", "7")
                    end
                    self:safeBlit(scr, qx + 1, qy, string.format("[%s] %dE (%d ON)", q.slot, qH.total, qH.online), online and "9" or "e", "8")
                    self:safeBlit(scr, qx + 1, qy + 1, string.format("THR: %4.1f P:%d", val, FlightCore.engineOutputs[q.slot] or 0), "5", "7")
                end

                local statY = h - 4
                self:safeBlit(scr, 2, statY, string.format("BASE: %4.2f | %s", FlightCore.state.baseThrottle, FlightCore.state.statusMsg:sub(1, math.max(10, w - 18))), "0", "b")

                local bY1 = h - 3
                local bW4 = math.floor((w - 5) / 4)
                addBtn(2, bY1, bW4, 1, "+10m", "0", "5", function() FlightCore.adjustTargetAlt(10) end)
                addBtn(3 + bW4, bY1, bW4, 1, "-10m", "0", "e", function() FlightCore.adjustTargetAlt(-10) end)
                addBtn(4 + bW4*2, bY1, bW4, 1, "B+1", "3", "0", function() FlightCore.adjustBaseThrottle(1.0) end)
                addBtn(5 + bW4*3, bY1, bW4, 1, "B-1", "9", "0", function() FlightCore.adjustBaseThrottle(-1.0) end)

                local bY2 = h - 1
                local bW2 = math.floor((w - 3) / 2)
                local holdBg = (FlightCore.state.mode == "HOLD_ALT") and "5" or "3"
                addBtn(2, bY2, bW2, 1, "HOLD", "0", holdBg, function() FlightCore.holdAltitude() end)
                local stopBg = (FlightCore.state.mode == "IDLE") and "e" or "6"
                addBtn(3 + bW2, bY2, bW2, 1, "STOP", "0", stopBg, function() FlightCore.stopEngines() end)
            end

        elseif scr.currentView == "PFD" then
            local currAlt = FlightCore.altiSensor and FlightCore.altiSensor.getHeight() or 0
            local currVspeed = FlightCore.altiSensor and FlightCore.altiSensor.getVerticalSpeed() or 0
            local currPitch, currRoll = FlightCore.getGimbalData()

            self:safeBlit(scr, 2, 3, string.format("ALT: %6.1fm | TARGET: %6.1fm", currAlt, FlightCore.state.targetAlt), "0", "b")
            self:safeBlit(scr, 2, 4, string.format("VERTICAL SPEED: %+5.2f m/s", currVspeed), "0", "8")
            self:safeBlit(scr, 2, 5, string.format("PITCH: %+4.1f* | ROLL: %+4.1f*", currPitch, currRoll), "0", "8")
            self:safeBlit(scr, 2, 6, string.format("MODE: [%s]", FlightCore.state.mode), (FlightCore.state.mode == "HOLD_ALT") and "5" or "e", "8")

            local btmY = h - (isFull and 3 or 2)
            local btnW = math.floor((w - 5) / 4)
            local btnH = isFull and 3 or 2
            addBtn(2, btmY, btnW, btnH, "+10m", "0", "5", function() FlightCore.adjustTargetAlt(10) end)
            addBtn(3 + btnW, btmY, btnW, btnH, "-10m", "0", "e", function() FlightCore.adjustTargetAlt(-10) end)
            addBtn(4 + btnW*2, btmY, btnW, btnH, "HOLD", "0", "3", function() FlightCore.holdAltitude() end)
            addBtn(5 + btnW*3, btmY, btnW, btnH, "STOP", "0", "e", function() FlightCore.stopEngines() end)

        elseif scr.currentView == "ECAM" then
            local gridY = 3
            local quadW = math.floor((w - 3) / 2)
            local quadH = math.max(3, math.floor((h - gridY - 2) / 2))

            local quadDefs = {
                {slot="FL", col=1, row=1, title="[FL] FRONT-LEFT"},
                {slot="FR", col=2, row=1, title="[FR] FRONT-RIGHT"},
                {slot="BL", col=1, row=2, title="[BL] BACK-LEFT"},
                {slot="BR", col=2, row=2, title="[BR] BACK-RIGHT"}
            }

            for _, q in ipairs(quadDefs) do
                local qx = 2 + (q.col - 1) * (quadW + 1)
                local qy = gridY + (q.row - 1) * (quadH + 1)
                local qH = FlightCore.getQuadHealth(q.slot)
                local val = FlightCore.virtualOutputs[q.slot]
                local sig = FlightCore.engineOutputs[q.slot]
                local online = qH.online > 0

                for r = 0, quadH - 1 do
                    self:safeBlit(scr, qx, qy + r, string.rep(" ", quadW), "0", "7")
                end
                self:safeBlit(scr, qx + 1, qy, string.format("%s (%dE)", q.title, qH.total), online and "9" or "e", "8")
                self:safeBlit(scr, qx + 1, qy + 1, string.format("THR: %4.1f/15.0", val), "5", "7")
                self:safeBlit(scr, qx + 1, qy + 2, string.format("PWM: %2d/15", sig), "3", "7")
                if quadH >= 4 then
                    self:safeBlit(scr, qx + 1, qy + 3, string.format("ACTIVE: %d/%d", qH.online, qH.total), online and "5" or "e", "7")
                end
            end

            self:safeBlit(scr, 2, h, string.format("BASE: %4.2f/15 | %s", FlightCore.state.baseThrottle, FlightCore.state.statusMsg:sub(1, w - 20)), "0", "b")

        elseif scr.currentView == "CTRL" then
            local currAlt = FlightCore.altiSensor and FlightCore.altiSensor.getHeight() or 0
            local currVspeed = FlightCore.altiSensor and FlightCore.altiSensor.getVerticalSpeed() or 0
            local currPitch, currRoll = FlightCore.getGimbalData()

            if isFull then
                -- CC Monitor 大螢幕版 (>= 5x5)
                local navInfo = ""
                if FlightCore.nav.x then
                    navInfo = string.format("ALT: %6.1fm | TGT: %4.0fm | POS:[%s] X:%.0f Z:%.0f | HDG: %03d*", currAlt, FlightCore.state.targetAlt, FlightCore.nav.source, FlightCore.nav.x, FlightCore.nav.z, math.floor(FlightCore.nav.yaw))
                else
                    navInfo = string.format("ALT: %6.1fm | TGT: %4.0fm | V.S: %+5.2f | HDG: %03d*", currAlt, FlightCore.state.targetAlt, currVspeed, math.floor(FlightCore.nav.yaw))
                end
                self:safeBlit(scr, 2, 3, navInfo, "0", "b")
                self:safeBlit(scr, 2, 4, string.format("MODE: [%s] | BASE: %4.2f | GYRO: %+2.0f*, %+2.0f*", FlightCore.state.mode, FlightCore.state.baseThrottle, currPitch, currRoll), "0", "8")

                local btnAreaH = h - 6
                local rowH = math.max(2, math.floor((btnAreaH - 4) / 3))

                -- Row 1: Target Alt
                local bY1 = 6
                local bW1 = math.floor((w - 7) / 6)
                addBtn(2, bY1, bW1, rowH, "+50m", "0", "5", function() FlightCore.adjustTargetAlt(50) end)
                addBtn(3 + bW1, bY1, bW1, rowH, "+10m", "0", "5", function() FlightCore.adjustTargetAlt(10) end)
                addBtn(4 + bW1*2, bY1, bW1, rowH, "+1m", "0", "5", function() FlightCore.adjustTargetAlt(1) end)
                addBtn(5 + bW1*3, bY1, bW1, rowH, "-1m", "0", "e", function() FlightCore.adjustTargetAlt(-1) end)
                addBtn(6 + bW1*4, bY1, bW1, rowH, "-10m", "0", "e", function() FlightCore.adjustTargetAlt(-10) end)
                addBtn(7 + bW1*5, bY1, bW1, rowH, "LOCK", "0", "3", function() FlightCore.lockCurrentAlt() end)

                -- Row 2: Throttle
                local bY2 = bY1 + rowH + 1
                local bW2 = math.floor((w - 5) / 4)
                addBtn(2, bY2, bW2, rowH, "BASE +1.0", "3", "0", function() FlightCore.adjustBaseThrottle(1.0) end)
                addBtn(3 + bW2, bY2, bW2, rowH, "BASE -1.0", "9", "0", function() FlightCore.adjustBaseThrottle(-1.0) end)
                addBtn(4 + bW2*2, bY2, bW2, rowH, "BASE +0.1", "b", "0", function() FlightCore.adjustBaseThrottle(0.1) end)
                addBtn(5 + bW2*3, bY2, bW2, rowH, "BASE -0.1", "7", "0", function() FlightCore.adjustBaseThrottle(-0.1) end)

                -- Row 3: Flight Ops
                local bY3 = bY2 + rowH + 1
                local bW3 = math.floor((w - 5) / 4)
                addBtn(2, bY3, bW3, rowH, "HOLD", "5", "0", function() FlightCore.holdAltitude() end)
                addBtn(3 + bW3, bY3, bW3, rowH, "CALIB", "a", "0", function() FlightCore.startCalibration() end)
                addBtn(4 + bW3*2, bY3, bW3, rowH, "RE-SCAN", "b", "0", function() FlightCore.scanQuadTurtles(); FlightCore.state.statusMsg="Re-scanned!" end)
                addBtn(5 + bW3*3, bY3, bW3, rowH, "STOP", "e", "0", function() FlightCore.stopEngines() end)
            else
                -- 3x3 ~ 4x5 精簡版
                local navCompact = FlightCore.nav.x and string.format("[%s] X:%.0f Z:%.0f H:%03d*", FlightCore.state.mode:sub(1,4), FlightCore.nav.x, FlightCore.nav.z, math.floor(FlightCore.nav.yaw)) or string.format("[%s] A:%.0f->%.0f H:%03d*", FlightCore.state.mode:sub(1,4), currAlt, FlightCore.state.targetAlt, math.floor(FlightCore.nav.yaw))
                self:safeBlit(scr, 2, 3, navCompact, "0", "b")

                local rowH = math.max(1, math.floor((h - 7) / 3))

                -- Row 1: Target Alt
                local bY1 = 5
                local bW1 = math.floor((w - 5) / 4)
                addBtn(2, bY1, bW1, rowH, "+10m", "0", "5", function() FlightCore.adjustTargetAlt(10) end)
                addBtn(3 + bW1, bY1, bW1, rowH, "+1m", "0", "5", function() FlightCore.adjustTargetAlt(1) end)
                addBtn(4 + bW1*2, bY1, bW1, rowH, "-1m", "0", "e", function() FlightCore.adjustTargetAlt(-1) end)
                addBtn(5 + bW1*3, bY1, bW1, rowH, "-10m", "0", "e", function() FlightCore.adjustTargetAlt(-10) end)

                -- Row 2: Throttle
                local bY2 = bY1 + rowH + 1
                local bW2 = math.floor((w - 5) / 4)
                addBtn(2, bY2, bW2, rowH, "B +1", "3", "0", function() FlightCore.adjustBaseThrottle(1.0) end)
                addBtn(3 + bW2, bY2, bW2, rowH, "B -1", "9", "0", function() FlightCore.adjustBaseThrottle(-1.0) end)
                addBtn(4 + bW2*2, bY2, bW2, rowH, "B +.1", "b", "0", function() FlightCore.adjustBaseThrottle(0.1) end)
                addBtn(5 + bW2*3, bY2, bW2, rowH, "B -.1", "7", "0", function() FlightCore.adjustBaseThrottle(-0.1) end)

                -- Row 3: Flight Ops
                local bY3 = bY2 + rowH + 1
                local bW3 = math.floor((w - 5) / 4)
                addBtn(2, bY3, bW3, rowH, "HOLD", "5", "0", function() FlightCore.holdAltitude() end)
                addBtn(3 + bW3, bY3, bW3, rowH, "CALIB", "a", "0", function() FlightCore.startCalibration() end)
                addBtn(4 + bW3*2, bY3, bW3, rowH, "RE-SCAN", "b", "0", function() FlightCore.scanQuadTurtles(); FlightCore.state.statusMsg="Re-scanned!" end)
                addBtn(5 + bW3*3, bY3, bW3, rowH, "STOP", "e", "0", function() FlightCore.stopEngines() end)
            end

        elseif scr.currentView == "NAV" then
            local currAlt = FlightCore.altiSensor and FlightCore.altiSensor.getHeight() or 0
            local currPitch, currRoll = FlightCore.getGimbalData()
            local holdStr = FlightCore.nav.headingHold and "ON" or "OFF"

            if isFull then
                local posStr = ""
                if FlightCore.nav.x then
                    posStr = string.format("POS:[%s] X:%+6.0f Y:%4.0f Z:%+6.0f | SPD: %4.1f m/s", FlightCore.nav.source, FlightCore.nav.x, FlightCore.nav.y or currAlt, FlightCore.nav.z, FlightCore.nav.speed)
                else
                    posStr = string.format("POS:[%s] ALT: %4.0fm | GYRO: P:%+2.0f* R:%+2.0f*", FlightCore.nav.source, currAlt, currPitch, currRoll)
                end
                self:safeBlit(scr, 2, 3, posStr, "0", "b")
                self:safeBlit(scr, 2, 4, string.format("CURRENT HDG: %03d* | TARGET HDG: %03d* | HEADING HOLD: [%s]", math.floor(FlightCore.nav.yaw), FlightCore.nav.targetHeading, holdStr), "0", "8")
            else
                local posStr = ""
                if FlightCore.nav.x and w >= 32 then
                    posStr = string.format("[%s] X:%.0f Z:%.0f S:%.1f", FlightCore.nav.source, FlightCore.nav.x, FlightCore.nav.z, FlightCore.nav.speed)
                else
                    posStr = string.format("ALT: %.0fm", currAlt)
                end
                self:safeBlit(scr, 2, 2, posStr, "0", "b")
                self:safeBlit(scr, 2, 3, string.format("H:%03d* -> T:%03d* [%s]", math.floor(FlightCore.nav.yaw), FlightCore.nav.targetHeading, holdStr), "0", "8")
            end

            local gridY = isFull and 6 or 4
            local btnAreaH = h - gridY - 1
            local rowH = math.max(1, math.floor((btnAreaH - 2) / 3))
            local colW = math.floor((w - 5) / 4)

            -- Row 1: 航向微調/步進調整 (Heading Step Adjustments)
            local bY1 = gridY
            addBtn(2, bY1, colW, rowH, isFull and "[ HDG -45* ]" or "-45*", "0", "3", function() FlightCore.adjustTargetHeading(-45) end)
            addBtn(3 + colW, bY1, colW, rowH, isFull and "[ HDG -5* ]" or "-5*", "0", "b", function() FlightCore.adjustTargetHeading(-5) end)
            addBtn(4 + colW*2, bY1, colW, rowH, isFull and "[ HDG +5* ]" or "+5*", "0", "b", function() FlightCore.adjustTargetHeading(5) end)
            addBtn(5 + colW*3, bY1, colW, rowH, isFull and "[ HDG +45* ]" or "+45*", "0", "3", function() FlightCore.adjustTargetHeading(45) end)

            -- Row 2: 四大主方位快速鎖定 (Cardinal Direction Presets)
            local bY2 = bY1 + rowH + 1
            addBtn(2, bY2, colW, rowH, isFull and "[ 000* NORTH ]" or "0* N", "0", "5", function() FlightCore.setTargetHeading(0) end)
            addBtn(3 + colW, bY2, colW, rowH, isFull and "[ 090* EAST ]" or "90* E", "0", "5", function() FlightCore.setTargetHeading(90) end)
            addBtn(4 + colW*2, bY2, colW, rowH, isFull and "[ 180* SOUTH ]" or "180* S", "0", "3", function() FlightCore.setTargetHeading(180) end)
            addBtn(5 + colW*3, bY2, colW, rowH, isFull and "[ 270* WEST ]" or "270* W", "0", "3", function() FlightCore.setTargetHeading(270) end)

            -- Row 3: 自駕儀航向保持與快速功能 (Heading Hold Autopilot & Ops)
            local bY3 = bY2 + rowH + 1
            local holdBg = FlightCore.nav.headingHold and "5" or "8"
            local holdText = isFull and (FlightCore.nav.headingHold and "[ HDG HOLD: ON ]" or "[ HDG HOLD: OFF ]") or (FlightCore.nav.headingHold and "HOLD:ON" or "HOLD:OFF")
            addBtn(2, bY3, colW, rowH, holdText, "0", holdBg, function() FlightCore.toggleHeadingHold() end)
            addBtn(3 + colW, bY3, colW, rowH, isFull and "[ SYNC HDG ]" or "SYNC HDG", "0", "9", function() FlightCore.syncHeading() end)
            local altHoldBg = (FlightCore.state.mode == "HOLD_ALT") and "5" or "3"
            addBtn(4 + colW*2, bY3, colW, rowH, isFull and "[ HOLD ALT ]" or "HOLD ALT", "0", altHoldBg, function() FlightCore.holdAltitude() end)
            local stopBg = (FlightCore.state.mode == "IDLE") and "e" or "6"
            addBtn(5 + colW*3, bY3, colW, rowH, isFull and "[ STOP IDLE ]" or "STOP", "0", stopBg, function() FlightCore.stopEngines() end)

        elseif scr.currentView == "SYS" then
            local slots = {
                {slot="FL", col=1, row=1, name="FL Quad"},
                {slot="FR", col=2, row=1, name="FR Quad"},
                {slot="BL", col=1, row=2, name="BL Quad"},
                {slot="BR", col=2, row=2, name="BR Quad"}
            }
            local cardW = math.floor((w - 3) / 2)
            local cardH = math.max(3, math.floor((h - 8) / 2))
            local startY = 4

            for _, s in ipairs(slots) do
                local cx = 2 + (s.col - 1) * (cardW + 1)
                local cy = startY + (s.row - 1) * (cardH + 1)
                local qH = FlightCore.getQuadHealth(s.slot)
                local bg = (qH.online > 0) and "5" or "e"

                for r = 0, cardH - 1 do
                    self:safeBlit(scr, cx, cy + r, string.rep(" ", cardW), "0", bg)
                end
                self:safeBlit(scr, cx + 1, cy, string.format("[%s] %d ENG", s.slot, qH.total), "0", bg)
                self:safeBlit(scr, cx + 1, cy + 1, string.format("ACT: %d/%d", qH.online, qH.total), "0", bg)
            end

            local btmY = h - 2
            local btmW = math.floor((w - 3) / 2)
            addBtn(2, btmY, btmW, 2, isFull and "[ RE-SCAN HARDWARE ]" or "RE-SCAN HW", "3", "0", function() FlightCore.scanQuadTurtles(); FlightCore.state.statusMsg="Re-scanned!" end)
            addBtn(3 + btmW, btmY, btmW, 2, isFull and "[ STOP ALL ENGINES ]" or "STOP ALL", "e", "0", function() FlightCore.stopEngines() end)
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
