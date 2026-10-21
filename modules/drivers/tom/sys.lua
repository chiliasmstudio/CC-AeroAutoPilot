-- tom SYS view renderer.
-- Shared drawing helpers and layout values are supplied by the parent driver.
return function(self, scr, ctx)
    local FlightCore = ctx.FlightCore
    local addBtn = ctx.addBtn
    local btnAreaY = ctx.btnAreaY
    local btnRowH = ctx.btnRowH
    local btnSpacing = ctx.btnSpacing
    local headerH = ctx.headerH
    local instH = ctx.instH
    local instY = ctx.instY
    local isFull = ctx.isFull
    local sArc = ctx.sArc
    local sFR = ctx.sFR
    local sL = ctx.sL
    local sLS = ctx.sLS
    local sR = ctx.sR
    local sTxt = ctx.sTxt
    local sh = ctx.sh
    local statH = ctx.statH
    local statY = ctx.statY
    local sw = ctx.sw
    local topMargin = ctx.topMargin
            local hw = (FlightCore.getHardwareInventory and FlightCore.getHardwareInventory()) or FlightCore.hw or {
                displays = { hasDirectGpu=false, tomsCount=0, monitorsCount=0, totalScreens=0 },
                engines = { lift={online=0, total=0}, cruise={online=0, total=0}, steer={online=0, total=0}, totalOnline=0, totalCount=0 },
                avionics = { onBody=false, aic=false, ins=false, navTable=false, alti=false, gimbal=false, modems=0, navSource="NONE" }
            }
            local d = hw.displays or {}
            local e = hw.engines or {}
            local a = hw.avionics or {}

            local btmSysH = btnRowH
            local btmY = sh - btmSysH - 4
            local availH = btmY - topMargin - 4
            local cardH = math.max(18, math.floor((availH - 8) / 3))
            local cardW = sw - 12
            local startX = 6

            -- 1. DISPLAYS & GPU
            local c1Y = topMargin
            sFR(startX, c1Y, cardW, cardH, 0x141E2B)
            sR(startX, c1Y, cardW, cardH, 0x283C50)
            sTxt(startX + 4, c1Y + 3, isFull and "[1] DISPLAYS & GPU HARDWARE" or "[1] DISPLAYS & GPU", 0x50E6FF, 1)
            local dgpuStr = d.hasDirectGpu and "YES (DirectGPU HW Fast)" or "NO (DirectGPU Not Found)"
            local dgpuCol = d.hasDirectGpu and 0x50FF78 or 0xB4B4B4
            local rowSp1 = math.max(10, math.floor((cardH - 12) / 2))
            sTxt(startX + 6, c1Y + 3 + rowSp1, "DirectGPU  : " .. dgpuStr, dgpuCol, 1)
            local scrStr = string.format("Screens    : Tom's GPU:%d | Native Mon:%d (Total: %d)", d.tomsCount or 0, d.monitorsCount or 0, d.totalScreens or 0)
            if cardH >= 26 then
                sTxt(startX + 6, c1Y + 3 + rowSp1 * 2, scrStr, 0xDCEDFF, 1)
            end

            -- 2. PROPULSION NODES
            local c2Y = c1Y + cardH + 4
            sFR(startX, c2Y, cardW, cardH, 0x141E2B)
            sR(startX, c2Y, cardW, cardH, 0x283C50)
            sTxt(startX + 4, c2Y + 3, isFull and "[2] PROPULSION ENGINE NODES" or "[2] ENGINES", 0x50E6FF, 1)
            local el = e.lift or {}
            local ec = e.cruise or {}
            local es = e.steer or {}
            local rowSp2 = math.max(9, math.floor((cardH - 12) / 3))
            local liftStr = string.format("Lift (4-Quad): FL:%d FR:%d BL:%d BR:%d (Act: %d/%d)", el.FL or 0, el.FR or 0, el.BL or 0, el.BR or 0, el.online or 0, el.total or 0)
            local liftCol = (el.online or 0) > 0 and 0x50FF78 or 0xFF5050
            sTxt(startX + 6, c2Y + 3 + rowSp2, liftStr, liftCol, 1)
            local cruiseStr = string.format("Cruise Thrust: FWD:%d | BWD:%d (Act: %d/%d)", ec.FWD or 0, ec.BWD or 0, ec.online or 0, ec.total or 0)
            local cruiseCol = (ec.online or 0) > 0 and 0x50FF78 or 0xB4C8DC
            if cardH >= 24 then
                sTxt(startX + 6, c2Y + 3 + rowSp2 * 2, cruiseStr, cruiseCol, 1)
            end
            local steerStr = string.format("Lateral/Steer: LEFT:%d | RIGHT:%d (Act: %d/%d)", es.LEFT or 0, es.RIGHT or 0, es.online or 0, es.total or 0)
            local steerCol = (es.online or 0) > 0 and 0x50FF78 or 0xB4C8DC
            if cardH >= 34 then
                sTxt(startX + 6, c2Y + 3 + rowSp2 * 3, steerStr, steerCol, 1)
            end

            -- 3. AVIONICS & SENSORS
            local c3Y = c2Y + cardH + 4
            sFR(startX, c3Y, cardW, cardH, 0x141E2B)
            sR(startX, c3Y, cardW, cardH, 0x283C50)
            sTxt(startX + 4, c3Y + 3, isFull and "[3] AVIONICS & SENSORS" or "[3] SENSORS", 0x50E6FF, 1)
            local aicStatus = a.onBody and "[DETECTED (ON BODY)]" or (a.aic and "[DETECTED]" or "[STANDBY/SEARCHING]")
            local aicCol = (a.onBody or a.aic) and 0x50FF78 or 0xFFA03C
            local rowSp3 = math.max(9, math.floor((cardH - 12) / 3))
            sTxt(startX + 6, c3Y + 3 + rowSp3, "CCPE AIC/FMC : " .. aicStatus, aicCol, 1)
            local sensStr = string.format("Sensors & Net: Alti:%s | Gyro:%s | NavTab:%s | Modems:%d", a.alti and "ON" or "OFF", a.gimbal and "ON" or "OFF", a.navTable and "ON" or "OFF", a.modems or 0)
            if cardH >= 24 then
                sTxt(startX + 6, c3Y + 3 + rowSp3 * 2, sensStr, 0xDCEDFF, 1)
            end
            local navPosStr = ""
            if FlightCore.nav.x and FlightCore.nav.z then
                navPosStr = string.format("Position Fix : X:%.0f Y:%.0f Z:%.0f (Source: %s)", FlightCore.nav.x, FlightCore.nav.y or 0, FlightCore.nav.z, a.navSource or "INS")
            else
                navPosStr = "Position Fix : [ SEARCHING AIC / GPS / NAV_TABLE ]"
            end
            if cardH >= 34 then
                local posCol = FlightCore.nav.x and 0x50FF78 or 0xFFCD4B
                sTxt(startX + 6, c3Y + 3 + rowSp3 * 3, navPosStr, posCol, 1)
            end

            local btmW = math.floor((sw - 16) / 2)
            addBtn(6, btmY, btmW, btmSysH, isFull and "[ RE-SCAN HARDWARE ]" or "RE-SCAN HW", 0x1964BE, 0xFFFFFF, function() FlightCore.scanQuadTurtles(); FlightCore.state.statusMsg="Re-scanned!" end)
            addBtn(6 + btmW + 4, btmY, btmW, btmSysH, isFull and "[ STOP ALL ENGINES ]" or "STOP ALL", 0xBE2828, 0xFFFFFF, function() FlightCore.stopEngines() end)
end
