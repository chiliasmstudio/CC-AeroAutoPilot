-- directgpu SYS view renderer.
-- Shared drawing helpers and layout values are supplied by the parent driver.
return function(self, scr, ctx)
    local FlightCore = ctx.FlightCore
    local addBtn = ctx.addBtn
    local btnAreaY = ctx.btnAreaY
    local btnRowH = ctx.btnRowH
    local btnSpacing = ctx.btnSpacing
    local dispId = ctx.dispId
    local gpu = ctx.gpu
    local headerH = ctx.headerH
    local instH = ctx.instH
    local instY = ctx.instY
    local isFull = ctx.isFull
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
            gpu.fillRect(dispId, startX, c1Y, cardW, cardH, 20, 30, 43)
            gpu.drawText(dispId, isFull and "[1] DISPLAYS & GPU HARDWARE" or "[1] DISPLAYS & GPU", startX + 6, c1Y + 3, 80, 230, 255, "Arial", 9, "bold")
            local dgpuStr = d.hasDirectGpu and "YES (DirectGPU HW Fast)" or "NO (DirectGPU Not Found)"
            local dgpuCol = d.hasDirectGpu and {80, 255, 120} or {180, 180, 180}
            local rowSp1 = math.max(10, math.floor((cardH - 12) / 2))
            gpu.drawText(dispId, "DirectGPU  : " .. dgpuStr, startX + 8, c1Y + 4 + rowSp1, dgpuCol[1], dgpuCol[2], dgpuCol[3], "Arial", 8, "bold")
            local scrStr = string.format("Screens    : Tom's GPU:%d | Native Mon:%d (Total: %d)", d.tomsCount or 0, d.monitorsCount or 0, d.totalScreens or 0)
            if cardH >= 26 then
                gpu.drawText(dispId, scrStr, startX + 8, c1Y + 4 + rowSp1 * 2, 220, 235, 255, "Arial", 8, "plain")
            end

            -- 2. PROPULSION NODES
            local c2Y = c1Y + cardH + 4
            gpu.fillRect(dispId, startX, c2Y, cardW, cardH, 20, 30, 43)
            gpu.drawText(dispId, isFull and "[2] PROPULSION ENGINE NODES" or "[2] ENGINES", startX + 6, c2Y + 3, 80, 230, 255, "Arial", 9, "bold")
            local el = e.lift or {}
            local ec = e.cruise or {}
            local es = e.steer or {}
            local rowSp2 = math.max(9, math.floor((cardH - 12) / 3))
            local liftStr = string.format("Lift (4-Quad): FL:%d FR:%d BL:%d BR:%d (Act: %d/%d)", el.FL or 0, el.FR or 0, el.BL or 0, el.BR or 0, el.online or 0, el.total or 0)
            local liftCol = (el.online or 0) > 0 and {80, 255, 120} or {255, 80, 80}
            gpu.drawText(dispId, liftStr, startX + 8, c2Y + 3 + rowSp2, liftCol[1], liftCol[2], liftCol[3], "Arial", 8, "bold")
            local cruiseStr = string.format("Cruise Thrust: FWD:%d | BWD:%d (Act: %d/%d)", ec.FWD or 0, ec.BWD or 0, ec.online or 0, ec.total or 0)
            local cruiseCol = (ec.online or 0) > 0 and {80, 255, 120} or {180, 200, 220}
            if cardH >= 24 then
                gpu.drawText(dispId, cruiseStr, startX + 8, c2Y + 3 + rowSp2 * 2, cruiseCol[1], cruiseCol[2], cruiseCol[3], "Arial", 8, "plain")
            end
            local steerStr = string.format("Lateral/Steer: LEFT:%d | RIGHT:%d (Act: %d/%d)", es.LEFT or 0, es.RIGHT or 0, es.online or 0, es.total or 0)
            local steerCol = (es.online or 0) > 0 and {80, 255, 120} or {180, 200, 220}
            if cardH >= 34 then
                gpu.drawText(dispId, steerStr, startX + 8, c2Y + 3 + rowSp2 * 3, steerCol[1], steerCol[2], steerCol[3], "Arial", 8, "plain")
            end

            -- 3. AVIONICS & SENSORS
            local c3Y = c2Y + cardH + 4
            gpu.fillRect(dispId, startX, c3Y, cardW, cardH, 20, 30, 43)
            gpu.drawText(dispId, isFull and "[3] AVIONICS & SENSORS" or "[3] SENSORS", startX + 6, c3Y + 3, 80, 230, 255, "Arial", 9, "bold")
            local aicStatus = a.onBody and "[DETECTED (ON BODY)]" or (a.aic and "[DETECTED]" or "[STANDBY/SEARCHING]")
            local aicCol = (a.onBody or a.aic) and {80, 255, 120} or {255, 160, 60}
            local rowSp3 = math.max(9, math.floor((cardH - 12) / 3))
            gpu.drawText(dispId, "CCPE AIC/FMC : " .. aicStatus, startX + 8, c3Y + 3 + rowSp3, aicCol[1], aicCol[2], aicCol[3], "Arial", 8, "bold")
            local sensStr = string.format("Sensors & Net: Alti:%s | Gyro:%s | NavTab:%s | Modems:%d", a.alti and "ON" or "OFF", a.gimbal and "ON" or "OFF", a.navTable and "ON" or "OFF", a.modems or 0)
            if cardH >= 24 then
                gpu.drawText(dispId, sensStr, startX + 8, c3Y + 3 + rowSp3 * 2, 220, 235, 255, "Arial", 8, "plain")
            end
            local navPosStr = ""
            if FlightCore.nav.x and FlightCore.nav.z then
                navPosStr = string.format("Position Fix : X:%.0f Y:%.0f Z:%.0f (Source: %s)", FlightCore.nav.x, FlightCore.nav.y or 0, FlightCore.nav.z, a.navSource or "INS")
            else
                navPosStr = "Position Fix : [ SEARCHING AIC / GPS / NAV_TABLE ]"
            end
            if cardH >= 34 then
                local posCol = FlightCore.nav.x and {80, 255, 120} or {255, 205, 75}
                gpu.drawText(dispId, navPosStr, startX + 8, c3Y + 3 + rowSp3 * 3, posCol[1], posCol[2], posCol[3], "Arial", 8, "plain")
            end

            local btmW = math.floor((sw - 16) / 2)
            addBtn(6, btmY, btmW, btmSysH, isFull and "[ RE-SCAN HARDWARE ]" or "RE-SCAN HW", {25, 100, 190}, {255, 255, 255}, function() FlightCore.scanQuadTurtles(); FlightCore.state.statusMsg="Re-scanned!" end)
            addBtn(6 + btmW + 4, btmY, btmW, btmSysH, isFull and "[ STOP ALL ENGINES ]" or "STOP ALL", {190, 40, 40}, {255, 255, 255}, function() FlightCore.stopEngines() end)
end
