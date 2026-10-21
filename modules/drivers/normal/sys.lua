-- normal SYS view renderer.
-- Shared drawing helpers and layout values are supplied by the parent driver.
return function(self, scr, ctx)
    local FlightCore = ctx.FlightCore
    local addBtn = ctx.addBtn
    local h = ctx.h
    local isFull = ctx.isFull
    local w = ctx.w
            local hw = (FlightCore.getHardwareInventory and FlightCore.getHardwareInventory()) or FlightCore.hw or {
                displays = { hasDirectGpu=false, tomsCount=0, monitorsCount=0, totalScreens=0 },
                engines = { lift={online=0, total=0}, cruise={online=0, total=0}, steer={online=0, total=0}, totalOnline=0, totalCount=0 },
                avionics = { onBody=false, aic=false, ins=false, navTable=false, alti=false, gimbal=false, modems=0, navSource="NONE" }
            }
            local d = hw.displays or {}
            local e = hw.engines or {}
            local a = hw.avionics or {}

            local curY = 4
            -- 1. DISPLAYS & GPU
            self:safeBlit(scr, 2, curY, isFull and "[1] DISPLAYS & GPU HARDWARE" or "[1] DISPLAYS & GPU", "9", "f")
            curY = curY + 1
            local dgpuStr = d.hasDirectGpu and "YES (DirectGPU HW Fast)" or "NO (DirectGPU Not Found)"
            local dgpuCol = d.hasDirectGpu and "d" or "7"
            self:safeBlit(scr, 3, curY, "DirectGPU  : " .. dgpuStr, dgpuCol, "f")
            curY = curY + 1
            local scrStr = string.format("Screens    : Tom's GPU:%d | Native Mon:%d (Total: %d)", d.tomsCount or 0, d.monitorsCount or 0, d.totalScreens or 0)
            self:safeBlit(scr, 3, curY, scrStr, "0", "f")
            curY = curY + 1

            -- 2. PROPULSION NODES
            curY = curY + 1
            self:safeBlit(scr, 2, curY, isFull and "[2] PROPULSION ENGINE NODES" or "[2] ENGINES", "9", "f")
            curY = curY + 1
            local el = e.lift or {}
            local ec = e.cruise or {}
            local es = e.steer or {}
            local liftStr = string.format("Lift (4-Quad): FL:%d FR:%d BL:%d BR:%d (Act: %d/%d)", el.FL or 0, el.FR or 0, el.BL or 0, el.BR or 0, el.online or 0, el.total or 0)
            self:safeBlit(scr, 3, curY, liftStr, (el.online or 0) > 0 and "5" or "e", "f")
            curY = curY + 1
            local cruiseStr = string.format("Cruise Thrust: FWD:%d | BWD:%d (Act: %d/%d)", ec.FWD or 0, ec.BWD or 0, ec.online or 0, ec.total or 0)
            self:safeBlit(scr, 3, curY, cruiseStr, (ec.online or 0) > 0 and "5" or "7", "f")
            curY = curY + 1
            local steerStr = string.format("Lateral/Steer: LEFT:%d | RIGHT:%d (Act: %d/%d)", es.LEFT or 0, es.RIGHT or 0, es.online or 0, es.total or 0)
            self:safeBlit(scr, 3, curY, steerStr, (es.online or 0) > 0 and "5" or "7", "f")
            curY = curY + 1

            -- 3. AVIONICS & SENSORS
            curY = curY + 1
            self:safeBlit(scr, 2, curY, isFull and "[3] AVIONICS & SENSORS" or "[3] SENSORS", "9", "f")
            curY = curY + 1
            local aicStatus = a.onBody and "[DETECTED (ON BODY)]" or (a.aic and "[DETECTED]" or "[STANDBY/SEARCHING]")
            local aicCol = (a.onBody or a.aic) and "d" or "e"
            self:safeBlit(scr, 3, curY, "CCPE AIC/FMC : " .. aicStatus, aicCol, "f")
            curY = curY + 1
            local sensStr = string.format("Sensors & Net: Alti:%s | Gyro:%s | NavTab:%s | Modems:%d", a.alti and "ON" or "OFF", a.gimbal and "ON" or "OFF", a.navTable and "ON" or "OFF", a.modems or 0)
            self:safeBlit(scr, 3, curY, sensStr, "0", "f")
            curY = curY + 1
            local navPosStr = ""
            if FlightCore.nav.x and FlightCore.nav.z then
                navPosStr = string.format("Position Fix : X:%.0f Y:%.0f Z:%.0f (Source: %s)", FlightCore.nav.x, FlightCore.nav.y or 0, FlightCore.nav.z, a.navSource or "INS")
            else
                navPosStr = "Position Fix : [ SEARCHING AIC / GPS / NAV_TABLE ]"
            end
            self:safeBlit(scr, 3, curY, navPosStr, FlightCore.nav.x and "d" or "e", "f")

            local btmY = h - 2
            local btmW = math.floor((w - 3) / 2)
            addBtn(2, btmY, btmW, 2, isFull and "[ RE-SCAN HARDWARE ]" or "RE-SCAN HW", "3", "0", function() FlightCore.scanQuadTurtles(); FlightCore.state.statusMsg="Re-scanned!" end)
            addBtn(3 + btmW, btmY, btmW, 2, isFull and "[ STOP ALL ENGINES ]" or "STOP ALL", "e", "0", function() FlightCore.stopEngines() end)
end
