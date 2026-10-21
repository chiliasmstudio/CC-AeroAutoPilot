-- tom NAV view renderer.
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
            local currAlt = FlightCore.altiSensor and FlightCore.altiSensor.getHeight() or 0
            local currVspeed = FlightCore.altiSensor and FlightCore.altiSensor.getVerticalSpeed() or 0
            local currPitch, currRoll = FlightCore.getGimbalData()

            local statNavH = isFull and 36 or 22
            sFR(6, headerH + 4, sw - 12, statNavH, 0x141C28)
            sR(6, headerH + 4, sw - 12, statNavH, 0x284864)

            local holdStr = FlightCore.nav.headingHold and "ON" or "OFF"

            if isFull then
                local posStr = ""
                if FlightCore.nav.x then
                    posStr = string.format("POS:[%s] X:%+6.0f Y:%4.0f Z:%+6.0f | SPD: %4.1f m/s", FlightCore.nav.source, FlightCore.nav.x, FlightCore.nav.y or currAlt, FlightCore.nav.z, FlightCore.nav.speed)
                else
                    posStr = string.format("POS:[%s] ALT: %4.0fm | GYRO: P:%+2.0f* R:%+2.0f*", FlightCore.nav.source, currAlt, currPitch, currRoll)
                end
                sTxt(10, headerH + 7, posStr, 0x8CA0B4, 1)
                local hdgStr = string.format("HDG: %03d*  |  TARGET HDG: %03d*  |  HEADING HOLD: [%s]", math.floor(FlightCore.nav.yaw), FlightCore.nav.targetHeading, holdStr)
                sTxt(10, headerH + 21, hdgStr, 0x50E6FF, 1)
            else
                if FlightCore.nav.x and sw >= 180 then
                    sTxt(8, headerH + 6, string.format("[%s] X:%.0f Z:%.0f S:%.1f", FlightCore.nav.source, FlightCore.nav.x, FlightCore.nav.z, FlightCore.nav.speed), 0x8CA0B4, 1)
                else
                    sTxt(8, headerH + 6, string.format("ALT:%.0f (V:%+.1f)", currAlt, currVspeed), 0x8CA0B4, 1)
                end
                sTxt(8, headerH + 15, string.format("H:%03d* -> T:%03d* [%s]", math.floor(FlightCore.nav.yaw), FlightCore.nav.targetHeading, holdStr), 0x50E6FF, 1)
            end

            local gridY = headerH + 4 + statNavH + 4
            local btnAreaH = sh - gridY - 4
            local rowH = math.floor((btnAreaH - 8) / 3)

            -- Row 1: 航向微調/步進調整 (Heading Step Adjustments)
            local bW4 = math.floor((sw - 12 - 12) / 4)
            addBtn(6, gridY, bW4, rowH, isFull and "[ HDG -45* ]" or "-45*", 0x144664, 0xFFFFFF, function() FlightCore.adjustTargetHeading(-45) end)
            addBtn(6 + (bW4+4), gridY, bW4, rowH, isFull and "[ HDG -5* ]" or "-5*", 0x1E5A78, 0xFFFFFF, function() FlightCore.adjustTargetHeading(-5) end)
            addBtn(6 + (bW4+4)*2, gridY, bW4, rowH, isFull and "[ HDG +5* ]" or "+5*", 0x1E5A78, 0xFFFFFF, function() FlightCore.adjustTargetHeading(5) end)
            addBtn(6 + (bW4+4)*3, gridY, bW4, rowH, isFull and "[ HDG +45* ]" or "+45*", 0x144664, 0xFFFFFF, function() FlightCore.adjustTargetHeading(45) end)

            -- Row 2: 四大主方位快速鎖定 (Cardinal Direction Presets)
            local r2Y = gridY + rowH + 4
            addBtn(6, r2Y, bW4, rowH, isFull and "[ 000* NORTH ]" or "0* N", 0x1B5E20, 0xFFFFFF, function() FlightCore.setTargetHeading(0) end)
            addBtn(6 + (bW4+4), r2Y, bW4, rowH, isFull and "[ 090* EAST ]" or "90* E", 0x2E7D32, 0xFFFFFF, function() FlightCore.setTargetHeading(90) end)
            addBtn(6 + (bW4+4)*2, r2Y, bW4, rowH, isFull and "[ 180* SOUTH ]" or "180* S", 0x00838F, 0xFFFFFF, function() FlightCore.setTargetHeading(180) end)
            addBtn(6 + (bW4+4)*3, r2Y, bW4, rowH, isFull and "[ 270* WEST ]" or "270* W", 0x00695C, 0xFFFFFF, function() FlightCore.setTargetHeading(270) end)

            -- Row 3: 自駕儀航向保持與快速功能 (Heading Hold Autopilot & Ops)
            local r3Y = r2Y + rowH + 4
            local holdBg = FlightCore.nav.headingHold and 0x2E7D32 or 0x37474F
            local holdText = isFull and (FlightCore.nav.headingHold and "[ HDG HOLD: ON ]" or "[ HDG HOLD: OFF ]") or (FlightCore.nav.headingHold and "HOLD:ON" or "HOLD:OFF")
            addBtn(6, r3Y, bW4, rowH, holdText, holdBg, 0xFFFFFF, function() FlightCore.toggleHeadingHold() end)
            addBtn(6 + (bW4+4), r3Y, bW4, rowH, isFull and "[ SYNC HDG ]" or "SYNC HDG", 0x1565C0, 0xFFFFFF, function() FlightCore.syncHeading() end)
            local altHoldBg = (FlightCore.state.mode == "HOLD_ALT") and 0x2E7D32 or 0x1B5E20
            addBtn(6 + (bW4+4)*2, r3Y, bW4, rowH, isFull and "[ HOLD ALT ]" or "HOLD ALT", altHoldBg, 0xFFFFFF, function() FlightCore.holdAltitude() end)
            local stopBg = (FlightCore.state.mode == "IDLE") and 0xC62828 or 0xB71C1C
            addBtn(6 + (bW4+4)*3, r3Y, bW4, rowH, isFull and "[ STOP IDLE ]" or "STOP", stopBg, 0xFFFFFF, function() FlightCore.stopEngines() end)
end
