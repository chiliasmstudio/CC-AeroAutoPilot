-- normal NAV view renderer.
-- Shared drawing helpers and layout values are supplied by the parent driver.
return function(self, scr, ctx)
    local FlightCore = ctx.FlightCore
    local addBtn = ctx.addBtn
    local h = ctx.h
    local isFull = ctx.isFull
    local w = ctx.w
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
end
