-- directgpu NAV view renderer.
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
end
