-- normal CTRL view renderer.
-- Shared drawing helpers and layout values are supplied by the parent driver.
return function(self, scr, ctx)
    local FlightCore = ctx.FlightCore
    local addBtn = ctx.addBtn
    local h = ctx.h
    local isFull = ctx.isFull
    local w = ctx.w
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
end
