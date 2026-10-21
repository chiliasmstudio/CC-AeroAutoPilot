-- normal OVERVIEW view renderer.
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
end
