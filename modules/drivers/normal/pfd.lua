-- normal PFD view renderer.
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
end
