-- tom PFD view renderer.
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

            local mainH = sh - headerH - btnRowH - 12
            local mainY = headerH + 4

            local leftW = math.floor(sw * 0.52)
            sFR(6, mainY, leftW, mainH, 0x141A26)
            sR(6, mainY, leftW, mainH, 0x283850)

            local hCX = 6 + math.floor(leftW / 2)
            local hCY = mainY + math.floor(mainH / 2)
            local hR = math.min(math.floor(leftW * 0.38), math.floor(mainH * 0.40))
            sArc(hCX, hCY, hR, 0, 360, 10, 0x50AAF0)

            local rollRad = math.rad(-currRoll)
            local lx1 = math.floor(hCX - hR * 0.85 * math.cos(rollRad) + 0.5)
            local ly1 = math.floor(hCY - hR * 0.85 * math.sin(rollRad) + 0.5)
            local lx2 = math.floor(hCX + hR * 0.85 * math.cos(rollRad) + 0.5)
            local ly2 = math.floor(hCY + hR * 0.85 * math.sin(rollRad) + 0.5)
            sLS(lx1, ly1, lx2, ly2, 0xFFD700)
            if leftW >= 80 then
                sTxt(10, mainY + mainH - 12, string.format("P:%+3.0f* R:%+3.0f*", currPitch, currRoll), 0xC8E6FF, 1)
            end

            local rightX = 6 + leftW + 5
            local rightW = sw - rightX - 6
            sFR(rightX, mainY, rightW, mainH, 0x18202E)
            sR(rightX, mainY, rightW, mainH, 0x283850)

            sTxt(rightX + 4, mainY + 4, rightW < 55 and "ALT" or "CURRENT ALT", 0x8CA0B4, 1)
            local altStr = string.format("%.0fm", currAlt)
            local altSz = (isFull and rightW >= 60) and 2 or 1
            sTxt(rightX + 4, mainY + 16, altStr, 0xFFFFFF, altSz)

            local midY = mainY + (isFull and 36 or 28)
            sTxt(rightX + 4, midY, rightW < 55 and string.format("T:%.0f", FlightCore.state.targetAlt) or string.format("TGT: %.0fm", FlightCore.state.targetAlt), 0x50E6FF, 1)
            sTxt(rightX + 4, midY + 11, rightW < 55 and string.format("V:%+.1f", currVspeed) or string.format("V.S: %+.2f", currVspeed), 0xFFCD4B, 1)

            local bY = mainY + mainH + 4
            local bW = math.floor((sw - 12 - 12) / 4)
            addBtn(6, bY, bW, btnRowH, isFull and "+10m" or "+10", 0x1E642D, 0xFFFFFF, function() FlightCore.adjustTargetAlt(10) end)
            addBtn(6 + (bW+4), bY, bW, btnRowH, isFull and "-10m" or "-10", 0xBE2828, 0xFFFFFF, function() FlightCore.adjustTargetAlt(-10) end)
            local holdBg = (FlightCore.state.mode == "HOLD_ALT") and 0x2D8C3C or 0x1E5A28
            addBtn(6 + (bW+4)*2, bY, bW, btnRowH, "HOLD", holdBg, 0xFFFFFF, function() FlightCore.holdAltitude() end)
            local stopBg = (FlightCore.state.mode == "IDLE") and 0xC82D2D or 0xA01E1E
            addBtn(6 + (bW+4)*3, bY, bW, btnRowH, "STOP", stopBg, 0xFFFFFF, function() FlightCore.stopEngines() end)
end
