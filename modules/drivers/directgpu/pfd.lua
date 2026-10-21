-- directgpu PFD view renderer.
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

            local mainH = sh - headerH - btnRowH - 12
            local mainY = headerH + 4

            local leftW = math.floor(sw * 0.52)
            gpu.fillRect(dispId, 6, mainY, leftW, mainH, 20, 26, 38)

            local hCX = 6 + math.floor(leftW / 2)
            local hCY = mainY + math.floor(mainH / 2)
            local hR = math.min(math.floor(leftW * 0.38), math.floor(mainH * 0.40))
            gpu.drawCircle(dispId, hCX, hCY, hR, 80, 170, 240, false)

            local rollRad = math.rad(-currRoll)
            local lx1 = math.floor(hCX - hR * 0.85 * math.cos(rollRad) + 0.5)
            local ly1 = math.floor(hCY - hR * 0.85 * math.sin(rollRad) + 0.5)
            local lx2 = math.floor(hCX + hR * 0.85 * math.cos(rollRad) + 0.5)
            local ly2 = math.floor(hCY + hR * 0.85 * math.sin(rollRad) + 0.5)
            gpu.drawLine(dispId, lx1, ly1, lx2, ly2, 255, 215, 0)
            gpu.drawCircle(dispId, hCX, hCY, 3, 255, 80, 80, true)
            if leftW >= 80 then
                gpu.drawText(dispId, string.format("P:%+3.0f* R:%+3.0f*", currPitch, currRoll), 10, mainY + mainH - 12, 200, 230, 255, "Arial", 9, "bold")
            end

            local rightX = 6 + leftW + 5
            local rightW = sw - rightX - 6
            gpu.fillRect(dispId, rightX, mainY, rightW, mainH, 24, 32, 46)

            gpu.drawText(dispId, rightW < 55 and "ALT" or "CURRENT ALT", rightX + 4, mainY + 4, 140, 160, 180, "Arial", 8, "plain")
            local altStr = string.format("%.0fm", currAlt)
            local altSz = (isFull and rightW >= 60) and 14 or 10
            gpu.drawText(dispId, altStr, rightX + 4, mainY + 16, 255, 255, 255, "Arial", altSz, "bold")

            local midY = mainY + (isFull and 36 or 28)
            gpu.drawText(dispId, rightW < 55 and string.format("T:%.0f", FlightCore.state.targetAlt) or string.format("TGT: %.0fm", FlightCore.state.targetAlt), rightX + 4, midY, 80, 230, 255, "Arial", 8, "bold")
            gpu.drawText(dispId, rightW < 55 and string.format("V:%+.1f", currVspeed) or string.format("V.S: %+.2f", currVspeed), rightX + 4, midY + 11, 255, 205, 75, "Arial", 8, "bold")

            local bY = mainY + mainH + 4
            local bW = math.floor((sw - 12 - 12) / 4)
            addBtn(6, bY, bW, btnRowH, "+10m", {30, 100, 45}, {255, 255, 255}, function() FlightCore.adjustTargetAlt(10) end)
            addBtn(6 + (bW+4), bY, bW, btnRowH, "-10m", {190, 40, 40}, {255, 255, 255}, function() FlightCore.adjustTargetAlt(-10) end)
            local holdBg = (FlightCore.state.mode == "HOLD_ALT") and {45, 140, 60} or {30, 90, 40}
            addBtn(6 + (bW+4)*2, bY, bW, btnRowH, "HOLD", holdBg, {255, 255, 255}, function() FlightCore.holdAltitude() end)
            local stopBg = (FlightCore.state.mode == "IDLE") and {200, 45, 45} or {160, 30, 30}
            addBtn(6 + (bW+4)*3, bY, bW, btnRowH, "STOP", stopBg, {255, 255, 255}, function() FlightCore.stopEngines() end)
end
