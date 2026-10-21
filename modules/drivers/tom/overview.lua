-- tom OVERVIEW view renderer.
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

            if isFull then
                -- 5x5 或以上：完整航電儀表板 (Full Avionics Dashboard)
                local leftW = math.max(140, math.floor(sw * 0.38))
                local pfdX = 6
                local ecamX = pfdX + leftW + 6
                local ecamW = sw - ecamX - 6

                -- 左側卡片: 主飛行儀表 (PFD)
                sFR(pfdX, instY, leftW, instH, 0x141A26)
                sR(pfdX, instY, leftW, instH, 0x283850)
                sTxt(pfdX + 6, instY + 4, "PRIMARY FLIGHT", 0xAAD2E6, 1)

                local gR = math.max(16, math.min(36, math.floor(instH * 0.26), math.floor(leftW * 0.24)))
                local gCX = pfdX + math.floor(leftW * 0.25)
                local gCY = instY + math.floor(instH * 0.54)
                for dy = -gR, gR do
                    local dx = math.floor(math.sqrt(math.max(0, gR*gR - dy*dy)) + 0.5)
                    sL(gCX - dx, gCY + dy, gCX + dx, gCY + dy, 0x1E2837)
                end
                sArc(gCX, gCY, gR, 0, 360, 10, 0x50AAF0)
                local altText = string.format("%.0f", currAlt)
                local altSz = (gR >= 22) and 2 or 1
                sTxt(gCX - math.floor(#altText * 6 * altSz / 2), gCY - 3 * altSz - 1, altText, 0xFFFFFF, altSz)

                local textX = pfdX + math.floor(leftW * 0.52)
                local rowSpacing = math.max(12, math.floor((instH - 24) / 4))
                sTxt(textX, instY + 14, string.format("TGT: %.0fm", FlightCore.state.targetAlt), 0x50E6FF, 1)
                sTxt(textX, instY + 14 + rowSpacing, string.format("V.S: %+.2f", currVspeed), 0xFFCD4B, 1)
                local mCol = (FlightCore.state.mode == "HOLD_ALT") and 0x50FF78 or 0xFF5050
                sTxt(textX, instY + 14 + rowSpacing * 2, string.format("MOD: %s", FlightCore.state.mode:sub(1,6)), mCol, 1)
                sTxt(textX, instY + 14 + rowSpacing * 3, string.format("P:%+2.0f R:%+2.0f", currPitch, currRoll), 0xB4DCFF, 1)

                -- 右側卡片: 2x2 四象限發動機動力監控
                sFR(ecamX, instY, ecamW, instH, 0x121822)
                sR(ecamX, instY, ecamW, instH, 0x283850)

                local subW = math.floor((ecamW - 8) / 2)
                local subH = math.floor((instH - 8) / 2)
                local miniSlots = {
                    {slot="FL", col=1, row=1, name="FRONT-LEFT"},
                    {slot="FR", col=2, row=1, name="FRONT-RIGHT"},
                    {slot="BL", col=1, row=2, name="BACK-LEFT"},
                    {slot="BR", col=2, row=2, name="BACK-RIGHT"}
                }
                for _, ms in ipairs(miniSlots) do
                    local sx = ecamX + 3 + (ms.col - 1) * (subW + 2)
                    local sy = instY + 3 + (ms.row - 1) * (subH + 2)
                    local qH = FlightCore.getQuadHealth(ms.slot)
                    local val = FlightCore.virtualOutputs[ms.slot]
                    local online = qH.online > 0

                    sFR(sx, sy, subW, subH, 0x18202E)
                    sR(sx, sy, subW, subH, online and 0x284864 or 0x642828)

                    local slotTitle = (subW < 100) and string.format("[%s]", ms.slot) or string.format("[%s] %d ENG", ms.slot, qH.total)
                    sTxt(sx + 4, sy + 3, slotTitle, 0xC8E6FF, 1)
                    local onCol = online and 0x50FF78 or 0xFF5050
                    local onText = (subW < 80) and string.format("%d/%d", qH.online, qH.total) or string.format("%d/%d ON", qH.online, qH.total)
                    sTxt(sx + subW - #onText * 6 - 4, sy + 3, onText, onCol, 1)

                    -- 動態油門條
                    local barY = sy + math.floor(subH * 0.38)
                    local barW = subW - 8
                    local barH = math.max(6, math.min(14, math.floor(subH * 0.20)))
                    sFR(sx + 4, barY, barW, barH, 0x0C121C)
                    local fillW = math.floor(barW * math.min(1.0, math.max(0.0, val / 15.0)))
                    local barCol = (val > 12) and 0xFF4500 or ((val > 8) and 0xFFD700 or 0x32CD32)
                    if fillW > 0 then sFR(sx + 4, barY, fillW, barH, barCol) end
                    sR(sx + 4, barY, barW, barH, 0x3C4B64)

                    -- 數值指示
                    local thrText = (subW < 90) and string.format("%.1f", val) or string.format("THR: %.1f", val)
                    sTxt(sx + 4, sy + subH - 10, thrText, 0x46FF78, 1)
                    local pwmText = string.format("PWM:%d", FlightCore.engineOutputs[ms.slot] or 0)
                    sTxt(sx + subW - #pwmText * 6 - 4, sy + subH - 10, pwmText, 0x96BEE1, 1)
                end

                -- 中段狀態通報
                sFR(pfdX, statY, sw - 12, statH, 0x1E2432)
                sR(pfdX, statY, sw - 12, statH, 0x3C4B64)
                local navInfo = ""
                if FlightCore.nav.x then
                    navInfo = string.format("POS:[%s] X:%.0f Y:%.0f Z:%.0f | HDG:%03d* | SPD:%.1fm/s | ", FlightCore.nav.source, FlightCore.nav.x, FlightCore.nav.y or currAlt, FlightCore.nav.z, math.floor(FlightCore.nav.yaw), FlightCore.nav.speed)
                else
                    navInfo = string.format("HDG:%03d* | ", math.floor(FlightCore.nav.yaw))
                end
                sTxt(pfdX + 6, statY + 4, navInfo .. string.format("STATUS: %s", FlightCore.state.statusMsg:sub(1, 35)), 0x50E6FF, 1)

                -- 底部 3 排控制按鈕群 (高度與推力控制，無 XZ 按鈕)
                local bY1 = btnAreaY
                local bW1 = math.floor((sw - 12 - 20) / 6)
                addBtn(pfdX, bY1, bW1, btnRowH, "+50m", 0x145A32, 0xFFFFFF, function() FlightCore.adjustTargetAlt(50) end)
                addBtn(pfdX + (bW1+4), bY1, bW1, btnRowH, "+10m", 0x1B5E20, 0xFFFFFF, function() FlightCore.adjustTargetAlt(10) end)
                addBtn(pfdX + (bW1+4)*2, bY1, bW1, btnRowH, "+1m", 0x2E7D32, 0xFFFFFF, function() FlightCore.adjustTargetAlt(1) end)
                addBtn(pfdX + (bW1+4)*3, bY1, bW1, btnRowH, "-1m", 0xE65100, 0xFFFFFF, function() FlightCore.adjustTargetAlt(-1) end)
                addBtn(pfdX + (bW1+4)*4, bY1, bW1, btnRowH, "-10m", 0xC62828, 0xFFFFFF, function() FlightCore.adjustTargetAlt(-10) end)
                addBtn(pfdX + (bW1+4)*5, bY1, bW1, btnRowH, "LOCK", 0x00838F, 0xFFFFFF, function() FlightCore.lockCurrentAlt() end)

                local bY2 = bY1 + btnRowH + btnSpacing
                local bW2 = math.floor((sw - 12 - 12) / 4)
                addBtn(pfdX, bY2, bW2, btnRowH, "BASE +1", 0x00695C, 0xFFFFFF, function() FlightCore.adjustBaseThrottle(1.0) end)
                addBtn(pfdX + bW2 + 4, bY2, bW2, btnRowH, "BASE -1", 0x37474F, 0xFFFFFF, function() FlightCore.adjustBaseThrottle(-1.0) end)
                addBtn(pfdX + (bW2 + 4)*2, bY2, bW2, btnRowH, "RE-SCAN", 0x1565C0, 0xFFFFFF, function() FlightCore.scanQuadTurtles(); FlightCore.state.statusMsg="Re-scanned!" end)
                addBtn(pfdX + (bW2 + 4)*3, bY2, bW2, btnRowH, "CALIB", 0x6A1B9A, 0xFFFFFF, function() FlightCore.startCalibration() end)

                local bY3 = bY2 + btnRowH + btnSpacing
                local bW3 = math.floor((sw - 12 - 4) / 2)
                local holdBg = (FlightCore.state.mode == "HOLD_ALT") and 0x2E7D32 or 0x1B5E20
                addBtn(pfdX, bY3, bW3, btnRowH, "[ HOLD ALTITUDE ]", holdBg, 0xFFFFFF, function() FlightCore.holdAltitude() end)
                local stopBg = (FlightCore.state.mode == "IDLE") and 0xC62828 or 0xB71C1C
                addBtn(pfdX + bW3 + 4, bY3, bW3, btnRowH, "[ STOP / IDLE ]", stopBg, 0xFFFFFF, function() FlightCore.stopEngines() end)
            else
                -- 3x3 ~ 4x5 精簡模式 (<5x5 Compact Mode, 適配 384x384 到 512x640 及小螢幕)
                local colW = math.floor((sw - 12) / 2)
                local pfdX = 4
                local ecamX = pfdX + colW + 4

                if colW >= 85 then
                    -- 寬度充足精簡版 (4x4, 5x4): 圓形高度錶 + 2x2 四象限進度條
                    sFR(pfdX, instY, colW, instH, 0x141A26)
                    sR(pfdX, instY, colW, instH, 0x283850)
                    sTxt(pfdX + 4, instY + 3, "ALT/ATT", 0xAAD2E6, 1)

                    local gR = math.max(8, math.min(22, math.floor(colW * 0.18), math.floor(instH * 0.22)))
                    local gCX = pfdX + math.max(gR + 4, math.floor(colW * 0.24))
                    local gCY = instY + math.floor(instH * 0.54)
                    for dy = -gR, gR do
                        local dx = math.floor(math.sqrt(math.max(0, gR*gR - dy*dy)) + 0.5)
                        sL(gCX - dx, gCY + dy, gCX + dx, gCY + dy, 0x1E2837)
                    end
                    sArc(gCX, gCY, gR, 0, 360, 15, 0x50AAF0)
                    local altText = string.format("%.0f", currAlt)
                    sTxt(gCX - math.floor(#altText * 3), gCY - 3, altText, 0xFFFFFF, 1)

                    local textX = gCX + gR + 6
                    local rowSpacing = math.max(8, math.min(18, math.floor((instH - 18) / 4)))
                    sTxt(textX, instY + 10, string.format("T:%.0f", FlightCore.state.targetAlt), 0x50E6FF, 1)
                    sTxt(textX, instY + 10 + rowSpacing, string.format("V:%+.1f", currVspeed), 0xFFCD4B, 1)
                    local mCol = (FlightCore.state.mode == "HOLD_ALT") and 0x50FF78 or 0xFF5050
                    sTxt(textX, instY + 10 + rowSpacing * 2, string.format("M:%s", FlightCore.state.mode:sub(1,4)), mCol, 1)
                    sTxt(textX, instY + 10 + rowSpacing * 3, string.format("H:%03d*", math.floor(FlightCore.nav.yaw)), 0xB4DCFF, 1)

                    -- 右側: 2x2 Mini Quad Engines (FL/FR/BL/BR)
                    sFR(ecamX, instY, colW, instH, 0x121822)
                    sR(ecamX, instY, colW, instH, 0x283850)

                    local subW = math.floor((colW - 6) / 2)
                    local subH = math.floor((instH - 6) / 2)
                    local miniSlots = {
                        {slot="FL", col=1, row=1},
                        {slot="FR", col=2, row=1},
                        {slot="BL", col=1, row=2},
                        {slot="BR", col=2, row=2}
                    }
                    for _, ms in ipairs(miniSlots) do
                        local sx = ecamX + 2 + (ms.col - 1) * (subW + 2)
                        local sy = instY + 2 + (ms.row - 1) * (subH + 2)
                        local qH = FlightCore.getQuadHealth(ms.slot)
                        local val = FlightCore.virtualOutputs[ms.slot]
                        local online = qH.online > 0

                        sFR(sx, sy, subW, subH, 0x18202E)
                        sR(sx, sy, subW, subH, online and 0x284864 or 0x642828)
                        sTxt(sx + 2, sy + 2, string.format("[%s] %dE", ms.slot, qH.total), 0xC8E6FF, 1)

                        -- 動態微型進度條 (帶底框和邊框)
                        local barY = sy + math.floor(subH * 0.40)
                        local barW = math.max(4, subW - 6)
                        local barH = math.max(4, math.min(10, math.floor(subH * 0.18)))
                        sFR(sx + 3, barY, barW, barH, 0x0A1018)
                        local fillW = math.floor(barW * math.min(1.0, math.max(0.0, val / 15.0)))
                        local barCol = (val > 12) and 0xFF4500 or ((val > 8) and 0xFFD700 or 0x32CD32)
                        if fillW > 0 then sFR(sx + 3, barY, fillW, barH, barCol) end
                        sR(sx + 3, barY, barW, barH, 0x384860)

                        -- 數值指示
                        local btmLineY = sy + subH - 9
                        sTxt(sx + 2, btmLineY, string.format("%4.1f", val), 0x46FF78, 1)
                        local pwmStr = string.format("P:%d", FlightCore.engineOutputs[ms.slot] or 0)
                        sTxt(sx + subW - #pwmStr * 6 - 2, btmLineY, pwmStr, 0x96BEE1, 1)
                    end
                else
                    -- 極致窄螢幕 (3x3 螢幕, colW < 85px): 乾淨直向數據流，零碰撞
                    -- 左側: PFD 飛行姿態垂直數據堆疊
                    sFR(pfdX, instY, colW, instH, 0x141A26)
                    sR(pfdX, instY, colW, instH, 0x283850)
                    sTxt(pfdX + 3, instY + 2, "ALT/ATT", 0xAAD2E6, 1)

                    local rowSp = math.max(8, math.min(12, math.floor((instH - 14) / 4)))
                    local yBase = instY + 13
                    sTxt(pfdX + 3, yBase, string.format("ALT:%4.0f", currAlt), 0xFFFFFF, 1)
                    sTxt(pfdX + 3, yBase + rowSp, string.format("TGT:%4.0f", FlightCore.state.targetAlt), 0x50E6FF, 1)
                    sTxt(pfdX + 3, yBase + rowSp * 2, string.format("V.S:%+4.1f", currVspeed), 0xFFCD4B, 1)
                    local mCol = (FlightCore.state.mode == "HOLD_ALT") and 0x50FF78 or 0xFF5050
                    sTxt(pfdX + 3, yBase + rowSp * 3, string.format("HDG:%03d*", math.floor(FlightCore.nav.yaw)), mCol, 1)

                    -- 右側: ECAM 四象限發動機直向數據堆疊 (FL, FR, BL, BR 逐行排布)
                    sFR(ecamX, instY, colW, instH, 0x121822)
                    sR(ecamX, instY, colW, instH, 0x283850)
                    sTxt(ecamX + 3, instY + 2, "ECAM 4Q", 0xAAD2E6, 1)

                    local qSlots = {"FL", "FR", "BL", "BR"}
                    for idx, slot in ipairs(qSlots) do
                        local qH = FlightCore.getQuadHealth(slot)
                        local val = FlightCore.virtualOutputs[slot]
                        local sig = FlightCore.engineOutputs[slot] or 0
                        local yPos = yBase + (idx - 1) * rowSp
                        local colVal = (val > 12) and 0xFF4500 or ((val > 8) and 0xFFD700 or 0x50FF78)
                        if qH.online == 0 then colVal = 0xFF5050 end
                        sTxt(ecamX + 3, yPos, string.format("%s%4.1f P%d", slot, val, sig), colVal, 1)
                    end
                end

                -- 底部狀態列 (包含導航簡報)
                sFR(pfdX, statY, sw - 8, statH, 0x1E2432)
                sR(pfdX, statY, sw - 8, statH, 0x3C4B64)
                local navShort = ""
                if FlightCore.nav.x then
                    navShort = string.format("[%s] X:%.0f Z:%.0f H:%03d* | ", FlightCore.nav.source, FlightCore.nav.x, FlightCore.nav.z, math.floor(FlightCore.nav.yaw))
                else
                    navShort = string.format("H:%03d* | ", math.floor(FlightCore.nav.yaw))
                end
                local maxStatChars = math.max(6, math.floor((sw - 20) / 6) - #navShort)
                sTxt(pfdX + 4, statY + 3, navShort .. FlightCore.state.statusMsg:sub(1, maxStatChars), 0x50E6FF, 1)

                -- 底部 2 排按鈕 (高度與推力控制，無 XZ 按鈕)
                local bW4 = math.floor((sw - 8 - 9) / 4)
                addBtn(pfdX, btnAreaY, bW4, btnRowH, "+10", 0x1B5E20, 0xFFFFFF, function() FlightCore.adjustTargetAlt(10) end)
                addBtn(pfdX + bW4 + 3, btnAreaY, bW4, btnRowH, "-10", 0xC62828, 0xFFFFFF, function() FlightCore.adjustTargetAlt(-10) end)
                addBtn(pfdX + (bW4 + 3)*2, btnAreaY, bW4, btnRowH, "B+", 0x00695C, 0xFFFFFF, function() FlightCore.adjustBaseThrottle(1.0) end)
                addBtn(pfdX + (bW4 + 3)*3, btnAreaY, bW4, btnRowH, "B-", 0x37474F, 0xFFFFFF, function() FlightCore.adjustBaseThrottle(-1.0) end)

                local bY2 = btnAreaY + btnRowH + btnSpacing
                local bW2 = math.floor((sw - 8 - 3) / 2)
                local holdBg = (FlightCore.state.mode == "HOLD_ALT") and 0x2E7D32 or 0x1B5E20
                addBtn(pfdX, bY2, bW2, btnRowH, "HOLD", holdBg, 0xFFFFFF, function() FlightCore.holdAltitude() end)
                local stopBg = (FlightCore.state.mode == "IDLE") and 0xC62828 or 0xB71C1C
                addBtn(pfdX + bW2 + 3, bY2, bW2, btnRowH, "STOP", stopBg, 0xFFFFFF, function() FlightCore.stopEngines() end)
            end
end
