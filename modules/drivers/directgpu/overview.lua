-- directgpu OVERVIEW view renderer.
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

            if isFull then
                -- 5x5 或以上：超大高解析度完整儀表 (Full Avionics Dashboard)
                local leftW = math.max(140, math.floor(sw * 0.38))
                local pfdX = 6
                local ecamX = pfdX + leftW + 6
                local ecamW = sw - ecamX - 6

                -- 左側卡片: 主飛行儀表 (PFD)
                gpu.fillRect(dispId, pfdX, instY, leftW, instH, 20, 26, 38)
                gpu.drawText(dispId, "PRIMARY FLIGHT", pfdX + 6, instY + 4, 170, 210, 230, "Arial", 11, "bold")

                local gR = math.max(16, math.min(36, math.floor(instH * 0.26), math.floor(leftW * 0.24)))
                local gCX = pfdX + math.floor(leftW * 0.25)
                local gCY = instY + math.floor(instH * 0.54)
                gpu.drawCircle(dispId, gCX, gCY, gR, 80, 170, 240, true)
                local altStr = string.format("%.0f", currAlt)
                local altSz = (gR >= 22) and 13 or 10
                gpu.drawText(dispId, altStr, gCX - math.floor(#altStr * 4), gCY - 6, 255, 255, 255, "Arial", altSz, "bold")

                local textX = pfdX + math.floor(leftW * 0.52)
                local rowSpacing = math.max(12, math.floor((instH - 24) / 4))
                gpu.drawText(dispId, string.format("TGT: %.0fm", FlightCore.state.targetAlt), textX, instY + 14, 80, 230, 255, "Arial", 10, "bold")
                gpu.drawText(dispId, string.format("V.S: %+.2f", currVspeed), textX, instY + 14 + rowSpacing, 255, 205, 75, "Arial", 10, "bold")
                local mCol = (FlightCore.state.mode == "HOLD_ALT") and {80, 255, 120} or {255, 80, 80}
                gpu.drawText(dispId, string.format("MOD: %s", FlightCore.state.mode:sub(1,6)), textX, instY + 14 + rowSpacing * 2, mCol[1], mCol[2], mCol[3], "Arial", 10, "bold")
                gpu.drawText(dispId, string.format("P:%+2.0f R:%+2.0f", currPitch, currRoll), textX, instY + 14 + rowSpacing * 3, 180, 220, 255, "Arial", 9, "plain")

                -- 右側卡片: 2x2 四象限發動機動力監控
                gpu.fillRect(dispId, ecamX, instY, ecamW, instH, 18, 24, 34)

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

                    gpu.fillRect(dispId, sx, sy, subW, subH, 24, 32, 46)
                    local slotTitle = (subW < 100) and string.format("[%s]", ms.slot) or string.format("[%s] %d ENG", ms.slot, qH.total)
                    gpu.drawText(dispId, slotTitle, sx + 4, sy + 3, 200, 230, 255, "Arial", 9, "bold")
                    local onCol = online and {80, 255, 120} or {255, 80, 80}
                    local onText = (subW < 80) and string.format("%d/%d", qH.online, qH.total) or string.format("%d/%d ON", qH.online, qH.total)
                    gpu.drawText(dispId, onText, sx + subW - math.floor(#onText * 6) - 4, sy + 3, onCol[1], onCol[2], onCol[3], "Arial", 9, "bold")

                    -- 動態油門條
                    local barY = sy + math.floor(subH * 0.38)
                    local barW = subW - 8
                    local barH = math.max(6, math.min(14, math.floor(subH * 0.20)))
                    gpu.fillRect(dispId, sx + 4, barY, barW, barH, 10, 18, 28)
                    local fillW = math.floor(barW * math.min(1.0, math.max(0.0, val / 15.0)))
                    local barCol = (val > 12) and {255, 69, 0} or ((val > 8) and {255, 215, 0} or {50, 205, 50})
                    if fillW > 0 then gpu.fillRect(dispId, sx + 4, barY, fillW, barH, barCol[1], barCol[2], barCol[3]) end

                    -- 數值指示
                    local thrText = (subW < 90) and string.format("%.1f", val) or string.format("THR: %.1f", val)
                    gpu.drawText(dispId, thrText, sx + 4, sy + subH - 11, 70, 255, 120, "Arial", 9, "bold")
                    local pwmText = string.format("PWM:%d", FlightCore.engineOutputs[ms.slot] or 0)
                    gpu.drawText(dispId, pwmText, sx + subW - math.floor(#pwmText * 6) - 4, sy + subH - 11, 150, 190, 225, "Arial", 8, "plain")
                end

                -- 中段狀態列 (包含導航簡報)
                gpu.fillRect(dispId, pfdX, statY, sw - 12, statH, 30, 36, 50)
                local navInfo = ""
                if FlightCore.nav.x then
                    navInfo = string.format("POS:[%s] X:%.0f Y:%.0f Z:%.0f | HDG:%03d* | SPD:%.1fm/s | ", FlightCore.nav.source, FlightCore.nav.x, FlightCore.nav.y or currAlt, FlightCore.nav.z, math.floor(FlightCore.nav.yaw), FlightCore.nav.speed)
                else
                    navInfo = string.format("HDG:%03d* | ", math.floor(FlightCore.nav.yaw))
                end
                gpu.drawText(dispId, navInfo .. string.format("STATUS: %s", FlightCore.state.statusMsg:sub(1,35)), pfdX + 8, statY + 4, 80, 230, 255, "Arial", 10, "bold")

                -- 底部 3 排控制按鈕群
                local bY1 = btnAreaY
                local bW1 = math.floor((sw - 12 - 20) / 6)
                addBtn(pfdX, bY1, bW1, btnRowH, "+50m", {20, 90, 50}, {255, 255, 255}, function() FlightCore.adjustTargetAlt(50) end)
                addBtn(pfdX + (bW1 + 4), bY1, bW1, btnRowH, "+10m", {27, 94, 32}, {255, 255, 255}, function() FlightCore.adjustTargetAlt(10) end)
                addBtn(pfdX + (bW1 + 4)*2, bY1, bW1, btnRowH, "+1m", {46, 125, 50}, {255, 255, 255}, function() FlightCore.adjustTargetAlt(1) end)
                addBtn(pfdX + (bW1 + 4)*3, bY1, bW1, btnRowH, "-1m", {230, 81, 0}, {255, 255, 255}, function() FlightCore.adjustTargetAlt(-1) end)
                addBtn(pfdX + (bW1 + 4)*4, bY1, bW1, btnRowH, "-10m", {198, 40, 40}, {255, 255, 255}, function() FlightCore.adjustTargetAlt(-10) end)
                addBtn(pfdX + (bW1 + 4)*5, bY1, bW1, btnRowH, "LOCK", {0, 131, 143}, {255, 255, 255}, function() FlightCore.lockCurrentAlt() end)

                local bY2 = bY1 + btnRowH + btnSpacing
                local bW2 = math.floor((sw - 12 - 12) / 4)
                addBtn(pfdX, bY2, bW2, btnRowH, "BASE +1", {0, 105, 92}, {255, 255, 255}, function() FlightCore.adjustBaseThrottle(1.0) end)
                addBtn(pfdX + bW2 + 4, bY2, bW2, btnRowH, "BASE -1", {55, 71, 79}, {255, 255, 255}, function() FlightCore.adjustBaseThrottle(-1.0) end)
                addBtn(pfdX + (bW2 + 4)*2, bY2, bW2, btnRowH, "RE-SCAN", {21, 101, 192}, {255, 255, 255}, function() FlightCore.scanQuadTurtles(); FlightCore.state.statusMsg="Re-scanned!" end)
                addBtn(pfdX + (bW2 + 4)*3, bY2, bW2, btnRowH, "CALIB", {106, 27, 154}, {255, 255, 255}, function() FlightCore.startCalibration() end)

                local bY3 = bY2 + btnRowH + btnSpacing
                local bW3 = math.floor((sw - 12 - 4) / 2)
                local holdBg = (FlightCore.state.mode == "HOLD_ALT") and {46, 125, 50} or {27, 94, 32}
                addBtn(pfdX, bY3, bW3, btnRowH, "[ HOLD ALTITUDE ]", holdBg, {255, 255, 255}, function() FlightCore.holdAltitude() end)
                local stopBg = (FlightCore.state.mode == "IDLE") and {198, 40, 40} or {183, 28, 28}
                addBtn(pfdX + bW3 + 4, bY3, bW3, btnRowH, "[ STOP / IDLE ]", stopBg, {255, 255, 255}, function() FlightCore.stopEngines() end)
            else
                -- 3x3 ~ 4x5 精簡模式 (<5x5 Compact Mode, 適配 384x384 到 512x640 及小螢幕)
                local colW = math.floor((sw - 12) / 2)
                local pfdX = 4
                local ecamX = pfdX + colW + 4

                if colW >= 85 then
                    -- 寬度充足精簡版 (4x4, 5x4): 圓形高度錶 + 2x2 四象限進度條
                    gpu.fillRect(dispId, pfdX, instY, colW, instH, 20, 26, 38)
                    gpu.drawText(dispId, "ALT/ATT", pfdX + 4, instY + 3, 170, 210, 230, "Arial", 9, "bold")

                    local gR = math.max(8, math.min(22, math.floor(colW * 0.18), math.floor(instH * 0.22)))
                    local gCX = pfdX + math.max(gR + 4, math.floor(colW * 0.24))
                    local gCY = instY + math.floor(instH * 0.54)
                    gpu.drawCircle(dispId, gCX, gCY, gR, 80, 170, 240, true)
                    local altStr = string.format("%.0f", currAlt)
                    gpu.drawText(dispId, altStr, gCX - math.floor(#altStr * 4), gCY - 4, 255, 255, 255, "Arial", 9, "bold")

                    local textX = gCX + gR + 6
                    local rowSpacing = math.max(8, math.min(18, math.floor((instH - 18) / 4)))
                    gpu.drawText(dispId, string.format("T:%.0f", FlightCore.state.targetAlt), textX, instY + 10, 80, 230, 255, "Arial", 9, "bold")
                    gpu.drawText(dispId, string.format("V:%+.1f", currVspeed), textX, instY + 10 + rowSpacing, 255, 205, 75, "Arial", 9, "bold")
                    local mCol = (FlightCore.state.mode == "HOLD_ALT") and {80, 255, 120} or {255, 80, 80}
                    gpu.drawText(dispId, string.format("M:%s", FlightCore.state.mode:sub(1,4)), textX, instY + 10 + rowSpacing * 2, mCol[1], mCol[2], mCol[3], "Arial", 9, "bold")
                    gpu.drawText(dispId, string.format("P:%+.0f", currPitch), textX, instY + 10 + rowSpacing * 3, 180, 220, 255, "Arial", 8, "plain")

                    -- 右側: 2x2 Mini Quad Engines
                    gpu.fillRect(dispId, ecamX, instY, colW, instH, 18, 24, 34)
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

                        gpu.fillRect(dispId, sx, sy, subW, subH, 24, 32, 46)
                        gpu.drawText(dispId, string.format("[%s] %dE", ms.slot, qH.total), sx + 2, sy + 2, 200, 230, 255, "Arial", 8, "bold")

                        -- 動態微型進度條 (帶清晰邊框)
                        local barY = sy + math.floor(subH * 0.40)
                        local barW = math.max(4, subW - 6)
                        local barH = math.max(4, math.min(10, math.floor(subH * 0.18)))
                        gpu.fillRect(dispId, sx + 3, barY, barW, barH, 10, 16, 24)
                        local fillW = math.floor(barW * math.min(1.0, math.max(0.0, val / 15.0)))
                        local barCol = (val > 12) and {255, 69, 0} or ((val > 8) and {255, 215, 0} or {50, 205, 50})
                        if fillW > 0 then gpu.fillRect(dispId, sx + 3, barY, fillW, barH, barCol[1], barCol[2], barCol[3]) end

                        -- 數值指示
                        local btmLineY = sy + subH - 9
                        gpu.drawText(dispId, string.format("%4.1f", val), sx + 2, btmLineY, 70, 255, 120, "Arial", 8, "plain")
                        local pwmStr = string.format("P:%d", FlightCore.engineOutputs[ms.slot] or 0)
                        gpu.drawText(dispId, pwmStr, sx + subW - math.floor(#pwmStr * 5) - 2, btmLineY, 150, 190, 225, "Arial", 8, "plain")
                    end
                else
                    -- 極致窄螢幕 (3x3 螢幕, colW < 85px): 乾淨直向數據流，零碰撞
                    -- 左側: PFD 飛行姿態垂直數據堆疊
                    gpu.fillRect(dispId, pfdX, instY, colW, instH, 20, 26, 38)
                    gpu.drawText(dispId, "ALT/ATT", pfdX + 3, instY + 2, 170, 210, 230, "Arial", 8, "bold")

                    local rowSp = math.max(8, math.min(12, math.floor((instH - 14) / 4)))
                    local yBase = instY + 13
                    gpu.drawText(dispId, string.format("ALT:%4.0f", currAlt), pfdX + 3, yBase, 255, 255, 255, "Arial", 8, "bold")
                    gpu.drawText(dispId, string.format("TGT:%4.0f", FlightCore.state.targetAlt), pfdX + 3, yBase + rowSp, 80, 230, 255, "Arial", 8, "bold")
                    gpu.drawText(dispId, string.format("V.S:%+4.1f", currVspeed), pfdX + 3, yBase + rowSp * 2, 255, 205, 75, "Arial", 8, "bold")
                    local mCol = (FlightCore.state.mode == "HOLD_ALT") and {80, 255, 120} or {255, 80, 80}
                    gpu.drawText(dispId, string.format("MOD:%s", FlightCore.state.mode:sub(1, 4)), pfdX + 3, yBase + rowSp * 3, mCol[1], mCol[2], mCol[3], "Arial", 8, "bold")

                    -- 右側: ECAM 四象限發動機直向數據堆疊 (FL, FR, BL, BR 逐行排布)
                    gpu.fillRect(dispId, ecamX, instY, colW, instH, 18, 24, 34)
                    gpu.drawText(dispId, "ECAM 4Q", ecamX + 3, instY + 2, 170, 210, 230, "Arial", 8, "bold")

                    local qSlots = {"FL", "FR", "BL", "BR"}
                    for idx, slot in ipairs(qSlots) do
                        local qH = FlightCore.getQuadHealth(slot)
                        local val = FlightCore.virtualOutputs[slot]
                        local sig = FlightCore.engineOutputs[slot] or 0
                        local yPos = yBase + (idx - 1) * rowSp
                        local colVal = (val > 12) and {255, 69, 0} or ((val > 8) and {255, 215, 0} or {80, 255, 120})
                        if qH.online == 0 then colVal = {255, 80, 80} end
                        gpu.drawText(dispId, string.format("%s%4.1f P%d", slot, val, sig), ecamX + 3, yPos, colVal[1], colVal[2], colVal[3], "Arial", 8, "plain")
                    end
                end

                -- 底部狀態 (包含導航簡報)
                gpu.fillRect(dispId, pfdX, statY, sw - 8, statH, 30, 36, 50)
                local navShort = ""
                if FlightCore.nav.x then
                    navShort = string.format("[%s] X:%.0f Z:%.0f H:%03d* | ", FlightCore.nav.source, FlightCore.nav.x, FlightCore.nav.z, math.floor(FlightCore.nav.yaw))
                else
                    navShort = string.format("H:%03d* | ", math.floor(FlightCore.nav.yaw))
                end
                local maxStatChars = math.max(6, math.floor((sw - 20) / 6) - #navShort)
                gpu.drawText(dispId, navShort .. FlightCore.state.statusMsg:sub(1, maxStatChars), pfdX + 4, statY + 3, 80, 230, 255, "Arial", 8, "bold")

                -- 底部按鈕 (2 排按鈕，嚴格鎖定高度防重疊)
                local bW4 = math.floor((sw - 8 - 9) / 4)
                addBtn(pfdX, btnAreaY, bW4, btnRowH, "+10", {27, 94, 32}, {255, 255, 255}, function() FlightCore.adjustTargetAlt(10) end)
                addBtn(pfdX + bW4 + 3, btnAreaY, bW4, btnRowH, "-10", {198, 40, 40}, {255, 255, 255}, function() FlightCore.adjustTargetAlt(-10) end)
                addBtn(pfdX + (bW4 + 3)*2, btnAreaY, bW4, btnRowH, "B+", {0, 105, 92}, {255, 255, 255}, function() FlightCore.adjustBaseThrottle(1.0) end)
                addBtn(pfdX + (bW4 + 3)*3, btnAreaY, bW4, btnRowH, "B-", {55, 71, 79}, {255, 255, 255}, function() FlightCore.adjustBaseThrottle(-1.0) end)

                local bY2 = btnAreaY + btnRowH + btnSpacing
                local bW2 = math.floor((sw - 8 - 3) / 2)
                local holdBg = (FlightCore.state.mode == "HOLD_ALT") and {46, 125, 50} or {27, 94, 32}
                addBtn(pfdX, bY2, bW2, btnRowH, "HOLD", holdBg, {255, 255, 255}, function() FlightCore.holdAltitude() end)
                local stopBg = (FlightCore.state.mode == "IDLE") and {198, 40, 40} or {183, 28, 28}
                addBtn(pfdX + bW2 + 3, bY2, bW2, btnRowH, "STOP", stopBg, {255, 255, 255}, function() FlightCore.stopEngines() end)
            end
end
