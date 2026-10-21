-- tom CTRL view renderer.
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
                -- 大型螢幕 >=5x5 專業飛控管理中心 (Full Avionics Flight Management System)
                local cardW = math.floor((sw - 16) / 2)
                local c1X = 6
                local c2X = c1X + cardW + 4

                -- 卡片 1: 飛行高度與升降剖面
                sFR(c1X, instY, cardW, instH, 0x141E2B)
                sR(c1X, instY, cardW, instH, 0x284864)
                sTxt(c1X + 6, instY + 4, "ALTITUDE PROFILE", 0x8CA0B4, 1)
                sTxt(c1X + 8, instY + 18, string.format("%.1fm", currAlt), 0xFFFFFF, 2)

                local textOffY = instY + 36
                local rowSp = math.max(12, math.floor((instH - 42) / 3))
                local tgtStr = (cardW < 170) and string.format("TGT:%.0fm (DIFF:%+.0fm)", FlightCore.state.targetAlt, FlightCore.state.targetAlt - currAlt) or string.format("TARGET: %5.1fm (DIFF:%+5.1fm)", FlightCore.state.targetAlt, FlightCore.state.targetAlt - currAlt)
                sTxt(c1X + 8, textOffY, tgtStr, 0x50E6FF, 1)
                local vsCol = (math.abs(currVspeed) < 0.2) and 0x50FF78 or 0xFFCD4B
                sTxt(c1X + 8, textOffY + rowSp, string.format("V.SPEED: %+5.2f m/s", currVspeed), vsCol, 1)
                local mCol = (FlightCore.state.mode == "HOLD_ALT") and 0x50FF78 or ((FlightCore.state.mode == "CALIBRATING") and 0xFFCD4B or 0xFF5050)
                sTxt(c1X + 8, textOffY + rowSp * 2, string.format("MODE: [%s]", FlightCore.state.mode), mCol, 1)

                -- 卡片 2: 姿態平衡與推力總覽
                sFR(c2X, instY, cardW, instH, 0x141E2B)
                sR(c2X, instY, cardW, instH, 0x284864)
                sTxt(c2X + 6, instY + 4, "ATTITUDE & PROPULSION", 0x8CA0B4, 1)
                local baseStr = (cardW < 170) and string.format("BASE: %4.2f", FlightCore.state.baseThrottle) or string.format("BASE: %4.2f / 15.0", FlightCore.state.baseThrottle)
                sTxt(c2X + 8, instY + 18, baseStr, 0x50FF78, 2)
                sTxt(c2X + 8, textOffY, string.format("GYRO: P:%+4.1f*  R:%+4.1f*", currPitch, currRoll), 0xB4DCFF, 1)

                local quadSummary = (cardW < 170) and string.format("F:%.1f,%.1f B:%.1f,%.1f", FlightCore.virtualOutputs.FL, FlightCore.virtualOutputs.FR, FlightCore.virtualOutputs.BL, FlightCore.virtualOutputs.BR) or string.format("FL:%.1f FR:%.1f BL:%.1f BR:%.1f", FlightCore.virtualOutputs.FL, FlightCore.virtualOutputs.FR, FlightCore.virtualOutputs.BL, FlightCore.virtualOutputs.BR)
                sTxt(c2X + 8, textOffY + rowSp, quadSummary, 0xFFCD4B, 1)
                local sysSummary = (cardW < 170) and "SYS: BALANCED & SYNCED" or "SYSTEM: BALANCED & SYNCED"
                sTxt(c2X + 8, textOffY + rowSp * 2, sysSummary, 0x50E6FF, 1)

                -- 中段狀態列 (包含導航簡報)
                sFR(6, statY, sw - 12, statH, 0x1E2432)
                sR(6, statY, sw - 12, statH, 0x3C4B64)
                local navInfo = ""
                if FlightCore.nav.x then
                    navInfo = string.format("POS:[%s] X:%.0f Y:%.0f Z:%.0f | HDG:%03d* | ", FlightCore.nav.source, FlightCore.nav.x, FlightCore.nav.y or currAlt, FlightCore.nav.z, math.floor(FlightCore.nav.yaw))
                else
                    navInfo = string.format("HDG:%03d* | ", math.floor(FlightCore.nav.yaw))
                end
                sTxt(10, statY + 4, navInfo .. string.format("STATUS: %s", FlightCore.state.statusMsg:sub(1, 35)), 0x50E6FF, 1)

                -- 下半部控制按鈕群 (3 組分類)
                local bW1 = math.floor((sw - 12 - 20) / 6)
                addBtn(6, btnAreaY, bW1, btnRowH, "+50m", 0x145A32, 0xFFFFFF, function() FlightCore.adjustTargetAlt(50) end)
                addBtn(6 + (bW1+4), btnAreaY, bW1, btnRowH, "+10m", 0x1B5E20, 0xFFFFFF, function() FlightCore.adjustTargetAlt(10) end)
                addBtn(6 + (bW1+4)*2, btnAreaY, bW1, btnRowH, "+1m", 0x2E7D32, 0xFFFFFF, function() FlightCore.adjustTargetAlt(1) end)
                addBtn(6 + (bW1+4)*3, btnAreaY, bW1, btnRowH, "-1m", 0xE65100, 0xFFFFFF, function() FlightCore.adjustTargetAlt(-1) end)
                addBtn(6 + (bW1+4)*4, btnAreaY, bW1, btnRowH, "-10m", 0xC62828, 0xFFFFFF, function() FlightCore.adjustTargetAlt(-10) end)
                addBtn(6 + (bW1+4)*5, btnAreaY, bW1, btnRowH, "LOCK", 0x00838F, 0xFFFFFF, function() FlightCore.lockCurrentAlt() end)

                local r2Y = btnAreaY + btnRowH + btnSpacing
                local bW2 = math.floor((sw - 12 - 12) / 4)
                addBtn(6, r2Y, bW2, btnRowH, "BASE +1.0", 0x00695C, 0xFFFFFF, function() FlightCore.adjustBaseThrottle(1.0) end)
                addBtn(6 + (bW2+4), r2Y, bW2, btnRowH, "BASE -1.0", 0x37474F, 0xFFFFFF, function() FlightCore.adjustBaseThrottle(-1.0) end)
                addBtn(6 + (bW2+4)*2, r2Y, bW2, btnRowH, "BASE +0.1", 0x00897B, 0xFFFFFF, function() FlightCore.adjustBaseThrottle(0.1) end)
                addBtn(6 + (bW2+4)*3, r2Y, bW2, btnRowH, "BASE -0.1", 0x455A64, 0xFFFFFF, function() FlightCore.adjustBaseThrottle(-0.1) end)

                local r3Y = r2Y + btnRowH + btnSpacing
                local bW3 = math.floor((sw - 12 - 12) / 4)
                local holdBg = (FlightCore.state.mode == "HOLD_ALT") and 0x2E7D32 or 0x1B5E20
                addBtn(6, r3Y, bW3, btnRowH, "[ HOLD ALT ]", holdBg, 0xFFFFFF, function() FlightCore.holdAltitude() end)
                addBtn(6 + (bW3+4), r3Y, bW3, btnRowH, "[ CALIB HOVER ]", 0x6A1B9A, 0xFFFFFF, function() FlightCore.startCalibration() end)
                addBtn(6 + (bW3+4)*2, r3Y, bW3, btnRowH, "[ RE-SCAN HW ]", 0x1565C0, 0xFFFFFF, function() FlightCore.scanQuadTurtles(); FlightCore.state.statusMsg="Re-scanned!" end)
                local stopBg = (FlightCore.state.mode == "IDLE") and 0xC62828 or 0xB71C1C
                addBtn(6 + (bW3+4)*3, r3Y, bW3, btnRowH, "[ STOP IDLE ]", stopBg, 0xFFFFFF, function() FlightCore.stopEngines() end)
            else
                -- 緊湊模式 (<5x5 Compact Mode)
                local topH = math.max(16, math.min(22, math.floor(instH * 0.28)))
                sFR(6, instY, sw - 12, topH, 0x141C2A)
                sR(6, instY, sw - 12, topH, 0x284864)
                local mCol = (FlightCore.state.mode == "HOLD_ALT") and 0x50FF78 or 0xFF5050
                if sw < 160 then
                    sTxt(8, instY + 3, string.format("[%s] A:%.0f H:%03d*", FlightCore.state.mode:sub(1,4), currAlt, math.floor(FlightCore.nav.yaw)), 0xC8E6FF, 1)
                else
                    sTxt(10, instY + 4, string.format("[%s]", FlightCore.state.mode:sub(1,6)), mCol, 1)
                    local navCompact = FlightCore.nav.x and string.format("X:%.0f Z:%.0f H:%03d*", FlightCore.nav.x, FlightCore.nav.z, math.floor(FlightCore.nav.yaw)) or string.format("ALT:%.0f->%.0f H:%03d*", currAlt, FlightCore.state.targetAlt, math.floor(FlightCore.nav.yaw))
                    sTxt(65, instY + 4, navCompact, 0xC8E6FF, 1)
                end

                local ctrlRowH = math.max(12, math.min(24, math.floor((sh - instY - topH - 12) / 3)))
                local gridY = instY + topH + 4

                -- Row 1: Target Altitude
                local bW1 = math.floor((sw - 12 - 12) / 4)
                addBtn(6, gridY, bW1, ctrlRowH, "+10m", 0x1E642D, 0xFFFFFF, function() FlightCore.adjustTargetAlt(10) end)
                addBtn(6 + (bW1+4), gridY, bW1, ctrlRowH, "+1m", 0x2D8237, 0xFFFFFF, function() FlightCore.adjustTargetAlt(1) end)
                addBtn(6 + (bW1+4)*2, gridY, bW1, ctrlRowH, "-1m", 0xD25F14, 0xFFFFFF, function() FlightCore.adjustTargetAlt(-1) end)
                addBtn(6 + (bW1+4)*3, gridY, bW1, ctrlRowH, "-10m", 0xBE2828, 0xFFFFFF, function() FlightCore.adjustTargetAlt(-10) end)

                -- Row 2: Base Throttle
                local r2Y = gridY + ctrlRowH + 3
                local bW2 = math.floor((sw - 12 - 12) / 4)
                addBtn(6, r2Y, bW2, ctrlRowH, "B +1", 0x006E5F, 0xFFFFFF, function() FlightCore.adjustBaseThrottle(1.0) end)
                addBtn(6 + (bW2+4), r2Y, bW2, ctrlRowH, "B -1", 0x374650, 0xFFFFFF, function() FlightCore.adjustBaseThrottle(-1.0) end)
                addBtn(6 + (bW2+4)*2, r2Y, bW2, ctrlRowH, "B +.1", 0x14786E, 0xFFFFFF, function() FlightCore.adjustBaseThrottle(0.1) end)
                addBtn(6 + (bW2+4)*3, r2Y, bW2, ctrlRowH, "B -.1", 0x46505A, 0xFFFFFF, function() FlightCore.adjustBaseThrottle(-0.1) end)

                -- Row 3: Flight Ops
                local r3Y = r2Y + ctrlRowH + 3
                local bW3 = math.floor((sw - 12 - 12) / 4)
                local holdBg = (FlightCore.state.mode == "HOLD_ALT") and 0x2D8C3C or 0x1E5A28
                addBtn(6, r3Y, bW3, ctrlRowH, "HOLD", holdBg, 0xFFFFFF, function() FlightCore.holdAltitude() end)
                addBtn(6 + (bW3+4), r3Y, bW3, ctrlRowH, "CALIB", 0x6E1E9B, 0xFFFFFF, function() FlightCore.startCalibration() end)
                addBtn(6 + (bW3+4)*2, r3Y, bW3, ctrlRowH, "RE-SCAN", 0x1964BE, 0xFFFFFF, function() FlightCore.scanQuadTurtles(); FlightCore.state.statusMsg="Re-scanned!" end)
                local stopBg = (FlightCore.state.mode == "IDLE") and 0xC82D2D or 0xA01E1E
                addBtn(6 + (bW3+4)*3, r3Y, bW3, ctrlRowH, "STOP", stopBg, 0xFFFFFF, function() FlightCore.stopEngines() end)
            end
end
