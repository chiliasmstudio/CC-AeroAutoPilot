-- directgpu CTRL view renderer.
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
                -- 大型螢幕 >=5x5 專業飛控管理中心 (Full Avionics Flight Management System)
                local cardW = math.floor((sw - 16) / 2)
                local c1X = 6
                local c2X = c1X + cardW + 4

                -- 卡片 1: 飛行高度與升降剖面
                gpu.fillRect(dispId, c1X, instY, cardW, instH, 20, 30, 43)
                gpu.drawText(dispId, "ALTITUDE PROFILE", c1X + 6, instY + 4, 140, 160, 180, "Arial", 9, "bold")

                local altStr = string.format("%.1fm", currAlt)
                gpu.drawText(dispId, altStr, c1X + 8, instY + 18, 255, 255, 255, "Arial", 14, "bold")

                local textOffY = instY + 36
                local rowSp = math.max(12, math.floor((instH - 42) / 3))
                local tgtStr = (cardW < 170) and string.format("TGT:%.0fm (DIFF:%+.0fm)", FlightCore.state.targetAlt, FlightCore.state.targetAlt - currAlt) or string.format("TARGET: %5.1fm (DIFF:%+5.1fm)", FlightCore.state.targetAlt, FlightCore.state.targetAlt - currAlt)
                gpu.drawText(dispId, tgtStr, c1X + 8, textOffY, 80, 230, 255, "Arial", 9, "bold")
                local vsCol = (math.abs(currVspeed) < 0.2) and {80, 255, 120} or {255, 205, 75}
                gpu.drawText(dispId, string.format("V.SPEED: %+5.2f m/s", currVspeed), c1X + 8, textOffY + rowSp, vsCol[1], vsCol[2], vsCol[3], "Arial", 9, "bold")
                local mCol = (FlightCore.state.mode == "HOLD_ALT") and {80, 255, 120} or ((FlightCore.state.mode == "CALIBRATING") and {255, 205, 75} or {255, 80, 80})
                gpu.drawText(dispId, string.format("MODE: [%s]", FlightCore.state.mode), c1X + 8, textOffY + rowSp * 2, mCol[1], mCol[2], mCol[3], "Arial", 9, "bold")

                -- 卡片 2: 姿態平衡與推力總覽
                gpu.fillRect(dispId, c2X, instY, cardW, instH, 20, 30, 43)
                gpu.drawText(dispId, "ATTITUDE & PROPULSION", c2X + 6, instY + 4, 140, 160, 180, "Arial", 9, "bold")

                local baseStr = (cardW < 170) and string.format("BASE: %4.2f", FlightCore.state.baseThrottle) or string.format("BASE: %4.2f / 15.0", FlightCore.state.baseThrottle)
                gpu.drawText(dispId, baseStr, c2X + 8, instY + 18, 80, 255, 120, "Arial", 14, "bold")
                gpu.drawText(dispId, string.format("GYRO: P:%+4.1f*  R:%+4.1f*", currPitch, currRoll), c2X + 8, textOffY, 180, 220, 255, "Arial", 9, "bold")

                local quadSummary = (cardW < 170) and string.format("F:%.1f,%.1f B:%.1f,%.1f", FlightCore.virtualOutputs.FL, FlightCore.virtualOutputs.FR, FlightCore.virtualOutputs.BL, FlightCore.virtualOutputs.BR) or string.format("FL:%.1f FR:%.1f BL:%.1f BR:%.1f", FlightCore.virtualOutputs.FL, FlightCore.virtualOutputs.FR, FlightCore.virtualOutputs.BL, FlightCore.virtualOutputs.BR)
                gpu.drawText(dispId, quadSummary, c2X + 8, textOffY + rowSp, 255, 205, 75, "Arial", 9, "bold")
                local sysSummary = (cardW < 170) and "SYS: BALANCED & SYNCED" or "SYSTEM: BALANCED & SYNCED"
                gpu.drawText(dispId, sysSummary, c2X + 8, textOffY + rowSp * 2, 80, 230, 255, "Arial", 9, "bold")

                -- 中段狀態列 (包含導航簡報)
                gpu.fillRect(dispId, 6, statY, sw - 12, statH, 30, 36, 50)
                local navInfo = ""
                if FlightCore.nav.x then
                    navInfo = string.format("POS:[%s] X:%.0f Y:%.0f Z:%.0f | HDG:%03d* | ", FlightCore.nav.source, FlightCore.nav.x, FlightCore.nav.y or currAlt, FlightCore.nav.z, math.floor(FlightCore.nav.yaw))
                else
                    navInfo = string.format("HDG:%03d* | ", math.floor(FlightCore.nav.yaw))
                end
                gpu.drawText(dispId, navInfo .. string.format("STATUS: %s", FlightCore.state.statusMsg:sub(1, 35)), 10, statY + 4, 80, 230, 255, "Arial", 9, "bold")

                -- 下半部控制按鈕群 (3 組分類)
                local bW1 = math.floor((sw - 12 - 20) / 6)
                addBtn(6, btnAreaY, bW1, btnRowH, "+50m", {20, 90, 50}, {255, 255, 255}, function() FlightCore.adjustTargetAlt(50) end)
                addBtn(6 + (bW1+4), btnAreaY, bW1, btnRowH, "+10m", {27, 94, 32}, {255, 255, 255}, function() FlightCore.adjustTargetAlt(10) end)
                addBtn(6 + (bW1+4)*2, btnAreaY, bW1, btnRowH, "+1m", {46, 125, 50}, {255, 255, 255}, function() FlightCore.adjustTargetAlt(1) end)
                addBtn(6 + (bW1+4)*3, btnAreaY, bW1, btnRowH, "-1m", {230, 81, 0}, {255, 255, 255}, function() FlightCore.adjustTargetAlt(-1) end)
                addBtn(6 + (bW1+4)*4, btnAreaY, bW1, btnRowH, "-10m", {198, 40, 40}, {255, 255, 255}, function() FlightCore.adjustTargetAlt(-10) end)
                addBtn(6 + (bW1+4)*5, btnAreaY, bW1, btnRowH, "LOCK", {0, 131, 143}, {255, 255, 255}, function() FlightCore.lockCurrentAlt() end)

                local r2Y = btnAreaY + btnRowH + btnSpacing
                local bW2 = math.floor((sw - 12 - 12) / 4)
                addBtn(6, r2Y, bW2, btnRowH, "BASE +1.0", {0, 105, 92}, {255, 255, 255}, function() FlightCore.adjustBaseThrottle(1.0) end)
                addBtn(6 + (bW2+4), r2Y, bW2, btnRowH, "BASE -1.0", {55, 71, 79}, {255, 255, 255}, function() FlightCore.adjustBaseThrottle(-1.0) end)
                addBtn(6 + (bW2+4)*2, r2Y, bW2, btnRowH, "BASE +0.1", {0, 137, 123}, {255, 255, 255}, function() FlightCore.adjustBaseThrottle(0.1) end)
                addBtn(6 + (bW2+4)*3, r2Y, bW2, btnRowH, "BASE -0.1", {69, 90, 100}, {255, 255, 255}, function() FlightCore.adjustBaseThrottle(-0.1) end)

                local r3Y = r2Y + btnRowH + btnSpacing
                local bW3 = math.floor((sw - 12 - 12) / 4)
                local holdBg = (FlightCore.state.mode == "HOLD_ALT") and {46, 125, 50} or {27, 94, 32}
                addBtn(6, r3Y, bW3, btnRowH, "[ HOLD ALT ]", holdBg, {255, 255, 255}, function() FlightCore.holdAltitude() end)
                addBtn(6 + (bW3+4), r3Y, bW3, btnRowH, "[ CALIB HOVER ]", {106, 27, 154}, {255, 255, 255}, function() FlightCore.startCalibration() end)
                addBtn(6 + (bW3+4)*2, r3Y, bW3, btnRowH, "[ RE-SCAN HW ]", {21, 101, 192}, {255, 255, 255}, function() FlightCore.scanQuadTurtles(); FlightCore.state.statusMsg="Re-scanned!" end)
                local stopBg = (FlightCore.state.mode == "IDLE") and {198, 40, 40} or {183, 28, 28}
                addBtn(6 + (bW3+4)*3, r3Y, bW3, btnRowH, "[ STOP IDLE ]", stopBg, {255, 255, 255}, function() FlightCore.stopEngines() end)
            else
                -- 緊湊模式 (<5x5 Compact Mode)
                local topH = math.max(16, math.min(22, math.floor(instH * 0.28)))
                gpu.fillRect(dispId, 6, instY, sw - 12, topH, 20, 28, 42)
                local mCol = (FlightCore.state.mode == "HOLD_ALT") and {80, 255, 120} or {255, 80, 80}
                if sw < 160 then
                    gpu.drawText(dispId, string.format("[%s] A:%.0f H:%03d*", FlightCore.state.mode:sub(1,4), currAlt, math.floor(FlightCore.nav.yaw)), 8, instY + 3, 200, 230, 255, "Arial", 8, "plain")
                else
                    gpu.drawText(dispId, string.format("[%s]", FlightCore.state.mode:sub(1,6)), 10, instY + 4, mCol[1], mCol[2], mCol[3], "Arial", 9, "bold")
                    local navCompact = FlightCore.nav.x and string.format("X:%.0f Z:%.0f H:%03d*", FlightCore.nav.x, FlightCore.nav.z, math.floor(FlightCore.nav.yaw)) or string.format("ALT:%.0f->%.0f H:%03d*", currAlt, FlightCore.state.targetAlt, math.floor(FlightCore.nav.yaw))
                    gpu.drawText(dispId, navCompact, 65, instY + 4, 200, 230, 255, "Arial", 9, "plain")
                end

                local ctrlRowH = math.max(12, math.min(24, math.floor((sh - instY - topH - 12) / 3)))
                local gridY = instY + topH + 4

                -- Row 1: Target Altitude
                local bW1 = math.floor((sw - 12 - 12) / 4)
                addBtn(6, gridY, bW1, ctrlRowH, "+10m", {30, 100, 45}, {240, 255, 240}, function() FlightCore.adjustTargetAlt(10) end)
                addBtn(6 + (bW1+4), gridY, bW1, ctrlRowH, "+1m", {45, 130, 55}, {240, 255, 240}, function() FlightCore.adjustTargetAlt(1) end)
                addBtn(6 + (bW1+4)*2, gridY, bW1, ctrlRowH, "-1m", {210, 95, 20}, {255, 245, 230}, function() FlightCore.adjustTargetAlt(-1) end)
                addBtn(6 + (bW1+4)*3, gridY, bW1, ctrlRowH, "-10m", {190, 40, 40}, {255, 235, 235}, function() FlightCore.adjustTargetAlt(-10) end)

                -- Row 2: Base Throttle
                local r2Y = gridY + ctrlRowH + 3
                local bW2 = math.floor((sw - 12 - 12) / 4)
                addBtn(6, r2Y, bW2, ctrlRowH, "B +1", {0, 110, 95}, {230, 250, 245}, function() FlightCore.adjustBaseThrottle(1.0) end)
                addBtn(6 + (bW2+4), r2Y, bW2, ctrlRowH, "B -1", {55, 70, 80}, {235, 240, 245}, function() FlightCore.adjustBaseThrottle(-1.0) end)
                addBtn(6 + (bW2+4)*2, r2Y, bW2, ctrlRowH, "B +.1", {20, 120, 110}, {230, 255, 250}, function() FlightCore.adjustBaseThrottle(0.1) end)
                addBtn(6 + (bW2+4)*3, r2Y, bW2, ctrlRowH, "B -.1", {70, 80, 90}, {235, 240, 245}, function() FlightCore.adjustBaseThrottle(-0.1) end)

                -- Row 3: Flight Ops
                local r3Y = r2Y + ctrlRowH + 3
                local bW3 = math.floor((sw - 12 - 12) / 4)
                local holdBg = (FlightCore.state.mode == "HOLD_ALT") and {45, 140, 60} or {30, 90, 40}
                addBtn(6, r3Y, bW3, ctrlRowH, "HOLD", holdBg, {255, 255, 255}, function() FlightCore.holdAltitude() end)
                addBtn(6 + (bW3+4), r3Y, bW3, ctrlRowH, "CALIB", {110, 30, 155}, {250, 230, 255}, function() FlightCore.startCalibration() end)
                addBtn(6 + (bW3+4)*2, r3Y, bW3, ctrlRowH, "RE-SCAN", {25, 100, 190}, {230, 245, 255}, function() FlightCore.scanQuadTurtles(); FlightCore.state.statusMsg="Re-scanned!" end)
                local stopBg = (FlightCore.state.mode == "IDLE") and {200, 45, 45} or {160, 30, 30}
                addBtn(6 + (bW3+4)*3, r3Y, bW3, ctrlRowH, "STOP", stopBg, {255, 255, 255}, function() FlightCore.stopEngines() end)
            end
end
