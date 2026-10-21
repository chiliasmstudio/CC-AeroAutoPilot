-- tom ECAM view renderer.
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
            local topY = headerH + 3
            local btmH = isFull and 22 or 14
            local areaH = sh - topY - btmH - 4
            local quadW = math.floor((sw - 12) / 2)
            local quadH = math.floor((areaH - 3) / 2)

            local quadDefs = {
                {slot="FL", col=1, row=1, title="[FL] FRONT-LEFT"},
                {slot="FR", col=2, row=1, title="[FR] FRONT-RIGHT"},
                {slot="BL", col=1, row=2, title="[BL] BACK-LEFT"},
                {slot="BR", col=2, row=2, title="[BR] BACK-RIGHT"}
            }

            for _, q in ipairs(quadDefs) do
                local qx = 4 + (q.col - 1) * (quadW + 4)
                local qy = topY + (q.row - 1) * (quadH + 3)
                sFR(qx, qy, quadW, quadH, 0x121822)
                sR(qx, qy, quadW, quadH, 0x283850)

                local qH = FlightCore.getQuadHealth(q.slot)
                local online = qH.online > 0
                local val = FlightCore.virtualOutputs[q.slot]

                if isFull and (quadW >= 110) then
                    local titleCol = online and 0xB4E6FF or 0xFF7878
                    local titleStr = (quadW < 160) and string.format("[%s] QUAD", q.slot) or q.title
                    sTxt(qx + 6, qy + 4, titleStr, titleCol, 1)

                    local engCountStr = (quadW < 130) and string.format("%dE", qH.total) or string.format("ENG:%d", qH.total)
                    sTxt(qx + quadW - #engCountStr * 6 - 6, qy + 4, engCountStr, 0x96BEE6, 1)

                    -- 圓形刻度盤 (Tom's GPU - Airbus A350/A320 ECAM Style)
                    local dialR = math.min(math.floor(quadW * 0.28), math.floor(quadH * 0.28))
                    local engCenterX = qx + math.floor(quadW / 2)
                    local engCenterY = qy + math.floor(quadH * 0.46)

                    -- 1. 主圓弧軌跡 (210° 底左 -> 90° 頂部 -> -30° 底右，頂部拱起，底部開放)
                    sArc(engCenterX, engCenterY, dialR, 210, -30, 8, 0x647890)
                    -- 紅色超限警告區間 (10° -> -30°)
                    sArc(engCenterX, engCenterY, dialR, 10, -30, 5, 0xFF3737)

                    -- 2. 刻度標記線 (210°=0, 90°=7.5, -30°=15)
                    for _, deg in ipairs({210, 90, -30}) do
                        local rad = math.rad(deg)
                        local tx1 = engCenterX + math.floor((dialR - 3) * math.cos(rad) + 0.5)
                        local ty1 = engCenterY - math.floor((dialR - 3) * math.sin(rad) + 0.5)
                        local tx2 = engCenterX + math.floor((dialR + 3) * math.cos(rad) + 0.5)
                        local ty2 = engCenterY - math.floor((dialR + 3) * math.sin(rad) + 0.5)
                        sLS(tx1, ty1, tx2, ty2, 0x96B4DC)
                    end

                    -- 3. 指針 (0 時指向 210° 底左，隨推力順時針旋轉至 -30° 底右)
                    local ratio = math.min(1.0, math.max(0.0, val / 15.0))
                    local needleDeg = 210 - ratio * 240
                    local nRad = math.rad(needleDeg)
                    local px = engCenterX + math.floor((dialR - 2) * math.cos(nRad) + 0.5)
                    local py = engCenterY - math.floor((dialR - 2) * math.sin(nRad) + 0.5)
                    sLS(engCenterX, engCenterY, px, py, 0x50FF78)

                    -- 中心軸心點
                    sFR(engCenterX - 1, engCenterY - 1, 3, 3, 0xC8E6FF)

                    -- 4. 底部數位指示方框 (Airbus ECAM 數位讀數框，位於指針下方不遮擋)
                    local boxW = math.max(28, math.floor(dialR * 1.3))
                    local boxH = 11
                    local boxX = engCenterX - math.floor(boxW / 2)
                    local boxY = engCenterY + math.floor(dialR * 0.35)
                    sFR(boxX, boxY, boxW, boxH, 0x0C121C)
                    sR(boxX, boxY, boxW, boxH, 0x384860)

                    local thrValStr = string.format("%4.1f", val)
                    sTxt(boxX + math.floor((boxW - #thrValStr * 6) / 2) + 1, boxY + 2, thrValStr, 0x50FF78, 1)

                    local statText = string.format("ACT:%d/%d", qH.online, qH.total)
                    if qH.total == 0 then statText = "NO ENG" end
                    local statCol = (qH.online > 0) and 0x50FF78 or 0xFF5050
                    sTxt(qx + 6, qy + quadH - 12, statText, statCol, 1)

                    local pwmText = string.format("PWM:%d", FlightCore.engineOutputs[q.slot] or 0)
                    sTxt(qx + quadW - #pwmText * 6 - 6, qy + quadH - 12, pwmText, 0x96BEE1, 1)
                else
                    -- 緊湊/直立螢幕版
                    local qTitle = string.format("[%s] %dE", q.slot, qH.total)
                    local titleCol = online and 0xB4E6FF or 0xFF7878
                    sTxt(qx + 3, qy + 3, qTitle, titleCol, 1)

                    local barY = qy + math.floor(quadH * 0.40)
                    local barW = math.max(4, quadW - 6)
                    local barH = math.max(4, math.min(10, math.floor(quadH * 0.18)))
                    sFR(qx + 3, barY, barW, barH, 0x0A1018)
                    local fillW = math.floor(barW * math.min(1.0, math.max(0.0, val / 15.0)))
                    local barCol = (val > 12) and 0xFF4500 or ((val > 8) and 0xFFD700 or 0x32CD32)
                    if fillW > 0 then sFR(qx + 3, barY, fillW, barH, barCol) end
                    sR(qx + 3, barY, barW, barH, 0x384860)

                    local btmLineY = qy + quadH - 9
                    sTxt(qx + 3, btmLineY, string.format("%.1f", val), 0x46FF78, 1)
                    local actStr = (quadW < 65) and string.format("P:%d", FlightCore.engineOutputs[q.slot] or 0) or string.format("P:%d %d/%d", FlightCore.engineOutputs[q.slot] or 0, qH.online, qH.total)
                    local actCol = (qH.online > 0) and 0x50FF78 or 0xFF5050
                    sTxt(qx + quadW - #actStr * 6 - 3, btmLineY, actStr, actCol, 1)
                end
            end

            local statY = topY + areaH + 2
            sFR(4, statY, sw - 8, btmH, 0x19202D)
            sR(4, statY, sw - 8, btmH, 0x3C4B64)
            if isFull then
                local statChars = math.max(6, math.floor((sw - 160) / 6))
                sTxt(8, statY + 4, string.format("BASE: %4.2f/15 | BALANCED | STATUS: %s", FlightCore.state.baseThrottle, FlightCore.state.statusMsg:sub(1, statChars)), 0x50E6FF, 1)
            else
                local statChars = math.max(6, math.floor((sw - 60) / 6))
                sTxt(6, statY + 3, string.format("B:%4.2f | %s", FlightCore.state.baseThrottle, FlightCore.state.statusMsg:sub(1, statChars)), 0x50E6FF, 1)
            end
end
