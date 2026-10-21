-- directgpu ECAM view renderer.
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
                gpu.fillRect(dispId, qx, qy, quadW, quadH, 18, 24, 34)

                local qH = FlightCore.getQuadHealth(q.slot)
                local online = qH.online > 0

                if isFull and (quadW >= 110) then
                    local titleCol = online and {180, 230, 255} or {255, 120, 120}
                    local titleStr = (quadW < 160) and string.format("[%s] QUAD", q.slot) or q.title
                    gpu.drawText(dispId, titleStr, qx + 6, qy + 4, titleCol[1], titleCol[2], titleCol[3], "Arial", 10, "bold")

                    local engCountStr = (quadW < 130) and string.format("%dE", qH.total) or string.format("ENG:%d", qH.total)
                    gpu.drawText(dispId, engCountStr, qx + quadW - math.floor(#engCountStr * 6) - 6, qy + 4, 150, 190, 230, "Arial", 9, "plain")

                    local dialR = math.min(math.floor(quadW * 0.28), math.floor(quadH * 0.28))
                    local engCenterX = qx + math.floor(quadW / 2)
                    local engCenterY = qy + math.floor(quadH * 0.52)
                    self:drawA350Dial(scr, engCenterX, engCenterY, dialR, FlightCore.virtualOutputs[q.slot], 15.0, q.slot, FlightCore.engineOutputs[q.slot], q.slot)

                    local statText = string.format("ACT:%d/%d", qH.online, qH.total)
                    if qH.total == 0 then statText = "NO ENG" end
                    local statCol = (qH.online > 0) and {80, 255, 120} or {255, 80, 80}
                    gpu.drawText(dispId, statText, qx + 6, qy + quadH - 12, statCol[1], statCol[2], statCol[3], "Arial", 9, "bold")
                else
                    -- 緊湊/直立螢幕版
                    local qTitle = string.format("[%s] %dE", q.slot, qH.total)
                    local titleCol = online and {180, 230, 255} or {255, 120, 120}
                    gpu.drawText(dispId, qTitle, qx + 3, qy + 3, titleCol[1], titleCol[2], titleCol[3], "Arial", 8, "bold")

                    local barY = qy + math.floor(quadH * 0.40)
                    local barW = math.max(4, quadW - 6)
                    local barH = math.max(4, math.min(10, math.floor(quadH * 0.18)))
                    gpu.fillRect(dispId, qx + 3, barY, barW, barH, 10, 16, 24)
                    local fillW = math.floor(barW * math.min(1.0, math.max(0.0, FlightCore.virtualOutputs[q.slot] / 15.0)))
                    local barCol = (FlightCore.virtualOutputs[q.slot] > 12) and {255, 69, 0} or ((FlightCore.virtualOutputs[q.slot] > 8) and {255, 215, 0} or {50, 205, 50})
                    if fillW > 0 then gpu.fillRect(dispId, qx + 3, barY, fillW, barH, barCol[1], barCol[2], barCol[3]) end

                    local btmLineY = qy + quadH - 9
                    gpu.drawText(dispId, string.format("%.1f", FlightCore.virtualOutputs[q.slot]), qx + 3, btmLineY, 70, 255, 120, "Arial", 8, "bold")
                    local actStr = (quadW < 65) and string.format("P:%d", FlightCore.engineOutputs[q.slot] or 0) or string.format("P:%d %d/%d", FlightCore.engineOutputs[q.slot] or 0, qH.online, qH.total)
                    local actCol = (qH.online > 0) and {80, 255, 120} or {255, 80, 80}
                    gpu.drawText(dispId, actStr, qx + quadW - math.floor(#actStr * 5) - 3, btmLineY, actCol[1], actCol[2], actCol[3], "Arial", 8, "plain")
                end
            end

            local statY = topY + areaH + 2
            gpu.fillRect(dispId, 4, statY, sw - 8, btmH, 25, 32, 45)
            if isFull then
                gpu.drawText(dispId, string.format("BASE: %4.2f/15 | BALANCED | STATUS: %s", FlightCore.state.baseThrottle, FlightCore.state.statusMsg:sub(1, 26)), 8, statY + 4, 80, 230, 255, "Arial", 9, "bold")
            else
                local statChars = math.max(6, math.floor((sw - 60) / 6))
                gpu.drawText(dispId, string.format("B:%4.2f | %s", FlightCore.state.baseThrottle, FlightCore.state.statusMsg:sub(1, statChars)), 6, statY + 3, 80, 230, 255, "Arial", 8, "bold")
            end
end
