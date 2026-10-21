-- normal ECAM view renderer.
-- Shared drawing helpers and layout values are supplied by the parent driver.
return function(self, scr, ctx)
    local FlightCore = ctx.FlightCore
    local addBtn = ctx.addBtn
    local h = ctx.h
    local isFull = ctx.isFull
    local w = ctx.w
            local gridY = 3
            local quadW = math.floor((w - 3) / 2)
            local quadH = math.max(3, math.floor((h - gridY - 2) / 2))

            local quadDefs = {
                {slot="FL", col=1, row=1, title="[FL] FRONT-LEFT"},
                {slot="FR", col=2, row=1, title="[FR] FRONT-RIGHT"},
                {slot="BL", col=1, row=2, title="[BL] BACK-LEFT"},
                {slot="BR", col=2, row=2, title="[BR] BACK-RIGHT"}
            }

            for _, q in ipairs(quadDefs) do
                local qx = 2 + (q.col - 1) * (quadW + 1)
                local qy = gridY + (q.row - 1) * (quadH + 1)
                local qH = FlightCore.getQuadHealth(q.slot)
                local val = FlightCore.virtualOutputs[q.slot]
                local sig = FlightCore.engineOutputs[q.slot]
                local online = qH.online > 0

                for r = 0, quadH - 1 do
                    self:safeBlit(scr, qx, qy + r, string.rep(" ", quadW), "0", "7")
                end
                self:safeBlit(scr, qx + 1, qy, string.format("%s (%dE)", q.title, qH.total), online and "9" or "e", "8")
                self:safeBlit(scr, qx + 1, qy + 1, string.format("THR: %4.1f/15.0", val), "5", "7")
                self:safeBlit(scr, qx + 1, qy + 2, string.format("PWM: %2d/15", sig), "3", "7")
                if quadH >= 4 then
                    self:safeBlit(scr, qx + 1, qy + 3, string.format("ACTIVE: %d/%d", qH.online, qH.total), online and "5" or "e", "7")
                end
            end

            self:safeBlit(scr, 2, h, string.format("BASE: %4.2f/15 | %s", FlightCore.state.baseThrottle, FlightCore.state.statusMsg:sub(1, w - 20)), "0", "b")
end
