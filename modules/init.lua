--[[
    modules/init.lua
    Unified Avionics Modules Loader for CC: Tweaked
--]]

local function safeRequire(mod)
    local ok, res = pcall(require, mod)
    if ok then return res end
    return nil
end

local FlightCore = safeRequire("modules.flight_core")
local FONT_5X7 = safeRequire("modules.font_5x7")
local DirectDriver = safeRequire("modules.drivers.directgpu")
local TomDriver = safeRequire("modules.drivers.tom")
local NormalDriver = safeRequire("modules.drivers.normal")

return {
    FlightCore = FlightCore,
    FONT_5X7 = FONT_5X7,
    Drivers = {
        direct = DirectDriver,
        tom = TomDriver,
        normal = NormalDriver
    }
}
