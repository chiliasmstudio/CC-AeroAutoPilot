package.path = "./?.lua;./?/init.lua;" .. package.path

local passed, failed = 0, 0
function test(name, callback)
    local ok, message = pcall(callback)
    if ok then
        passed = passed + 1
        print("PASS " .. name)
    else
        failed = failed + 1
        print("FAIL " .. name .. ": " .. tostring(message))
    end
end

dofile("tests/oc2_wire_spec.lua")
dofile("tests/oc2_protocol_spec.lua")
dofile("tests/oc2_bridge_spec.lua")

print(string.format("OC2 tests: %d passed, %d failed", passed, failed))
if failed > 0 then error("OC2 test suite failed") end
