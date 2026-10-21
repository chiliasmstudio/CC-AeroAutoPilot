local Wire = require("modules.oc2_wire")

local function hex(value)
    return (value:gsub(".", function(byte) return string.format("%02x", byte:byte()) end))
end

test("golden raw serial fixture matches every byte", function()
    local part = Wire.split("hello", 0x1234)[1]
    assert(hex(part) == "4f41311234010168656c6c6f")
    assert(hex(Wire.encodeFrame(1, 2, part)) == "0101020c4f41311234010168656c6c6f2c")
end)

test("raw frames survive split reads and glued frames", function()
    local frames = {}
    local first = Wire.encodeFrame(1, 2, "alpha")
    local second = Wire.encodeFrame(1, 2, "beta")
    local suffix, errors = Wire.consumeFrames(first:sub(1, 4), function(to, from, payload)
        frames[#frames + 1] = { to, from, payload }
    end)
    assert(#frames == 0 and #suffix == 4 and errors == 0)
    suffix, errors = Wire.consumeFrames(suffix .. first:sub(5) .. second, function(to, from, payload)
        frames[#frames + 1] = { to, from, payload }
    end)
    assert(#suffix == 0 and errors == 0 and #frames == 2)
    assert(frames[1][1] == 1 and frames[1][2] == 2 and frames[1][3] == "alpha")
    assert(frames[2][3] == "beta")
end)

test("bad checksum and garbage resynchronize to next complete frame", function()
    local frames, errors = {}, 0
    local bad = Wire.encodeFrame(1, 2, "bad")
    bad = bad:sub(1, -2) .. string.char((bad:byte(-1) + 1) % 256)
    local good = Wire.encodeFrame(1, 2, "good")
    local suffix, count = Wire.consumeFrames("junk" .. bad .. good, function(to, from, payload)
        frames[#frames + 1] = payload
    end)
    errors = errors + count
    assert(suffix == "" and errors > 0 and #frames == 1 and frames[1] == "good")
end)

test("OA1 reassembles out of order and accepts identical duplicate fragments", function()
    local payload = string.rep("abc", 200)
    local parts = Wire.split(payload, 71)
    local now, reassembly = 0, nil
    reassembly = Wire.new(function() return now end)
    assert(reassembly.push(2, parts[3]) == nil)
    assert(reassembly.push(2, parts[3]) == nil)
    assert(reassembly.push(2, parts[1]) == nil)
    assert(reassembly.push(2, parts[2]) == nil)
    assert(reassembly.push(2, parts[4]) == payload)
    assert(reassembly.pendingCount() == 0)
end)

test("OA1 rejects conflicting fragments, caps groups, and expires stale groups", function()
    local now = 0
    local reassembly = Wire.new(function() return now end)
    local parts = Wire.split(string.rep("x", 300), 5)
    assert(reassembly.push(2, parts[1]) == nil)
    local conflicting = "OA1" .. parts[1]:sub(4, 7) .. string.rep("y", 192)
    local value, err = reassembly.push(2, conflicting)
    assert(value == nil and err == "CONFLICTING_FRAGMENT" and reassembly.pendingCount() == 0)

    for transfer = 1, 8 do
        local fragment = Wire.split(string.rep("z", 300), transfer)[1]
        assert(reassembly.push(2, fragment) == nil)
    end
    local ninth = Wire.split(string.rep("z", 300), 9)[1]
    value, err = reassembly.push(2, ninth)
    assert(value == nil and err == "REASSEMBLY_LIMIT")
    now = 10.1
    assert(reassembly.expire() == 8 and reassembly.pendingCount() == 0)
end)

test("OA1 enforces payload limits and transfer ids", function()
    assert(not pcall(Wire.split, string.rep("a", 4097), 1))
    assert(not pcall(Wire.split, "hello", 65536))
    local reassembly = Wire.new(function() return 0 end)
    local value, err = reassembly.push(255, Wire.split("hello", 1)[1])
    assert(value == nil and err == "INVALID_FRAGMENT")
end)
