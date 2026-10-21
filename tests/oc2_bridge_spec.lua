local Wire = require("modules.oc2_wire")
local Protocol = require("modules.oc2_protocol")
local Bridge = require("modules.oc2_bridge")

local function frameRequest(request, address, peer, transfer)
    return Wire.encodeFrame(address, peer, Wire.split("REQUEST", transfer)[1])
end

test("bridge validates serial peer and relays one expiring request over channel 104", function()
    local epoch, now = 5000, 0
    local receivedRequest, writeBytes = nil, {}
    local request = { protocol = Protocol.protocol, type = "request", channel = 104,
        client = "serial-client", id = 7, target = 42, op = "command", command = { cmd = "ADJ_BASE", delta = 0.5 } }
    local serial = {
        setAddress = function(address) serialAddress = address end,
        setBaudRate = function(baud) serialBaud = baud end,
        write = function(bytes) local count = math.min(3, #bytes); writeBytes[#writeBytes + 1] = bytes:sub(1, count); return count end
    }
    local modem = {
        isWireless = function() return false end,
        open = function(channel) openedChannel = channel end,
        transmit = function(channel, replyChannel, message)
            assert(channel == 104 and replyChannel == 105)
            receivedRequest = message
        end
    }
    local bridge = Bridge.create({ fcc_id = 42, address = 1, peer = 2, baud = 9600 }, {
        epoch = function() return epoch end,
        now = function() return now end,
        decode = function(text) assert(text == "REQUEST"); return request end,
        encode = function(reply) assert(reply.result == "applied"); return "REPLY" end
    })
    bridge.setHardware(serial, "serial", modem, "wired")
    assert(serialAddress == 1 and serialBaud == 9600 and openedChannel == 105)

    bridge.feedSerial(Wire.encodeFrame(1, 3, Wire.split("REQUEST", 7)[1]))
    assert(receivedRequest == nil)
    bridge.feedSerial(frameRequest(request, 1, 2, 7))
    assert(receivedRequest and receivedRequest.expires_at == 7500)
    assert(receivedRequest.command.cmd == "ADJ_BASE" and receivedRequest.command.delta == 0.5)
    for _ = 1, 3 do bridge.tick() end
    assert(bridge.snapshot().active == true)
end)

test("bridge pairs FCC id/source/client, then drains partial serial writes without retry", function()
    local epoch = 8000
    local writes, transmissions = {}, 0
    local request = { protocol = Protocol.protocol, type = "request", channel = 104,
        client = "pair-client", id = 19, target = 42, op = "hello" }
    local serial = {
        setAddress = function() end, setBaudRate = function() end,
        write = function(bytes) local count = math.min(2, #bytes); writes[#writes + 1] = bytes:sub(1, count); return count end
    }
    local modem = { isWireless = function() return false end, open = function() end,
        transmit = function() transmissions = transmissions + 1 end }
    local bridge = Bridge.create({ fcc_id = 42, address = 1, peer = 2, baud = 9600 }, {
        epoch = function() return epoch end, now = function() return epoch / 1000 end,
        decode = function() return request end, encode = function() return "REPLY" end
    })
    bridge.setHardware(serial, "serial", modem, "wired")
    bridge.feedSerial(frameRequest(request, 1, 2, 19))
    assert(transmissions == 1)
    bridge.onModemMessage(105, 104, { protocol = Protocol.protocol, type = "reply", channel = 105,
        client = request.client, id = request.id, source = 43, ok = true, result = "hello" })
    assert(bridge.snapshot().active == true)
    bridge.onModemMessage(105, 104, { protocol = Protocol.protocol, type = "reply", channel = 105,
        client = request.client, id = request.id, source = 42, ok = true, result = "status" })
    assert(bridge.snapshot().active == true)
    bridge.onModemMessage(105, 104, { protocol = Protocol.protocol, type = "reply", channel = 105,
        client = request.client, id = request.id, source = 42, ok = true, result = "hello" })
    for _ = 1, 20 do bridge.tick() end
    assert(bridge.snapshot().active == false and transmissions == 1)
    assert(#writes > 1)
    local raw = table.concat(writes)
    local frames, suffix = {}, nil
    suffix = Wire.consumeFrames(raw, function(to, from, payload) frames[#frames + 1] = { to, from, payload } end)
    assert(suffix == "" and #frames == 1 and frames[1][1] == 2 and frames[1][2] == 1)
    local completed, reassembly = nil, Wire.new(function() return 0 end)
    completed = reassembly.push(1, frames[1][3])
    assert(completed == "REPLY")
end)

test("wireless modem is rejected and missing FCC response is not retried", function()
    local epoch, sends = 1000, 0
    local request = { protocol = Protocol.protocol, type = "request", channel = 104,
        client = "timeout", id = 1, target = 42, op = "hello" }
    local serial = { setAddress = function() end, setBaudRate = function() end }
    local wireless = { isWireless = function() return true end, open = function() end,
        transmit = function() sends = sends + 1 end }
    local bridge = Bridge.create({ fcc_id = 42, address = 1, peer = 2, baud = 9600 }, {
        epoch = function() return epoch end, now = function() return epoch / 1000 end,
        decode = function() return request end, encode = function() return "REPLY" end
    })
    bridge.setHardware(serial, "serial", wireless, "wireless")
    bridge.feedSerial(frameRequest(request, 1, 2, 1))
    assert(sends == 0)
    local wired = { isWireless = function() return false end, open = function() end,
        transmit = function() sends = sends + 1 end }
    bridge.setHardware(serial, "serial", wired, "wired")
    bridge.feedSerial(frameRequest(request, 1, 2, 1))
    assert(sends == 1)
    epoch = epoch + 7001
    bridge.tick()
    bridge.tick()
    assert(sends == 1 and bridge.snapshot().active == false)
end)

test("modem reconnect drops a queued command instead of replaying it", function()
    local epoch, sends = 1000, 0
    local request = { protocol = Protocol.protocol, type = "request", channel = 104,
        client = "reconnect", id = 5, target = 42, op = "command", command = { cmd = "STOP" } }
    local serial = { setAddress = function() end, setBaudRate = function() end }
    local bridge = Bridge.create({ fcc_id = 42, address = 1, peer = 2, baud = 9600 }, {
        epoch = function() return epoch end, now = function() return epoch / 1000 end,
        decode = function() return request end, encode = function() return "REPLY" end
    })
    bridge.setHardware(serial, "serial", nil, nil)
    bridge.feedSerial(frameRequest(request, 1, 2, 5))
    assert(bridge.snapshot().queued == 1 and sends == 0)
    local wired = { isWireless = function() return false end, open = function() end,
        transmit = function() sends = sends + 1 end }
    bridge.setHardware(serial, "serial", wired, "wired")
    assert(bridge.snapshot().queued == 0 and sends == 0)
end)

test("bridge CLI rejects broadcast and invalid configuration", function()
    local config = Bridge.parseArgs({ "--fcc", "42", "--address", "1", "--peer", "2", "--baud", "9600" })
    assert(config.fcc_id == 42 and config.address == 1 and config.peer == 2)
    assert(select(2, Bridge.parseArgs({ "--fcc", "42", "--address", "255" })) ~= nil)
    assert(select(2, Bridge.parseArgs({ "--fcc", "42", "--baud", "123" })) ~= nil)
end)

test("serial corruption discards partial raw frames and resumes at the next marker", function()
    local request, sends = { protocol = Protocol.protocol, type = "request", channel = 104,
        client = "corrupt", id = 8, target = 42, op = "hello" }, 0
    local serial = { setAddress = function() end, setBaudRate = function() end }
    local modem = { isWireless = function() return false end, open = function() end,
        transmit = function() sends = sends + 1 end }
    local bridge = Bridge.create({ fcc_id = 42, address = 1, peer = 2, baud = 9600 }, {
        epoch = function() return 1000 end, now = function() return 1 end,
        decode = function() return request end, encode = function() return "REPLY" end
    })
    bridge.setHardware(serial, "serial", modem, "wired")
    local full = frameRequest(request, 1, 2, 8)
    bridge.feedSerial(full:sub(1, 6))
    bridge.feedSerial(full:sub(7), 1)
    assert(sends == 0)
    bridge.feedSerial(full)
    assert(sends == 1)
end)
