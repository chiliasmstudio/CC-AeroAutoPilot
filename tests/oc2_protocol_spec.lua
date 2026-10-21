local Protocol = require("modules.oc2_protocol")

local function request(id, operation, extra)
    local value = {
        protocol = Protocol.protocol, type = "request", channel = 104,
        client = "test-client", id = id, target = 42, op = operation
    }
    for key, item in pairs(extra or {}) do value[key] = item end
    return value
end

test("hello and status return protocol-shaped responses from sampled data", function()
    local core = { unexpectedSensorReads = 0 }
    local handler = Protocol.new(42, core)
    local hello = handler.handle(request(1, "hello"), 1000, {})
    assert(hello.ok and hello.result == "hello" and hello.channel == 105 and #hello.supported_commands > 0)
    local snapshot = {
        mode = "HOLD_ALT", currentAlt = 74.12345, targetAlt = 80, baseThrottle = 2.5,
        vspeed = -0.25, statusMsg = "Holding", nav = { x = 10, y = 74, z = -20, yaw = 90,
            targetHeading = 90, speed = 0.125, fixReady = true, headingReady = true,
            positionReady = true, wpActive = false },
        outputs = { FL = 2, FR = 2, BL = 2, BR = 2, FWD = 0, BWD = 0, LEFT = 0, RIGHT = 0 }
    }
    local status = handler.handle(request(2, "status", { detail = "flight" }), 1100, snapshot)
    assert(status.ok and status.result == "status")
    assert(status.telemetry.currentAlt == 74.123 and status.telemetry.nav.yaw == 90)
    assert(status.telemetry.outputs.FL == 2 and status.telemetry.hw == nil)
    assert(core.unexpectedSensorReads == 0)
end)

test("hardware status is a compact projection and unknown measurements stay omitted", function()
    local handler = Protocol.new(42, {})
    local snapshot = {
        mode = "IDLE", targetAlt = 0, baseThrottle = 1, currentAlt = 0,
        nav = { x = 12, yaw = 0, headingReady = false, positionReady = false, fixReady = false },
        outputs = { FL = 0 },
        hw = {
            displays = { hasDirectGpu = true, tomsCount = 2, monitorsCount = 1, totalScreens = 4 },
            engines = { lift = { online = 4, total = 4, nodes = { "must-not-leak" } }, totalOnline = 8 },
            avionics = { onBody = true, alti = false, navSource = "CCPE-AIC-DETAIL-STRING" }
        }
    }
    local reply = handler.handle(request(3, "status", { detail = "hardware" }), 1000, snapshot)
    assert(reply.ok and reply.telemetry.hw.displays.tomsCount == 2)
    assert(reply.telemetry.hw.engines.lift.online == 4 and reply.telemetry.hw.engines.lift.nodes == nil)
    assert(reply.telemetry.hw.avionics.navSource == "CCPE-AIC-DETAIL-STRING")
    assert(reply.telemetry.currentAlt == 0 and reply.telemetry.nav.yaw == nil)
    assert(reply.telemetry.nav.x == nil and reply.telemetry.nav.positionReady == false)
end)

test("command allowlist enforces types, finite values, ranges, and exact fields", function()
    assert(Protocol.validateCommand({ cmd = "ADJ_BASE", delta = 0.5 }) == "ADJ_BASE")
    assert(Protocol.validateCommand({ cmd = "SET_WAYPOINT", x = -30000000, z = 30000000, radius = 64 }) == "SET_WAYPOINT")
    assert(select(2, Protocol.validateCommand({ cmd = "SET_ALT", alt = math.huge })) == "INVALID_ARGUMENT")
    assert(select(2, Protocol.validateCommand({ cmd = "SET_TURN", val = -2.51 })) == "INVALID_ARGUMENT")
    assert(select(2, Protocol.validateCommand({ cmd = "SET_HEADING", hdg = 90.5 })) == "INVALID_ARGUMENT")
    assert(select(2, Protocol.validateCommand({ cmd = "STOP", extra = true })) == "INVALID_ARGUMENT")
    assert(select(2, Protocol.validateCommand({ cmd = "RUN_LUA", source = "" })) == "UNSUPPORTED_COMMAND")
    assert(select(2, Protocol.validateCommand({ cmd = "SET_WAYPOINT", x = 1, z = 2, y = -1 })) == "INVALID_ARGUMENT")
end)

test("malformed envelopes reject, and another FCC target is silently ignored", function()
    local handler = Protocol.new(42, {})
    local badProtocol = request(1, "hello")
    badProtocol.protocol = "other"
    local reply = handler.handle(badProtocol, 1000, {})
    assert(reply and reply.ok == false and reply.error == "INVALID_PROTOCOL")
    local badType = request(2, "hello")
    badType.type = "telemetry"
    reply = handler.handle(badType, 1000, {})
    assert(reply and reply.ok == false)
    local wrongTarget = request(3, "command", { command = { cmd = "STOP" }, expires_at = 1500 })
    wrongTarget.target = 43
    reply = handler.handle(wrongTarget, 1000, {})
    assert(reply == nil)
end)

test("ADJ_ALT executes once on replay; id reuse with changed content conflicts", function()
    local calls = 0
    local core = { adjustTargetAlt = function(delta) calls = calls + 1; coreDelta = delta end }
    local handler = Protocol.new(42, core)
    local command = request(10, "command", { command = { cmd = "ADJ_ALT", delta = 10 }, expires_at = 3500 })
    local first = handler.handle(command, 1000, {})
    local duplicate, replay = handler.handle(command, 1200, {})
    assert(first.ok and first.result == "applied" and first.command == "ADJ_ALT")
    assert(duplicate.ok and replay == "REPLAY" and calls == 1 and coreDelta == 10)

    local conflict = request(10, "command", { command = { cmd = "ADJ_ALT", delta = -5 }, expires_at = 3500 })
    local refused, reason = handler.handle(conflict, 1300, {})
    assert(refused.ok == false and refused.error == "ID_CONFLICT" and reason == "ID_CONFLICT" and calls == 1)
end)

test("expired commands do not execute, and base adjustment preserves delta semantics", function()
    local calls, observedDelta = 0, nil
    local core = { adjustBaseThrottle = function(delta) calls = calls + 1; observedDelta = delta end }
    local handler = Protocol.new(42, core)
    local expired = request(20, "command", { command = { cmd = "ADJ_ALT", delta = 10 }, expires_at = 999 })
    local reply = handler.handle(expired, 1000, {})
    assert(reply.ok == false and reply.error == "EXPIRED" and calls == 0)
    local base = request(21, "command", { command = { cmd = "ADJ_BASE", delta = 0.5 }, expires_at = 1200 })
    reply = handler.handle(base, 1000, {})
    assert(reply.ok and observedDelta == 0.5 and calls == 1)
end)

test("calibration refusal returns a negative ACK and cache remains bounded", function()
    local core = { startCalibration = function() return false, "No altitude sensor" end }
    local handler = Protocol.new(42, core)
    local reply = handler.handle(request(1, "command", { command = { cmd = "CALIB" }, expires_at = 2000 }), 1000, {})
    assert(reply.ok == false and reply.error == "COMMAND_REJECTED")
    for id = 2, 260 do handler.handle(request(id, "hello"), 1000 + id, {}) end
    assert(handler.cacheSize() <= 256)
end)

test("all OC2 command names map only to their allowlisted FlightCore methods", function()
    local calls = {}
    local function method(name)
        return function(...) calls[#calls + 1] = { name, ... }; return true end
    end
    local core = {
        holdAltitude = method("holdAltitude"), lockCurrentAlt = method("lockCurrentAlt"),
        startCalibration = method("startCalibration"), stopEngines = method("stopEngines"),
        cancelWaypoint = method("cancelWaypoint"), scanQuadTurtles = method("scanQuadTurtles"),
        syncHeading = method("syncHeading"), toggleHeadingHold = method("toggleHeadingHold"),
        stopHorizontalThrust = method("stopHorizontalThrust"), setTargetAlt = method("setTargetAlt"),
        adjustTargetAlt = method("adjustTargetAlt"), adjustBaseThrottle = method("adjustBaseThrottle"),
        setForwardThrottle = method("setForwardThrottle"), setTurnThrottle = method("setTurnThrottle"),
        setTargetHeading = method("setTargetHeading"), calibrateHeading = method("calibrateHeading"),
        setWaypoint = method("setWaypoint"), adjustForwardThrottle = method("adjustForwardThrottle"),
        adjustTurnThrottle = method("adjustTurnThrottle"), adjustTargetHeading = method("adjustTargetHeading")
    }
    local handler = Protocol.new(42, core)
    local commands = {
        { cmd = "HOLD_ALT" }, { cmd = "LOCK_ALT" }, { cmd = "CALIB" }, { cmd = "STOP" },
        { cmd = "CANCEL_WAYPOINT" }, { cmd = "RESCAN_HW" }, { cmd = "SYNC_HEADING" },
        { cmd = "TOGGLE_HEADING" }, { cmd = "STOP_HORIZ" }, { cmd = "SET_ALT", alt = 200 },
        { cmd = "ADJ_ALT", delta = 10 }, { cmd = "ADJ_BASE", delta = 0.5 },
        { cmd = "SET_FWD", val = 1 }, { cmd = "SET_TURN", val = -1 },
        { cmd = "SET_HEADING", hdg = 90 }, { cmd = "CALIBRATE_HEADING", hdg = 180 },
        { cmd = "SET_WAYPOINT", x = 100, z = 300, y = 200, radius = 12 }
    }
    for id, command in ipairs(commands) do
        local reply = handler.handle(request(id, "command", { command = command, expires_at = 2000 }), 1000, {})
        assert(reply.ok and reply.command == command.cmd)
    end
    assert(#calls == #commands)
    assert(calls[12][1] == "adjustBaseThrottle" and calls[12][2] == 0.5)
    assert(calls[17][1] == "setWaypoint" and calls[17][2] == 100 and calls[17][3] == 300 and calls[17][4] == 12 and calls[17][5] == 200)
    assert(Protocol.dispatchLegacy(core, { cmd = "ADJ_FWD", delta = 0.25 }) == true)
    assert(calls[#calls][1] == "adjustForwardThrottle" and calls[#calls][2] == 0.25)
    assert(Protocol.dispatchLegacy(core, { cmd = "LOADSTRING", source = "STOP" }) == false)
end)
