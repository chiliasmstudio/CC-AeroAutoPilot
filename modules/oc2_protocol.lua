-- OC-Aero protocol v1 validation, command dispatch, status projection, and ACK cache.
-- This module is embedded in fcc.lua and is also loaded by offline Lua tests.
local M = {
    protocol = "oc-aero/1",
    request_channel = 104,
    reply_channel = 105,
    max_json_bytes = 4096,
    cache_ms = 30000,
    max_cache_entries = 256
}

local COMMANDS = {
    HOLD_ALT = { method = "holdAltitude" },
    LOCK_ALT = { method = "lockCurrentAlt" },
    CALIB = { method = "startCalibration" },
    STOP = { method = "stopEngines" },
    CANCEL_WAYPOINT = { method = "cancelWaypoint" },
    RESCAN_HW = { method = "scanQuadTurtles" },
    SYNC_HEADING = { method = "syncHeading" },
    TOGGLE_HEADING = { method = "toggleHeadingHold" },
    STOP_HORIZ = { method = "stopHorizontalThrust" },
    SET_ALT = { method = "setTargetAlt", field = "alt", min = 0, max = 1000 },
    ADJ_ALT = { method = "adjustTargetAlt", field = "delta", min = -1000, max = 1000 },
    ADJ_BASE = { method = "adjustBaseThrottle", field = "delta", min = -15, max = 15 },
    SET_FWD = { method = "setForwardThrottle", field = "val", min = -4, max = 4 },
    SET_TURN = { method = "setTurnThrottle", field = "val", min = -2.5, max = 2.5 },
    SET_HEADING = { method = "setTargetHeading", field = "hdg", min = 0, max = 359, integer = true },
    CALIBRATE_HEADING = { method = "calibrateHeading", field = "hdg", min = 0, max = 359, integer = true },
    SET_WAYPOINT = { method = "setWaypoint", waypoint = true }
}

local NO_ARGUMENT = {
    HOLD_ALT = true, LOCK_ALT = true, CALIB = true, STOP = true,
    CANCEL_WAYPOINT = true, RESCAN_HW = true, SYNC_HEADING = true,
    TOGGLE_HEADING = true, STOP_HORIZ = true
}

local SUPPORTED_COMMANDS = {
    "HOLD_ALT", "LOCK_ALT", "CALIB", "STOP", "CANCEL_WAYPOINT",
    "RESCAN_HW", "SYNC_HEADING", "TOGGLE_HEADING", "STOP_HORIZ",
    "SET_ALT", "ADJ_ALT", "ADJ_BASE", "SET_FWD", "SET_TURN",
    "SET_HEADING", "CALIBRATE_HEADING", "SET_WAYPOINT"
}

local LEGACY_COMMANDS = {
    ADJ_FWD = { method = "adjustForwardThrottle", field = "delta" },
    ADJ_TURN = { method = "adjustTurnThrottle", field = "delta" },
    ADJ_HEADING = { method = "adjustTargetHeading", field = "delta" }
}

local function finiteNumber(value)
    return type(value) == "number" and value == value and value ~= math.huge and value ~= -math.huge
end

local function integer(value, minimum, maximum)
    return finiteNumber(value) and value % 1 == 0 and value >= minimum and value <= maximum
end

M.isFiniteNumber = finiteNumber

local function exactFields(value, fields)
    if type(value) ~= "table" then return false end
    local count = 0
    for key in pairs(value) do
        count = count + 1
        if count > 10 then return false end
        if not fields[key] then return false end
    end
    return true
end

function M.validateCommand(command)
    if type(command) ~= "table" or type(command.cmd) ~= "string" then
        return nil, "INVALID_ARGUMENT"
    end
    local spec = COMMANDS[command.cmd]
    if not spec then return nil, "UNSUPPORTED_COMMAND" end

    if NO_ARGUMENT[command.cmd] then
        if not exactFields(command, { cmd = true }) then return nil, "INVALID_ARGUMENT" end
        return command.cmd
    end

    if spec.waypoint then
        if not exactFields(command, { cmd = true, x = true, z = true, y = true, radius = true })
            or not finiteNumber(command.x) or command.x < -30000000 or command.x > 30000000
            or not finiteNumber(command.z) or command.z < -30000000 or command.z > 30000000
            or (command.y ~= nil and (not finiteNumber(command.y) or command.y < 0 or command.y > 1000))
            or (command.radius ~= nil and (not finiteNumber(command.radius) or command.radius < 3 or command.radius > 64)) then
            return nil, "INVALID_ARGUMENT"
        end
        return command.cmd
    end

    if not exactFields(command, { cmd = true, [spec.field] = true })
        or not finiteNumber(command[spec.field])
        or command[spec.field] < spec.min or command[spec.field] > spec.max
        or (spec.integer and command[spec.field] % 1 ~= 0) then
        return nil, "INVALID_ARGUMENT"
    end
    return command.cmd
end

local function validateEnvelope(request, fccId, requireExpiry)
    if type(request) ~= "table" then return nil, "INVALID_ARGUMENT" end
    if not integer(request.target, 0, 2147483647) then return nil, "INVALID_ARGUMENT" end
    if request.target ~= fccId then return nil, "TARGET_MISMATCH" end
    if request.protocol ~= M.protocol then return nil, "INVALID_PROTOCOL" end
    if request.type ~= "request" or request.channel ~= M.request_channel then return nil, "INVALID_ARGUMENT" end
    if type(request.client) ~= "string" or #request.client < 1 or #request.client > 64
        or not integer(request.id, 1, 2147483647) then
        return nil, "INVALID_ARGUMENT"
    end

    if request.op == "hello" then
        if not exactFields(request, { protocol = true, type = true, channel = true, client = true, id = true, target = true, op = true }) then
            return nil, "INVALID_ARGUMENT"
        end
    elseif request.op == "status" then
        if not exactFields(request, { protocol = true, type = true, channel = true, client = true, id = true, target = true, op = true, detail = true })
            or (request.detail ~= "flight" and request.detail ~= "hardware") then
            return nil, "INVALID_ARGUMENT"
        end
    elseif request.op == "command" then
        if not exactFields(request, { protocol = true, type = true, channel = true, client = true, id = true, target = true, op = true, command = true, expires_at = true }) then
            return nil, "INVALID_ARGUMENT"
        end
        if requireExpiry and not integer(request.expires_at, 0, 9007199254740991) then
            return nil, "INVALID_ARGUMENT"
        end
        local commandName, commandError = M.validateCommand(request.command)
        if not commandName then return nil, commandError end
        return commandName
    else
        return nil, "UNSUPPORTED_OPERATION"
    end
    return true
end

function M.validateRequest(request, fccId, requireExpiry)
    return validateEnvelope(request, fccId, requireExpiry == true)
end

local function quote(value)
    return '"' .. value:gsub('[%z\1-\31\\"]', function(char)
        local escapes = { ['\\'] = '\\\\', ['"'] = '\\"', ['\b'] = '\\b', ['\f'] = '\\f', ['\n'] = '\\n', ['\r'] = '\\r', ['\t'] = '\\t' }
        return escapes[char] or string.format("\\u%04x", char:byte())
    end) .. '"'
end

local function canonical(value, depth)
    depth = depth or 0
    if depth > 12 then error("JSON nesting limit") end
    local kind = type(value)
    if kind == "nil" then return "null" end
    if kind == "boolean" then return value and "true" or "false" end
    if kind == "number" then
        if not finiteNumber(value) then error("non-finite JSON number") end
        return string.format("%.17g", value)
    end
    if kind == "string" then return quote(value) end
    if kind ~= "table" then error("unsupported JSON value") end

    local keys, isArray, count = {}, true, 0
    for key in pairs(value) do
        count = count + 1
        if type(key) ~= "number" or key % 1 ~= 0 or key < 1 then isArray = false end
        keys[#keys + 1] = key
    end
    if isArray then
        for i = 1, count do if value[i] == nil then isArray = false; break end end
    end
    if isArray then
        local parts = {}
        for i = 1, count do parts[i] = canonical(value[i], depth + 1) end
        return "[" .. table.concat(parts, ",") .. "]"
    end
    table.sort(keys, function(a, b) return tostring(a) < tostring(b) end)
    local parts = {}
    for i, key in ipairs(keys) do
        if type(key) ~= "string" then error("JSON object key must be a string") end
        parts[i] = quote(key) .. ":" .. canonical(value[key], depth + 1)
    end
    return "{" .. table.concat(parts, ",") .. "}"
end

local function deepCopy(value)
    if type(value) ~= "table" then return value end
    local result = {}
    for key, item in pairs(value) do result[key] = deepCopy(item) end
    return result
end

local function rounded(value)
    if not finiteNumber(value) then return nil end
    local scaled = value * 1000
    local result = (scaled >= 0 and math.floor(scaled + 0.5) or math.ceil(scaled - 0.5)) / 1000
    if result == 0 then return 0 end
    return result
end

local function copyNumber(source, target, key)
    local value = rounded(source and source[key])
    if value ~= nil then target[key] = value end
end

local function copyBoolean(source, target, key)
    if source and type(source[key]) == "boolean" then target[key] = source[key] end
end

local function copyHardware(source)
    if type(source) ~= "table" then return nil end
    local result = { displays = {}, engines = {}, avionics = {} }
    local displays = source.displays or {}
    copyBoolean(displays, result.displays, "hasDirectGpu")
    copyNumber(displays, result.displays, "tomsCount")
    copyNumber(displays, result.displays, "monitorsCount")
    copyNumber(displays, result.displays, "totalScreens")

    local engines = source.engines or {}
    for _, key in ipairs({ "lift", "cruise", "steer" }) do
        local group = engines[key]
        if type(group) == "table" then
            result.engines[key] = {}
            copyNumber(group, result.engines[key], "online")
            copyNumber(group, result.engines[key], "total")
        end
    end
    copyNumber(engines, result.engines, "totalOnline")
    copyNumber(engines, result.engines, "totalCount")

    local avionics = source.avionics or {}
    for _, key in ipairs({ "onBody", "aic", "ins", "navTable", "alti", "gimbal" }) do
        copyBoolean(avionics, result.avionics, key)
    end
    copyNumber(avionics, result.avionics, "modems")
    if type(avionics.navSource) == "string" then result.avionics.navSource = avionics.navSource:sub(1, 32) end
    return result
end

local function statusProjection(snapshot, detail)
    local source = type(snapshot) == "table" and snapshot or {}
    local telemetry = { nav = {}, outputs = {} }
    for _, key in ipairs({ "currentAlt", "targetAlt", "baseThrottle", "vspeed" }) do copyNumber(source, telemetry, key) end
    if type(source.mode) == "string" then telemetry.mode = source.mode:sub(1, 24) end
    if type(source.statusMsg) == "string" then telemetry.statusMsg = source.statusMsg:sub(1, 160) end

    local nav = type(source.nav) == "table" and source.nav or {}
    for _, key in ipairs({ "fixReady", "headingReady", "positionReady", "wpActive" }) do copyBoolean(nav, telemetry.nav, key) end
    if nav.positionReady == true then
        for _, key in ipairs({ "x", "y", "z", "speed" }) do copyNumber(nav, telemetry.nav, key) end
    end
    if nav.headingReady == true then
        copyNumber(nav, telemetry.nav, "yaw")
        copyNumber(nav, telemetry.nav, "targetHeading")
    end
    local outputs = type(source.outputs) == "table" and source.outputs or {}
    for _, key in ipairs({ "FL", "FR", "BL", "BR", "FWD", "BWD", "LEFT", "RIGHT" }) do copyNumber(outputs, telemetry.outputs, key) end

    if detail == "hardware" then telemetry.hw = copyHardware(source.hw) end
    return telemetry
end

local function dispatch(core, command)
    local spec = COMMANDS[command.cmd] or LEGACY_COMMANDS[command.cmd]
    local fn = core and core[spec.method]
    if type(fn) ~= "function" then return false, "COMMAND_UNAVAILABLE" end

    local ok, result, reason = pcall(function()
        if spec.waypoint then
            return fn(command.x, command.z, command.radius, command.y)
        elseif spec.field then
            return fn(command[spec.field])
        else
            return fn()
        end
    end)
    if not ok then return false, "COMMAND_FAILED" end
    if result == false then
        if reason == "Invalid heading" or reason == "Invalid waypoint" then return false, "INVALID_ARGUMENT" end
        return false, "COMMAND_REJECTED"
    end
    return true
end

function M.dispatchLegacy(core, message)
    if type(message) ~= "table" or type(message.cmd) ~= "string" then return false end
    if not COMMANDS[message.cmd] and not LEGACY_COMMANDS[message.cmd] then return false end
    local command = { cmd = message.cmd }
    local spec = COMMANDS[message.cmd] or LEGACY_COMMANDS[message.cmd]
    if spec.field then command[spec.field] = message[spec.field] or 0 end
    if spec.waypoint then
        command.x, command.z = message.x, message.z
        command.y, command.radius = message.y, message.radius
    end
    return dispatch(core, command)
end

local function cacheKey(request)
    return request.client .. "\0" .. tostring(request.id)
end

local function makeReply(fccId, request, ok, result, field)
    local reply = {
        protocol = M.protocol, type = "reply", channel = M.reply_channel,
        client = request.client, id = request.id, source = fccId, ok = ok
    }
    if ok then
        reply.result = result
        if field ~= nil then reply.command = field end
    else
        reply.error = result
    end
    return reply
end

local function trimCache(cache, now)
    local count, oldestKey, oldestAt = 0, nil, math.huge
    for key, entry in pairs(cache) do
        if now - entry.at > M.cache_ms then
            cache[key] = nil
        else
            count = count + 1
            if entry.at < oldestAt then oldestAt, oldestKey = entry.at, key end
        end
    end
    return count, oldestKey
end

function M.new(fccId, core)
    local cache = {}
    local api = {}

    function api.handle(request, now, sampledTelemetry)
        local commandName, validationError = validateEnvelope(request, fccId, true)
        if validationError == "TARGET_MISMATCH" then return nil, validationError end
        if not commandName then
            if type(request) ~= "table" or type(request.client) ~= "string" or #request.client < 1 or #request.client > 64
                or not integer(request.id, 1, 2147483647) then
                return nil, validationError or "INVALID_ARGUMENT"
            end
            return makeReply(fccId, request, false, validationError or "INVALID_ARGUMENT")
        end

        local fingerprintOk, fingerprint = pcall(canonical, request)
        if not fingerprintOk then return nil, "INVALID_ARGUMENT" end
        local key = cacheKey(request)
        local cachedCount, oldestKey = trimCache(cache, now)
        local previous = cache[key]
        if previous then
            if previous.fingerprint == fingerprint then return deepCopy(previous.reply), "REPLAY" end
            return makeReply(fccId, request, false, "ID_CONFLICT"), "ID_CONFLICT"
        end
        if cachedCount >= M.max_cache_entries and oldestKey then cache[oldestKey] = nil end

        local reply
        if request.op == "hello" then
            reply = makeReply(fccId, request, true, "hello")
            reply.supported_commands = deepCopy(SUPPORTED_COMMANDS)
        elseif request.op == "status" then
            reply = makeReply(fccId, request, true, "status")
            reply.telemetry = statusProjection(sampledTelemetry, request.detail)
            local encodedOk, encoded = pcall(canonical, reply)
            if not encodedOk or #encoded > M.max_json_bytes then
                reply = makeReply(fccId, request, false, "RESPONSE_TOO_LARGE")
            end
        elseif request.op == "command" then
            local nowEpoch = now
            if not integer(request.expires_at, 0, 9007199254740991) or request.expires_at < nowEpoch then
                reply = makeReply(fccId, request, false, "EXPIRED")
            else
                local applied, dispatchError = dispatch(core, request.command)
                if applied then
                    reply = makeReply(fccId, request, true, "applied", commandName)
                else
                    reply = makeReply(fccId, request, false, dispatchError)
                end
            end
        else
            reply = makeReply(fccId, request, false, validationError or "UNSUPPORTED_OPERATION")
        end

        cache[key] = { fingerprint = fingerprint, reply = deepCopy(reply), at = now }
        return reply
    end

    function api.cacheSize()
        local count = 0
        for _ in pairs(cache) do count = count + 1 end
        return count
    end

    return api
end

return M
