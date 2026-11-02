--[[
    CC-AeroAutoPilot OC2 Serial Bridge
    Version: v4.6.2
    Dedicated CC computer: wired modem + OC2 Network Connector serial peripheral.
    Example: oc2-bridge --fcc 42 --address 1 --peer 2 --baud 9600
--]]

local OC2Wire = (function()
-- OC2 serial wire format: official raw frame + OA1 application fragmentation.
-- This module intentionally has no ComputerCraft or OpenComputers dependency.
local M = {
    max_payload = 200,
    max_message = 4096,
    chunk_size = 192,
    max_pending = 8,
    expiry = 10
}

local function byteInteger(value, maximum)
    return type(value) == "number" and value % 1 == 0 and value >= 0 and value <= maximum
end

local function checksum(to, from, payload)
    local sum = (to + from + #payload) % 256
    for index = 1, #payload do sum = (sum + payload:byte(index)) % 256 end
    return sum
end

local function findNextValidFrame(buffer, startAt)
    local cursor, incomplete = startAt, nil
    while cursor <= #buffer do
        local marker = buffer:find("\1", cursor, true)
        if not marker then break end
        local remaining = #buffer - marker + 1
        if remaining >= 4 then
            local to, from, length = buffer:byte(marker + 1, marker + 3)
            if length <= M.max_payload then
                local frameLength = length + 5
                if remaining >= frameLength then
                    local payload = buffer:sub(marker + 4, marker + 3 + length)
                    if buffer:byte(marker + frameLength - 1) == checksum(to, from, payload) then
                        return marker
                    end
                elseif not incomplete then
                    incomplete = marker
                end
            end
        elseif not incomplete then
            incomplete = marker
        end
        cursor = marker + 1
    end
    return nil, incomplete
end

function M.encodeFrame(to, from, payload)
    assert(byteInteger(to, 254) and byteInteger(from, 254), "Serial addresses must be 0..254")
    assert(type(payload) == "string" and #payload <= M.max_payload, "Invalid serial payload")
    return string.char(1, to, from, #payload) .. payload .. string.char(checksum(to, from, payload))
end

-- Parse as many complete frames as possible. The unconsumed suffix is returned.
-- Invalid bytes are dropped one at a time until a valid marker/frame is found.
function M.consumeFrames(buffer, onFrame)
    if type(buffer) ~= "string" then return "", 1 end
    local errors = 0
    while #buffer > 0 do
        local marker = buffer:find("\1", 1, true)
        if not marker then
            errors = errors + #buffer
            return "", errors
        end
        if marker > 1 then
            errors = errors + marker - 1
            buffer = buffer:sub(marker)
        end
        if #buffer < 4 then return buffer, errors end
        local to, from, length = buffer:byte(2, 4)
        if length > M.max_payload then
            errors = errors + 1
            buffer = buffer:sub(2)
        else
            local frameLength = length + 5
            if #buffer < frameLength then return buffer, errors end
            local payload = buffer:sub(5, 4 + length)
            local receivedChecksum = buffer:byte(frameLength)
            if receivedChecksum ~= checksum(to, from, payload) then
                errors = errors + 1
                local nextFrame, incomplete = findNextValidFrame(buffer, 2)
                if nextFrame then
                    buffer = buffer:sub(nextFrame)
                elseif incomplete then
                    buffer = buffer:sub(incomplete)
                    return buffer, errors
                else
                    buffer = buffer:sub(frameLength + 1)
                end
            else
                onFrame(to, from, payload)
                buffer = buffer:sub(frameLength + 1)
            end
        end
    end
    return buffer, errors
end

function M.split(text, transfer)
    assert(type(text) == "string" and #text > 0 and #text <= M.max_message, "Invalid message size")
    assert(byteInteger(transfer, 65535), "Invalid transfer ID")
    local count, parts = math.ceil(#text / M.chunk_size), {}
    for index = 1, count do
        parts[index] = "OA1" .. string.char(math.floor(transfer / 256), transfer % 256, index, count)
            .. text:sub((index - 1) * M.chunk_size + 1, index * M.chunk_size)
    end
    return parts
end

function M.new(now)
    assert(type(now) == "function", "Clock callback required")
    local pending = {}
    local api = {}

    function api.expire()
        local time, removed = now(), 0
        for key, entry in pairs(pending) do
            if time - entry.started > M.expiry then pending[key] = nil; removed = removed + 1 end
        end
        return removed
    end

    function api.push(sender, payload)
        api.expire()
        if not byteInteger(sender, 254) or type(payload) ~= "string" or #payload < 8
            or #payload > M.chunk_size + 7 or payload:sub(1, 3) ~= "OA1" then
            return nil, "INVALID_FRAGMENT"
        end
        local high, low, index, count = payload:byte(4, 7)
        if count < 1 or count > math.ceil(M.max_message / M.chunk_size) or index < 1 or index > count then
            return nil, "INVALID_FRAGMENT_INDEX"
        end
        local chunk = payload:sub(8)
        if #chunk < 1 or #chunk > M.chunk_size or (index < count and #chunk ~= M.chunk_size) then
            return nil, "INVALID_FRAGMENT_LENGTH"
        end

        local transfer = high * 256 + low
        local key = tostring(sender) .. ":" .. tostring(transfer)
        local entry = pending[key]
        if not entry then
            local entries = 0
            for _ in pairs(pending) do entries = entries + 1 end
            if entries >= M.max_pending then return nil, "REASSEMBLY_LIMIT" end
            entry = { started = now(), count = count, received = 0, size = 0, parts = {} }
            pending[key] = entry
        end
        if entry.count ~= count or (entry.parts[index] ~= nil and entry.parts[index] ~= chunk) then
            pending[key] = nil
            return nil, "CONFLICTING_FRAGMENT"
        end
        if entry.parts[index] == nil then
            entry.parts[index] = chunk
            entry.received = entry.received + 1
            entry.size = entry.size + #chunk
        end
        if entry.size > M.max_message then pending[key] = nil; return nil, "MESSAGE_TOO_LARGE" end
        if entry.received == entry.count then
            pending[key] = nil
            return table.concat(entry.parts)
        end
        return nil
    end

    function api.pendingCount()
        local count = 0
        for _ in pairs(pending) do count = count + 1 end
        return count
    end

    return api
end

return M
end)()

local OC2Protocol = (function()
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
end)()

local OC2Bridge = (function()
-- Standalone CC serial/modem bridge. See agent.md for topology and installation.
local Wire = OC2Wire or require("modules.oc2_wire")
local Protocol = OC2Protocol or require("modules.oc2_protocol")
local M = { max_queued_requests = 8, request_timeout_ms = 7000, command_lifetime_ms = 2500 }

local function integer(value, minimum, maximum)
    return type(value) == "number" and value == value and value ~= math.huge and value ~= -math.huge
        and value % 1 == 0 and value >= minimum and value <= maximum
end

local function clone(value)
    if type(value) ~= "table" then return value end
    local result = {}
    for key, item in pairs(value) do result[key] = clone(item) end
    return result
end

function M.parseArgs(args)
    local config = { address = 1, peer = 2, baud = 9600 }
    local flags = {
        ["--fcc"] = "fcc_id", ["--address"] = "address", ["--peer"] = "peer",
        ["--baud"] = "baud", ["--serial"] = "serial_name", ["--modem"] = "modem_name"
    }
    local index = 1
    while index <= #args do
        local flag = args[index]
        local field = flags[flag]
        if not field then
            if flag == "--help" or flag == "-h" then config.help = true; index = index + 1
            else return nil, "Unknown argument: " .. tostring(flag) end
        else
            local value = args[index + 1]
            if value == nil then return nil, flag .. " requires a value" end
            if field == "serial_name" or field == "modem_name" then config[field] = value
            else config[field] = tonumber(value) end
            index = index + 2
        end
    end

    if config.help then return config end
    if not integer(config.fcc_id, 0, 2147483647) then return nil, "Specify --fcc <FCC computer ID>" end
    if not integer(config.address, 0, 254) or not integer(config.peer, 0, 254) or config.address == config.peer then
        return nil, "Serial addresses must be distinct integers from 0 to 254 (255 broadcast is disabled)"
    end
    local baudRates = { [9600] = true, [19200] = true, [38400] = true, [57600] = true, [115200] = true }
    if not baudRates[config.baud] then return nil, "Unsupported baud rate" end
    if config.serial_name ~= nil and (type(config.serial_name) ~= "string" or #config.serial_name == 0) then
        return nil, "Invalid --serial peripheral name"
    end
    if config.modem_name ~= nil and (type(config.modem_name) ~= "string" or #config.modem_name == 0) then
        return nil, "Invalid --modem peripheral name"
    end
    return config
end

function M.create(config, environment)
    local env = environment or {}
    local nowEpoch = env.epoch or function() return os.epoch("utc") end
    local nowSeconds = env.now or function() return os.clock() end
    local decodeJSON = env.decode or function(text) return textutils.unserializeJSON(text) end
    local encodeJSON = env.encode or function(value) return textutils.serializeJSON(value) end
    local state = {
        serial = nil, serialName = nil, modem = nil, modemName = nil,
        receiveBuffer = "", reassembly = Wire.new(nowSeconds),
        requestQueue = {}, active = nil, transmit = nil,
        stats = { badFrames = 0, badMessages = 0, modemErrors = 0, serialErrors = 0, dropped = 0 }
    }

    local bridge = {}

    local function validUtf8(text)
        local index = 1
        while index <= #text do
            local a = text:byte(index)
            if a <= 0x7f then index = index + 1
            elseif a >= 0xc2 and a <= 0xdf then
                local b = text:byte(index + 1)
                if not b or b < 0x80 or b > 0xbf then return false end
                index = index + 2
            elseif a == 0xe0 then
                local b, c = text:byte(index + 1, index + 2)
                if not b or not c or b < 0xa0 or b > 0xbf or c < 0x80 or c > 0xbf then return false end
                index = index + 3
            elseif (a >= 0xe1 and a <= 0xec) or (a >= 0xee and a <= 0xef) then
                local b, c = text:byte(index + 1, index + 2)
                if not b or not c or b < 0x80 or b > 0xbf or c < 0x80 or c > 0xbf then return false end
                index = index + 3
            elseif a == 0xed then
                local b, c = text:byte(index + 1, index + 2)
                if not b or not c or b < 0x80 or b > 0x9f or c < 0x80 or c > 0xbf then return false end
                index = index + 3
            elseif a == 0xf0 then
                local b, c, d = text:byte(index + 1, index + 3)
                if not b or not c or not d or b < 0x90 or b > 0xbf or c < 0x80 or c > 0xbf or d < 0x80 or d > 0xbf then return false end
                index = index + 4
            elseif a >= 0xf1 and a <= 0xf3 then
                local b, c, d = text:byte(index + 1, index + 3)
                if not b or not c or not d or b < 0x80 or b > 0xbf or c < 0x80 or c > 0xbf or d < 0x80 or d > 0xbf then return false end
                index = index + 4
            elseif a == 0xf4 then
                local b, c, d = text:byte(index + 1, index + 3)
                if not b or not c or not d or b < 0x80 or b > 0x8f or c < 0x80 or c > 0xbf or d < 0x80 or d > 0xbf then return false end
                index = index + 4
            else return false end
        end
        return true
    end

    local function resetLink()
        state.receiveBuffer = ""
        state.reassembly = Wire.new(nowSeconds)
        state.requestQueue = {}
        state.active = nil
        state.transmit = nil
    end

    local function enqueueSerialReply(reply, active)
        local ok, encoded = pcall(encodeJSON, reply)
        if not ok or type(encoded) ~= "string" or #encoded < 1 or #encoded > Wire.max_message then
            state.stats.serialErrors = state.stats.serialErrors + 1
            state.stats.dropped = state.stats.dropped + 1
            state.active = nil
            return
        end
        local okParts, parts = pcall(Wire.split, encoded, active.request.id % 65536)
        if not okParts then
            state.stats.serialErrors = state.stats.serialErrors + 1
            state.stats.dropped = state.stats.dropped + 1
            state.active = nil
            return
        end
        local frames, bytes = {}, 0
        for _, part in ipairs(parts) do
            local frame = Wire.encodeFrame(active.peer, config.address, part)
            frames[#frames + 1] = frame
            bytes = bytes + #frame
        end
        if #frames > 22 or bytes > 4510 then
            state.stats.dropped = state.stats.dropped + 1
            state.active = nil
            return
        end
        state.transmit = { frames = frames, frameIndex = 1, offset = 1, startedAt = nowEpoch(), active = active }
        active.phase = "serial_write"
    end

    local function processMessage(sender, payload)
        local complete, fragmentError = state.reassembly.push(sender, payload)
        if fragmentError then state.stats.badMessages = state.stats.badMessages + 1; return end
        if not complete then return end
        if #complete > Wire.max_message or not validUtf8(complete) then
            state.stats.badMessages = state.stats.badMessages + 1
            return
        end
        local okDecode, request = pcall(decodeJSON, complete)
        if not okDecode or type(request) ~= "table" then
            state.stats.badMessages = state.stats.badMessages + 1
            return
        end
        local commandName, validationError = Protocol.validateRequest(request, config.fcc_id, false)
        if not commandName then
            state.stats.badMessages = state.stats.badMessages + 1
            return
        end

        local queued = clone(request)
        local receivedAt = nowEpoch()
        if queued.op == "command" then queued.expires_at = receivedAt + M.command_lifetime_ms end
        if #state.requestQueue >= M.max_queued_requests then
            state.stats.dropped = state.stats.dropped + 1
            return
        end
        state.requestQueue[#state.requestQueue + 1] = {
            request = queued, peer = sender, receivedAt = receivedAt,
            commandName = queued.op == "command" and commandName or nil
        }
        bridge.startNext()
    end

    local function onFrame(to, from, payload)
        if to ~= config.address or from ~= config.peer or from == 255 then
            state.stats.badFrames = state.stats.badFrames + 1
            return
        end
        processMessage(from, payload)
    end

    function bridge.setHardware(serial, serialName, modem, modemName)
        local serialChanged = serialName ~= state.serialName
        local modemChanged = modemName ~= state.modemName
        if serialChanged or modemChanged then resetLink() end

        state.serial, state.serialName = serial, serialName
        state.modem, state.modemName = modem, modemName
        if serialChanged and serial then
            local okAddress = pcall(function() return serial.setAddress(config.address) end)
            local okBaud = pcall(function() return serial.setBaudRate(config.baud) end)
            if not okAddress or not okBaud then
                state.stats.serialErrors = state.stats.serialErrors + 1
                state.serial, state.serialName = nil, nil
            end
        end
        if modemChanged and modem then
            local okWireless, wireless = pcall(function() return modem.isWireless() end)
            if not okWireless or wireless ~= false then
                state.stats.modemErrors = state.stats.modemErrors + 1
                state.modem, state.modemName = nil, nil
            else
                local okOpen = pcall(function() return modem.open(105) end)
                if not okOpen then
                    state.stats.modemErrors = state.stats.modemErrors + 1
                    state.modem, state.modemName = nil, nil
                end
            end
        end
        bridge.startNext()
    end

    function bridge.startNext()
        if state.active or state.transmit or not state.serial or not state.modem then return end
        while #state.requestQueue > 0 do
            local item = table.remove(state.requestQueue, 1)
            local now = nowEpoch()
            if now - item.receivedAt > M.request_timeout_ms
                or (item.commandName and item.request.expires_at <= now) then
                state.stats.dropped = state.stats.dropped + 1
            else
                local active = {
                    request = item.request, peer = item.peer,
                    deadline = now + M.request_timeout_ms, phase = "modem_wait"
                }
                state.active = active
                local ok = pcall(function() state.modem.transmit(104, 105, item.request) end)
                if not ok then
                    state.stats.modemErrors = state.stats.modemErrors + 1
                    state.stats.dropped = state.stats.dropped + 1
                    state.active = nil
                else
                    return
                end
            end
        end
    end

    function bridge.feedSerial(bytes, corruptedCount)
        if type(corruptedCount) == "number" and corruptedCount > 0 then
            state.stats.badFrames = state.stats.badFrames + math.floor(corruptedCount)
            state.receiveBuffer = ""
            state.reassembly = Wire.new(nowSeconds)
            -- The returned bytes may begin mid-frame after byte loss. Discard this
            -- read and wait for the next clean serial event/timer sweep to resync.
            return
        end
        if type(bytes) == "string" and #bytes > 0 then
            state.receiveBuffer = state.receiveBuffer .. bytes
            local remaining, errors = Wire.consumeFrames(state.receiveBuffer, onFrame)
            state.receiveBuffer = remaining
            state.stats.badFrames = state.stats.badFrames + errors
        end
    end

    function bridge.onModemMessage(channel, replyChannel, message)
        local active = state.active
        if channel ~= 105 or replyChannel ~= 104 or not active or active.phase ~= "modem_wait" or type(message) ~= "table" then return end
        if message.protocol ~= Protocol.protocol or message.type ~= "reply" or message.channel ~= 105
            or message.source ~= config.fcc_id or message.client ~= active.request.client
            or message.id ~= active.request.id or type(message.ok) ~= "boolean" then
            state.stats.modemErrors = state.stats.modemErrors + 1
            return
        end
        if message.ok and type(message.result) ~= "string" then
            state.stats.modemErrors = state.stats.modemErrors + 1
            return
        end
        if not message.ok and type(message.error) ~= "string" then
            state.stats.modemErrors = state.stats.modemErrors + 1
            return
        end
        if message.ok then
            local operation = active.request.op
            if (operation == "hello" and message.result ~= "hello")
                or (operation == "status" and (message.result ~= "status" or type(message.telemetry) ~= "table"))
                or (operation == "command" and (message.result ~= "applied" or message.command ~= active.request.command.cmd)) then
                state.stats.modemErrors = state.stats.modemErrors + 1
                return
            end
        end
        enqueueSerialReply(message, active)
    end

    function bridge.readSerial()
        if not state.serial then return end
        local ok, bytes, corrupted = pcall(function() return state.serial.read() end)
        if not ok then
            state.stats.serialErrors = state.stats.serialErrors + 1
            state.receiveBuffer = ""
            state.reassembly = Wire.new(nowSeconds)
            return
        end
        bridge.feedSerial(bytes, corrupted)
    end

    function bridge.tick()
        state.reassembly.expire()
        local now = nowEpoch()
        local kept = {}
        for _, item in ipairs(state.requestQueue) do
            if now - item.receivedAt <= M.request_timeout_ms
                and (not item.commandName or item.request.expires_at > now) then
                kept[#kept + 1] = item
            else state.stats.dropped = state.stats.dropped + 1 end
        end
        state.requestQueue = kept

        if state.active and now > state.active.deadline then
            state.stats.dropped = state.stats.dropped + 1
            state.active = nil
            state.transmit = nil
        end
        local tx = state.transmit
        if tx and state.serial then
            -- Bounded work per timer: drain partial writes promptly without busy-waiting.
            for _ = 1, 8 do
                tx = state.transmit
                if not tx then break end
                local frame = tx.frames[tx.frameIndex]
                if not frame then state.transmit, state.active = nil, nil; break end
                local remaining = frame:sub(tx.offset)
                local ok, written = pcall(function() return state.serial.write(remaining) end)
                if not ok then
                    state.stats.serialErrors = state.stats.serialErrors + 1
                    state.stats.dropped = state.stats.dropped + 1
                    state.transmit, state.active = nil, nil
                    break
                elseif type(written) == "number" and written > 0 and written <= #remaining then
                    tx.offset = tx.offset + written
                    if tx.offset > #frame then tx.frameIndex, tx.offset = tx.frameIndex + 1, 1 end
                    if tx.frameIndex > #tx.frames then state.transmit, state.active = nil, nil; break end
                elseif type(written) == "number" and written > #remaining then
                    state.stats.serialErrors = state.stats.serialErrors + 1
                    state.stats.dropped = state.stats.dropped + 1
                    state.transmit, state.active = nil, nil
                    break
                else
                    break
                end
            end
        end
        bridge.startNext()
    end

    function bridge.onPeripheralChange(serial, serialName, modem, modemName)
        bridge.setHardware(serial, serialName, modem, modemName)
    end

    function bridge.snapshot()
        return {
            serial = state.serialName, modem = state.modemName,
            active = state.active ~= nil, queued = #state.requestQueue,
            stats = clone(state.stats)
        }
    end

    return bridge
end

local USAGE = "Usage: oc2-bridge --fcc <id> [--serial <name>] [--modem <name>] [--address 1] [--peer 2] [--baud 9600]"

local function selectNamedPeripheral(peripheralApi, names, kind, explicitName)
    local matches = {}
    for _, name in ipairs(names) do
        if peripheralApi.getType(name) == kind then matches[#matches + 1] = name end
    end
    if explicitName then
        for _, name in ipairs(matches) do if name == explicitName then return name end end
        return nil, "Waiting for " .. kind .. " peripheral '" .. explicitName .. "'"
    end
    if #matches == 1 then return matches[1] end
    if #matches == 0 then return nil, "Waiting for " .. kind .. " peripheral" end
    return nil, "Multiple " .. kind .. " peripherals; specify --" .. (kind == "modem" and "modem" or "serial") .. " <name>"
end

function M.run(args, environment)
    local env = environment or _G
    local config, configError = M.parseArgs(args or {})
    if not config then (env.printError or env.print)(configError .. "\n" .. USAGE); return 1 end
    if config.help then (env.print or print)(USAGE); return 0 end

    local controller = M.create(config, {
        epoch = function() return env.os.epoch("utc") end,
        now = function() return env.os.clock() end,
        decode = function(text) return env.textutils.unserializeJSON(text) end,
        encode = function(value) return env.textutils.serializeJSON(value) end
    })
    local selectedSerial, selectedModem, lastReport = nil, nil, nil

    local function rescan()
        local names = env.peripheral.getNames()
        local serialName, serialErr = selectNamedPeripheral(env.peripheral, names, "oc2_serial_interface", config.serial_name)
        local modemName, modemErr = selectNamedPeripheral(env.peripheral, names, "modem", config.modem_name)
        local serial = serialName and env.peripheral.wrap(serialName) or nil
        local modem = modemName and env.peripheral.wrap(modemName) or nil
        if serialName and not serial then serialErr = "Cannot wrap serial peripheral '" .. serialName .. "'" end
        if modemName and not modem then modemErr = "Cannot wrap modem peripheral '" .. modemName .. "'" end
        if serialName ~= selectedSerial or modemName ~= selectedModem then
            selectedSerial, selectedModem = serialName, modemName
            controller.onPeripheralChange(serial, serialName, modem, modemName)
        end
        if serialErr or modemErr then lastReport = serialErr or modemErr else lastReport = nil end
    end

    local function safeRescan()
        local ok, err = pcall(rescan)
        if not ok then lastReport = "Peripheral scan failed: " .. tostring(err):sub(1, 80) end
    end

    safeRescan()
    local timer = env.os.startTimer(0.1)
    local reportAt = 0
    while true do
        local event, a, b, c, d, e = env.os.pullEventRaw()
        if event == "timer" and a == timer then
            timer = env.os.startTimer(0.1)
            controller.readSerial()
            controller.tick()
            local now = env.os.clock()
            if now - reportAt >= 1 then
                reportAt = now
                local state = controller.snapshot()
                local message = string.format("OC2 bridge | FCC %d | serial %s | wired modem %s | active %s | queued %d",
                    config.fcc_id, state.serial or "waiting", state.modem or "waiting", tostring(state.active), state.queued)
                if lastReport then message = message .. " | " .. lastReport end
                if env.term and env.term.setCursorPos and env.term.clear then
                    env.term.clear(); env.term.setCursorPos(1, 1); env.print(message)
                    env.print(string.format("Errors frame:%d message:%d serial:%d modem:%d dropped:%d",
                        state.stats.badFrames, state.stats.badMessages, state.stats.serialErrors,
                        state.stats.modemErrors, state.stats.dropped))
                end
            end
        elseif event == "oc2_serial_data" and a == selectedSerial then
            controller.readSerial()
        elseif event == "modem_message" and b == 105 then
            controller.onModemMessage(b, c, d)
        elseif event == "peripheral" or event == "peripheral_detach" then
            safeRescan()
        elseif event == "terminate" then
            return 0
        end
    end
end

return M
end)()

local exitCode = OC2Bridge.run({...})
if exitCode and exitCode ~= 0 then error("OC2 bridge failed with status " .. tostring(exitCode), 0) end
