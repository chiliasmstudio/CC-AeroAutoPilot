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
