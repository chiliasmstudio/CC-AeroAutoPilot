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
