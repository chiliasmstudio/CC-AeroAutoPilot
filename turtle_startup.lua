-- ========================================================
-- Turtle Engine Node Firmware (startup.lua)
-- Version: v3.6.0 (Analog PWM & Heartbeat Telemetry Edition)
-- 放置於 4 隻烏龜中，開機自動啟動，免手動操作
-- ========================================================

local VERSION = "v3.6.0"

-- 1. 取得烏龜標籤 (FL / FR / BL / BR)
local label = os.getComputerLabel() or ""
local role = "UNKNOWN"

local function matchPrefix(lbl, prefix)
    local u = string.upper(lbl)
    return u == prefix or u:sub(1, #prefix + 1) == (prefix .. "_") or u:sub(1, #prefix + 1) == (prefix .. "-") or u:sub(1, #prefix + 1) == (prefix .. " ")
end

if matchPrefix(label, "FL") then role = "FL"
elseif matchPrefix(label, "FR") then role = "FR"
elseif matchPrefix(label, "BL") then role = "BL"
elseif matchPrefix(label, "BR") then role = "BR"
end

term.clear()
term.setCursorPos(1, 1)
print("================================")
print("     VTOL Airship " .. VERSION .. "     ")
print("  Engine Node (Heartbeat Active) ")
print("================================")
print("ID: " .. os.getComputerID() .. " | Label: " .. (label ~= "" and label or "(None)"))
print("Role: [" .. role .. "] | Ver: " .. VERSION)

-- 2. 尋找並開啟數據機 (支援貼在任何一面的 Wired 或 Wireless Modem)
local modemSides = {"top", "bottom", "left", "right", "front", "back"}
local activeModems = {}

for _, side in ipairs(modemSides) do
    if peripheral.getType(side) == "modem" then
        local m = peripheral.wrap(side)
        m.open(100) -- 下行控制廣播頻道 (Main -> Turtles)
        table.insert(activeModems, m)
    end
end

if #activeModems == 0 then
    print("WARNING: No Modem attached!")
    print("Please attach a Wired Modem.")
else
    print("Modem Active on " .. #activeModems .. " side(s).")
    print("Listening: Ch 100 | Heartbeat: Ch 101")
end

-- 3. 5 方向輸出列表 (排除正面)
local outputSides = {"bottom", "top", "left", "right", "back"}
local currentSig = 0
local hbCount = 0

local function applySignal(power)
    -- 純類比紅石輸出 (0 ~ 15)，絕不呼叫 setOutput(true) 避免訊號被強制放大為 15 滿功率暴衝
    power = math.max(0, math.min(15, math.floor(power + 0.5)))
    currentSig = power
    for _, s in ipairs(outputSides) do
        rs.setAnalogOutput(s, power)
    end
end

-- 初始輸出 0
applySignal(0)

-- 4. 心跳封包發送函數 (上行廣播: Ch 101)
local function sendHeartbeat()
    hbCount = hbCount + 1
    local payload = {
        type = "HEARTBEAT",
        role = role,
        id = os.getComputerID(),
        label = label,
        sig = currentSig,
        ver = VERSION,
        uptime = os.epoch("utc")
    }
    for _, m in ipairs(activeModems) do
        pcall(function()
            m.transmit(101, 100, payload)
        end)
    end
end

-- 5. 主控制監聽線程
local function listenLoop()
    while true do
        local event, side, channel, replyChannel, message, distance = os.pullEvent("modem_message")
        if channel == 100 and type(message) == "table" then
            local sig = nil
            if role ~= "UNKNOWN" and message[role] ~= nil then
                sig = message[role]
            elseif message[os.getComputerID()] ~= nil then
                sig = message[os.getComputerID()]
            elseif message[label] ~= nil then
                sig = message[label]
            end

            if sig ~= nil then
                applySignal(sig)
                term.setCursorPos(1, 8)
                term.clearLine()
                term.write(string.format("[%s] VTOL %s | Sig: %2d/15 | HB: #%d", role, VERSION, sig, hbCount))
            end
        end
    end
end

-- 6. 定時心跳封包發送線程 (1Hz 週期)
local function heartbeatLoop()
    while true do
        sendHeartbeat()
        sleep(1.0)
    end
end

parallel.waitForAll(listenLoop, heartbeatLoop)
