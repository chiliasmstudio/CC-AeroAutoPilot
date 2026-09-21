-- ========================================================
-- Turtle Engine Node Firmware (startup.lua)
-- 放置於 4 隻烏龜中，開機自動啟動，免手動操作
-- ========================================================

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
print("  AVIONICS TURTLE ENGINE NODE   ")
print("================================")
print("ID: " .. os.getComputerID() .. " | Label: " .. (label ~= "" and label or "(None)"))
print("Role: [" .. role .. "]")

-- 2. 尋找並開啟數據機 (支援貼在任何一面的 Wired 或 Wireless Modem)
local modemSides = {"top", "bottom", "left", "right", "front", "back"}
local activeModems = {}

for _, side in ipairs(modemSides) do
    if peripheral.getType(side) == "modem" then
        local m = peripheral.wrap(side)
        m.open(100) -- 廣播頻道
        table.insert(activeModems, m)
    end
end

if #activeModems == 0 then
    print("WARNING: No Modem attached!")
    print("Please attach a Wired Modem.")
else
    print("Modem Active on " .. #activeModems .. " side(s).")
    print("Listening on Channel 100...")
end

-- 3. 5 方向輸出列表 (排除正面)
local outputSides = {"bottom", "top", "left", "right", "back"}

local function applySignal(power)
    -- 純類比紅石輸出 (0 ~ 15)，絕不呼叫 setOutput(true) 避免訊號被強制放大為 15 滿功率暴衝
    power = math.max(0, math.min(15, math.floor(power + 0.5)))
    for _, s in ipairs(outputSides) do
        rs.setAnalogOutput(s, power)
    end
end

-- 初始輸出 0
applySignal(0)

-- 4. 監聽主電腦控制廣播
while true do
    local event, side, channel, replyChannel, message, distance = os.pullEvent("modem_message")
    if channel == 100 and type(message) == "table" then
        -- 判斷是否為給本烏龜的訊號 (支援角色比對、ID 比對、或全域廣播)
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
            term.setCursorPos(1, 6)
            term.clearLine()
            term.write(string.format("Role: [%s]  Thrust: %2d/15", role, sig))
        end
    end
end
