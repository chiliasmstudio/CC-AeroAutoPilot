-- ========================================================
-- Turtle Engine Node Firmware (startup.lua)
-- Version: v4.0.11 (OTA Remote Update & Auto Modem Detection Edition)
-- 放置於動力烏龜中，開機自動啟動，免手動操作
-- ========================================================

local VERSION = "v4.0.11"

-- 1. 取得烏龜標籤 (FL / FR / BL / BR / FWD / BWD / LEFT / RIGHT)
local label = os.getComputerLabel() or ""
local role = "UNKNOWN"

local function resolveRole(lbl)
    if not lbl or lbl == "" then return nil end
    local u = string.upper(tostring(lbl)):gsub("^%s*(.-)%s*$", "%1")

    -- 1. 四象限垂直升力 (FL, FR, BL, BR)
    if u == "FL" or u:find("^FL[-_ ]") or u:find("^前左") or u == "FRONT_LEFT" then return "FL"
    elseif u == "FR" or u:find("^FR[-_ ]") or u:find("^前右") or u == "FRONT_RIGHT" then return "FR"
    elseif u == "BL" or u:find("^BL[-_ ]") or u:find("^後左") or u == "BACK_LEFT" then return "BL"
    elseif u == "BR" or u:find("^BR[-_ ]") or u:find("^後右") or u == "BACK_RIGHT" then return "BR"
    end

    -- 2. 前進 / 前推 (Forward / FWD)
    if u == "FWD" or u == "FORWARD" or u == "FRONT" or u == "前進" or u == "前推" or u == "前" or
       u:find("^FWD[-_ ]") or u:find("^FORWARD[-_ ]") or u:find("^PUSH_FWD") or u:find("^PUSH_FORWARD") or
       u:find("^前進") or u:find("^前推") then
        return "FWD"
    end

    -- 3. 後退 / 後推 (Backward / BWD)
    if u == "BWD" or u == "BACK" or u == "BACKWARD" or u == "REVERSE" or u == "後退" or u == "後推" or u == "後" or
       u:find("^BWD[-_ ]") or u:find("^BACK[-_ ]") or u:find("^PUSH_BWD") or u:find("^PUSH_BACK") or
       u:find("^後退") or u:find("^後推") then
        return "BWD"
    end

    -- 4. 左推 / 左轉向 (Left Turn / 面向左)
    if u == "LEFT" or u == "TURN_LEFT" or u == "PUSH_LEFT" or u == "ORIENT_LEFT" or u == "TL" or
       u == "左" or u == "左推" or u == "左轉" or u == "左舵" or u == "左向" or u == "左平移" or
       u:find("^LEFT[-_ ]") or u:find("^PUSH_LEFT") or u:find("^TURN_LEFT") or
       u:find("^左推") or u:find("^左轉") or u:find("^左舵") then
        return "LEFT"
    end

    -- 5. 右推 / 右轉向 (Right Turn / 面向右)
    if u == "RIGHT" or u == "TURN_RIGHT" or u == "PUSH_RIGHT" or u == "ORIENT_RIGHT" or u == "TR" or
       u == "右" or u == "右推" or u == "右轉" or u == "右舵" or u == "右向" or u == "右平移" or
       u:find("^RIGHT[-_ ]") or u:find("^PUSH_RIGHT") or u:find("^TURN_RIGHT") or
       u:find("^右推") or u:find("^右轉") or u:find("^右舵") then
        return "RIGHT"
    end

    return nil
end

role = resolveRole(label) or "UNKNOWN"

-- 2. 動態偵測數據機與紅石輸出面 (除了接觸 modem 的面不輸出紅石外，其他面全部輸出)
local allSides = {"top", "bottom", "left", "right", "front", "back"}
local activeModems = {}
local outputSides = {}

local function refreshSides()
    activeModems = {}
    outputSides = {}
    for _, side in ipairs(allSides) do
        if peripheral.getType(side) == "modem" then
            local m = peripheral.wrap(side)
            if m then
                pcall(function() m.open(100) end)
                table.insert(activeModems, m)
            end
            pcall(function() rs.setAnalogOutput(side, 0) end)
        else
            table.insert(outputSides, side)
        end
    end
end

refreshSides()

term.clear()
term.setCursorPos(1, 1)
print("================================")
print("     VTOL Airship " .. VERSION .. "     ")
print("  Engine Node (OTA Active)       ")
print("================================")
print("ID: " .. os.getComputerID() .. " | Label: " .. (label ~= "" and label or "(None)"))
print("Role: [" .. role .. "] | Ver: " .. VERSION)

if #activeModems == 0 then
    print("WARNING: No Modem attached!")
    print("Please attach a Wired Modem.")
else
    print("Modem Active on " .. #activeModems .. " side(s).")
    print("Redstone Outputs: " .. table.concat(outputSides, ", "))
    print("Listening: Ch 100 | Heartbeat: Ch 101")
end

-- 3. 紅石輸出控制
local currentSig = 0
local hbCount = 0

local function applySignal(power)
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
        local eventData = { os.pullEvent() }
        local event = eventData[1]
        if event == "modem_message" then
            local side, channel, replyChannel, message = eventData[2], eventData[3], eventData[4], eventData[5]
            if channel == 100 and type(message) == "table" then
                -- 遠端指令: 重啟烏龜 (Remote Reboot)
                if message.cmd == "REBOOT" or message.type == "REBOOT" then
                    term.setCursorPos(1, 9)
                    term.clearLine()
                    print("[REMOTE] Rebooting turtle...")
                    applySignal(0)
                    sleep(0.5)
                    os.reboot()
                    return
                end

                -- 遠端指令: 線上更新韌體 (Remote OTA Update)
                if message.cmd == "UPDATE" or message.type == "OTA_UPDATE" then
                    term.setCursorPos(1, 9)
                    term.clearLine()
                    print("[OTA] Downloading latest firmware...")
                    applySignal(0)
                    local url = "https://raw.githubusercontent.com/chiliasmstudio/CC-AeroAutoPilot/main/turtle_startup.lua"
                    local ok, resp = pcall(function() return http.get(url, {["Cache-Control"]="no-cache"}) end)
                    if ok and resp then
                        local code = resp.readAll()
                        resp.close()
                        if code and #code > 50 then
                            local f = fs.open("startup.lua", "w")
                            if f then
                                f.write(code)
                                f.close()
                                print("[OTA] Firmware updated! Rebooting...")
                                sleep(1)
                                os.reboot()
                                return
                            end
                        end
                    end
                    print("[OTA ERR] Update failed. Check HTTP connection.")
                end

                -- 動力控制訊號處理
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
                    term.setCursorPos(1, 9)
                    term.clearLine()
                    term.write(string.format("[%s] VTOL %s | Sig: %2d/15 | HB: #%d", role, VERSION, sig, hbCount))
                end
            end
        elseif event == "peripheral" or event == "peripheral_detach" then
            refreshSides()
            applySignal(currentSig)
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
