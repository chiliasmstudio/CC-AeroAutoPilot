--[[
    Create: Avionics & CC: Tweaked
    All-in-One Wireless Turtle Engine Node (Omnidirectional / 全向紅石輸出版)
    
    特性:
    - 烏龜身上【不需要】高度計，由主電腦統一運算控制。
    - 支援上下前後左右 (top, bottom, left, right, front, back) 全方向紅石輸出！
    - 自動避開 Wireless Modem 佔用的面，向其他所有面同步輸出類比紅石推力。
--]]

print("=== Wireless Turtle Engine Node (Omni) ===")

-- 1. 初始化無線網路 (Rednet)
local modemSide = nil
local allSides = {"top", "bottom", "left", "right", "front", "back"}

for _, side in ipairs(allSides) do
    if peripheral.getType(side) == "modem" then
        modemSide = side
        break
    end
end

if modemSide then
    rednet.open(modemSide)
    print(string.format("[OK] Wireless Modem (ID: %d) opened on [%s]", os.getComputerID(), modemSide))
else
    print("[WARN] No Wireless Modem found!")
end

local currentOutput = 0

-- 向除 Modem 外的所有面輸出類比紅石訊號
local function setEngineOutput(signal)
    signal = math.max(0, math.min(15, math.floor(signal + 0.5)))
    currentOutput = signal

    for _, side in ipairs(allSides) do
        if side ~= modemSide then
            redstone.setAnalogOutput(side, signal)
        end
    end
end

-- 2. 無線指令監聽 (接收主控端算好的紅石推力)
local function networkLoop()
    print("[Radio] Listening for Master flight commands...")
    while true do
        local senderId, message, protocol = rednet.receive("flight_control")
        if type(message) == "table" then
            local cmd = message.cmd
            
            -- 主電腦廣播推力數值 (0~15)
            if cmd == "THROTTLE" then
                local sig = message.signal or 0
                setEngineOutput(sig)
                
            -- 主電腦命令停機
            elseif cmd == "STOP" then
                setEngineOutput(0)
                print("[Radio] Engine Stopped (Signal 0)")
            end
        end
    end
end

-- 3. 本地手動指令介面
local function cliLoop()
    print("\n[Turtle Engine Ready - Omni Output]")
    print("Outputting redstone to all sides (except modem).")
    print("Local Commands:")
    print("  set <0-15> - Manually set redstone output")
    print("  status     - Show current output signal")
    print("  stop       - Stop engine (Signal 0)\n")

    while true do
        write("Turtle> ")
        local input = read()
        if input then
            local parts = {}
            for w in string.gmatch(input, "%S+") do table.insert(parts, w) end
            local cmd = string.lower(parts[1] or "")

            if cmd == "set" and parts[2] then
                local sig = tonumber(parts[2]) or 0
                setEngineOutput(sig)
                print(string.format("[Manual] All output sides set to: %d", currentOutput))
            elseif cmd == "status" then
                print(string.format("Current Redstone Output: %d/15 (All sides)", currentOutput))
            elseif cmd == "stop" then
                setEngineOutput(0)
                print("[Stopped] Output set to 0 on all sides")
            end
        end
    end
end

parallel.waitForAll(networkLoop, cliLoop)
