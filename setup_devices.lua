--[[
    Airship Device Setup & Identification Wizard (設備角色識別與綁定嚮導)
    
    功能:
    1. 掃描有線網路上所有設備 (Modem, Relay, Throttle Lever, Speed Controller 等)
    2. 點擊/選擇設備時，對該設備發送「高亮測試訊號 (Blink Test)」(讓對應的引擎轉動或閃爍)，讓你知道它是哪一個！
    3. 讓你為設備指定角色 (例如: "升降引擎-左前", "前進主推力", "航向轉向")
    4. 自動儲存為 airship_config.json
--]]

print("=== Airship Device Setup Wizard ===")

local allPeripherals = peripheral.getNames()
local config = {
    altitude_engines = {},
    forward_engines = {},
    steering = {}
}

if #allPeripherals == 0 then
    print("[WARN] No wired peripherals found on network!")
    print("Please make sure Wired Modems are right-clicked to activate (red light on).")
    return
end

print(string.format("Found %d devices on network.\n", #allPeripherals))

for i, name in ipairs(allPeripherals) do
    local pType = peripheral.getType(name)
    local p = peripheral.wrap(name)
    
    print("---------------------------------------------")
    print(string.format("[%d/%d] Device Name: %s (Type: %s)", i, #allPeripherals, name, pType))
    print("Testing device... (Blinking 15 -> 0)")
    
    -- 測試激勵：發送信號讓該設備動一下，方便玩家肉眼辨認是哪台引擎
    pcall(function()
        if pType == "redstone_relay" then
            p.setAnalogOutput("bottom", 15)
            sleep(0.5)
            p.setAnalogOutput("bottom", 0)
        elseif pType == "throttle_lever" then
            p.setSignal(15)
            sleep(0.5)
            p.setSignal(0)
        end
    end)
    
    print("What is this device used for?")
    print("  1. Altitude Engine (垂直升降)")
    print("  2. Forward Thrust  (前進推進)")
    print("  3. Steering / Yaw  (轉向偏航)")
    print("  4. Skip / Ignore   (忽略/其他)")
    
    write("Select (1-4, default 4): ")
    local choice = read()
    
    if choice == "1" then
        table.insert(config.altitude_engines, name)
        print("-> Assigned as: Altitude Engine")
    elseif choice == "2" then
        table.insert(config.forward_engines, name)
        print("-> Assigned as: Forward Thrust")
    elseif choice == "3" then
        table.insert(config.steering, name)
        print("-> Assigned as: Steering")
    else
        print("-> Skipped.")
    end
end

-- 儲存設定檔
local f = fs.open("airship_config.json", "w")
f.write(textutils.serialiseJSON(config))
f.close()

print("\n=============================================")
print("[SUCCESS] Device mapping saved to 'airship_config.json'!")
print(string.format("Configured: %d Altitude, %d Forward, %d Steering.",
    #config.altitude_engines, #config.forward_engines, #config.steering))
