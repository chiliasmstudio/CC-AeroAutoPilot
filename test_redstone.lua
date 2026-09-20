--[[
    Wired Turtle & Engine Redstone Diagnostic Tool (有線烏龜紅石輸出診斷器)
    
    功能:
    1. 測試主電腦是否能透過有線網路向 Turtle / Wired Modem / Relay 下達紅石指令
    2. 逐面測試 (Bottom, Top, Left, Right, Front, Back) 輸出 15 級紅石
    3. 檢測到底哪一個面才是連接動力變速箱/離合器的面！
--]]

print("=== Turtle Redstone Diagnostics ===")

local pNames = peripheral.getNames()
local testNodes = {}

for _, name in ipairs(pNames) do
    local pType = peripheral.getType(name)
    if pType == "turtle" or pType == "computer" or pType == "redstone_relay" or pType == "modem" then
        local p = peripheral.wrap(name)
        local lbl = (p.getLabel and p.getLabel()) or name
        table.insert(testNodes, { name = name, p = p, label = lbl, type = pType })
    end
end

if #testNodes == 0 then
    print("[ERROR] No wired nodes found on network!")
    print("Please make sure Wired Modems are right-clicked (red ring on).")
    return
end

print(string.format("Found %d testable node(s):\n", #testNodes))
for i, n in ipairs(testNodes) do
    print(string.format(" [%d] %-15s (Type: %s, Label: %s)", i, n.name, n.type, n.label))
end

print("\n-------------------------------------------")
print("Select a node number to test (or 'all'):")
write("Target: ")
local sel = read()

local targetNodes = {}
if sel == "all" then
    targetNodes = testNodes
else
    local idx = tonumber(sel)
    if idx and testNodes[idx] then
        table.insert(targetNodes, testNodes[idx])
    else
        targetNodes = testNodes
    end
end

print("\n--- Starting Full-Power Redstone Test (Signal 15) ---")
print("We will test each face for 3 seconds. Look at your engine to see when it spins!\n")

local sides = {"bottom", "top", "left", "right", "front", "back"}

for _, node in ipairs(targetNodes) do
    print(string.format("Testing Node: %s [%s]...", node.name, node.label))
    
    -- 1. 測試全部面同時輸出 15
    print("  -> Testing ALL SIDES = 15 for 3s...")
    for _, s in ipairs(sides) do
        pcall(function() 
            if node.p.setAnalogOutput then node.p.setAnalogOutput(s, 15) end
            if node.p.setOutput then node.p.setOutput(s, true) end
        end)
    end
    sleep(3.0)
    
    -- 關閉
    for _, s in ipairs(sides) do
        pcall(function() 
            if node.p.setAnalogOutput then node.p.setAnalogOutput(s, 0) end
            if node.p.setOutput then node.p.setOutput(s, false) end
        end)
    end
    sleep(0.5)

    -- 2. 逐面單獨測試
    for _, s in ipairs(sides) do
        write(string.format("  -> Testing face [%s] = 15 ... ", s))
        local ok, err = pcall(function()
            if node.p.setAnalogOutput then 
                node.p.setAnalogOutput(s, 15) 
            elseif node.p.setOutput then
                node.p.setOutput(s, true)
            end
        end)
        if ok then print("Output OK!") else print("ERR: " .. tostring(err)) end
        sleep(1.5)
        pcall(function()
            if node.p.setAnalogOutput then node.p.setAnalogOutput(s, 0) end
            if node.p.setOutput then node.p.setOutput(s, false) end
        end)
    end
    print("-------------------------------------------")
end

print("\n[Diagnostic Done] Did your engine spin during 'ALL SIDES' or any specific face?")
