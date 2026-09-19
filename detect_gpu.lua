--[[
    DirectGPU Peripheral Diagnostics & Scanner
    用來檢測電腦周圍所有連接的周邊設備名稱與類型
--]]

print("=== Peripheral Scanner ===")
local pList = peripheral.getNames()

if #pList == 0 then
    print("[WARN] No peripherals attached to this computer!")
else
    print(string.format("Found %d peripheral(s):", #pList))
    for _, name in ipairs(pList) do
        local pType = peripheral.getType(name)
        print(string.format("  * Side/Name: %-15s | Type: %s", name, pType))
    end
end

print("\n--- DirectGPU Check ---")
local gpu = peripheral.find("directgpu") or peripheral.find("direct_gpu") or peripheral.find("gpu")
if gpu then
    print("[SUCCESS] DirectGPU peripheral found!")
else
    print("[FAILED] Could not find any DirectGPU peripheral.")
    print("Please check:")
    print("1. Is the DirectGPU block placed directly adjacent to the computer (or wired modem)?")
    print("2. Is the CC-DirectGPU-Mod jar installed in the mods folder?")
end
