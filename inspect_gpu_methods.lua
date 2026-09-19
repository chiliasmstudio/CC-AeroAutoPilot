--[[
    DirectGPU API Inspector
    列印出 gpu 支援的所有方法名稱，尋找是否有可關閉滑鼠輸入或設定距離的 API
--]]

local gpu = peripheral.find("directgpu")
if not gpu then
    print("DirectGPU not found!")
    return
end

print("=== DirectGPU Methods ===")
local methods = peripheral.getMethods(peripheral.getName(gpu))
table.sort(methods)

for _, m in ipairs(methods) do
    if string.find(string.lower(m), "input") or 
       string.find(string.lower(m), "mouse") or 
       string.find(string.lower(m), "touch") or 
       string.find(string.lower(m), "scroll") or 
       string.find(string.lower(m), "range") or 
       string.find(string.lower(m), "reach") or 
       string.find(string.lower(m), "interact") or
       string.find(string.lower(m), "disable") or
       string.find(string.lower(m), "enable") or
       string.find(string.lower(m), "config") then
        print(" -> " .. m)
    end
end

print("\n--- All methods count: " .. #methods .. " ---")
for i, m in ipairs(methods) do
    write(m .. ", ")
    if i % 4 == 0 then print("") end
end
print("")
