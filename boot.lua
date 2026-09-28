--[[
    boot.lua — CC-AeroAutoPilot Universal Release Launcher & Auto-Updater
    Version: v4.0.11
    ====================================================================
    自動從 GitHub Releases 下載最新版本並執行或安裝至本機磁碟。

    使用方式:
      boot.lua                  (顯示互動式選單 [1] Install [2] Run)
      boot.lua install          (顯示安裝選單 [1] FCC [2] CLI [3] Display [4] Run [5] Turtle)
      boot.lua install fcc      (直接安裝 分散式飛控大腦 fcc.lua 為開機自啟 startup.lua)
      boot.lua install cli      (直接安裝 獨立命令終端 fcc-cli.lua 為開機自啟 startup.lua)
      boot.lua install display  (直接安裝 駕駛艙螢幕 display.lua 為開機自啟 startup.lua)
      boot.lua install turtle   (直接安裝 動力烏龜韌體 startup.lua 並重啟)
      boot.lua cli              (直接從 Release 下載並執行 fcc-cli.lua)
      boot.lua fcc              (直接從 Release 下載並執行 fcc.lua)
      boot.lua display          (直接從 Release 下載並執行 display.lua)
      boot.lua run              (直接從 Release 下載並執行 run.lua)
--]]

local rawArgs = { ... }
local OWNER = "chiliasmstudio"
local REPO = "CC-AeroAutoPilot"
local BRANCH = "main"

if not http then
    printError("Error: HTTP API is disabled in ComputerCraft config.")
    return
end

-- 1. 解析命令列參數
local isInstall = false
local targetType = nil
local forwardArgs = {}

for _, arg in ipairs(rawArgs) do
    local lArg = string.lower(arg)
    if lArg == "install" or lArg == "--install" or lArg == "-i" then
        isInstall = true
    elseif lArg == "cli" or lArg == "fcc-cli" or lArg == "fcc_cli" or lArg == "fcc-cli.lua" or lArg == "console" or lArg == "cmd" then
        targetType = "cli"
    elseif lArg == "fcc" or lArg == "fcc.lua" or lArg == "core" then
        targetType = "fcc"
    elseif lArg == "display" or lArg == "display.lua" or lArg == "screen" then
        targetType = "display"
    elseif lArg == "run" or lArg == "run.lua" or lArg == "all" then
        targetType = "run"
    elseif lArg == "turtle" or lArg == "turtle_startup.lua" or lArg == "node" then
        targetType = "turtle"
    else
        table.insert(forwardArgs, arg)
    end
end

-- 2. 若無傳入特定目標，顯示階層式互動選單
if not targetType then
    term.clear()
    term.setCursorPos(1, 1)
    print("========================================")
    print("   CC-AeroAutoPilot Universal Booter    ")
    print("========================================")
    
    if not isInstall then
        print(" Select Action:")
        print("  [1] Install to Disk  (寫入磁碟 / 開機自啟)")
        print("  [2] Run Directly     (直接從 GitHub Release 執行)")
        print("========================================")
        write(" Enter Action [1-2] (Default 1): ")
        local actionChoice = read()
        actionChoice = actionChoice:gsub("^%s*(.-)%s*$", "%1")
        if actionChoice == "2" or actionChoice == "run" or actionChoice == "r" then
            isInstall = false
        else
            isInstall = true
        end
    end

    if isInstall then
        print("\n--- Install Component to Disk (startup.lua) ---")
        print("  [1] FCC Flight Core   (飛控大腦 - 20Hz 純計算後台)")
        print("  [2] FCC-CLI Console   (命令列終端 - 輸入 go 100 100 等)")
        print("  [3] Cockpit Display   (駕駛艙螢幕 - DirectGPU / 儀表板)")
        print("  [4] All-In-One Run    (單機一體化全功能飛控)")
        print("  [5] Turtle Node       (動力烏龜韌體 - 安裝後自動重啟)")
        print("----------------------------------------")
        write(" Enter Component [1-5] (Default 1): ")
        local sub = read():gsub("^%s*(.-)%s*$", "%1")
        if sub == "2" or sub == "cli" then targetType = "cli"
        elseif sub == "3" or sub == "display" then targetType = "display"
        elseif sub == "4" or sub == "run" then targetType = "run"
        elseif sub == "5" or sub == "turtle" then targetType = "turtle"
        else targetType = "fcc" end
    else
        print("\n--- Run Latest Component from Release ---")
        print("  [1] FCC-CLI Console   (命令列終端 - 即時儀表與指令)")
        print("  [2] FCC Flight Core   (飛控大腦 - 20Hz 純計算後台)")
        print("  [3] Cockpit Display   (駕駛艙螢幕 - DirectGPU / 儀表板)")
        print("  [4] All-In-One Run    (單機一體化全功能飛控)")
        print("----------------------------------------")
        write(" Enter Component [1-4] (Default 1): ")
        local sub = read():gsub("^%s*(.-)%s*$", "%1")
        if sub == "2" or sub == "fcc" then targetType = "fcc"
        elseif sub == "3" or sub == "display" then targetType = "display"
        elseif sub == "4" or sub == "run" then targetType = "run"
        else targetType = "cli" end
    end
end

-- 3. 設定資產對應表
local assetMap = {
    run     = { asset = "run.lua",            saveAs = "run.lua" },
    fcc     = { asset = "fcc.lua",            saveAs = "fcc.lua" },
    display = { asset = "display.lua",        saveAs = "display.lua" },
    turtle  = { asset = "turtle_startup.lua", saveAs = "startup.lua" },
    cli     = { asset = "fcc-cli.lua",        saveAs = "fcc-cli.lua" }
}

local targetConfig = assetMap[targetType] or assetMap.cli
local ASSET_NAME = targetConfig.asset
local SAVE_FILENAME = targetConfig.saveAs

print(string.format("\nTarget: %s (%s)", ASSET_NAME, isInstall and "INSTALL TO DISK" or "RUN LATEST"))

-- 4. 從 GitHub Releases 直接下載發布資產 (Direct Release Asset Download)
local releaseAssetUrl = string.format("https://github.com/%s/%s/releases/latest/download/%s", OWNER, REPO, ASSET_NAME)
local httpHeaders = {
    ["User-Agent"] = "ComputerCraft-Client",
    ["Cache-Control"] = "no-cache"
}

print(string.format("Connecting to GitHub Release: '%s'...", ASSET_NAME))

local code = nil
local downloadSource = "Release Direct"

-- 嘗試 1: 直接從 Release Asset 下載 (最快、免 API 限制)
local okRel, respRel = pcall(function() return http.get(releaseAssetUrl, httpHeaders) end)
if okRel and respRel then
    local resCode = respRel.getResponseCode()
    if resCode == 200 then
        code = respRel.readAll()
    end
    respRel.close()
end

-- 嘗試 2: 若直接下載失敗，透過 GitHub Releases API 獲取 browser_download_url
if not code or #code < 50 then
    local apiUrl = string.format("https://api.github.com/repos/%s/%s/releases/latest", OWNER, REPO)
    local okApi, respApi = pcall(function() return http.get(apiUrl, httpHeaders) end)
    if okApi and respApi then
        local apiDataRaw = respApi.readAll()
        respApi.close()
        local releaseData = textutils.unserializeJSON(apiDataRaw)
        if releaseData and releaseData.assets then
            for _, asset in ipairs(releaseData.assets) do
                if asset.name == ASSET_NAME and asset.browser_download_url then
                    local okAsset, respAsset = pcall(function() return http.get(asset.browser_download_url, httpHeaders) end)
                    if okAsset and respAsset then
                        code = respAsset.readAll()
                        respAsset.close()
                        downloadSource = "Release API (" .. (releaseData.tag_name or "latest") .. ")"
                        break
                    end
                end
            end
        end
    end
end

-- 嘗試 3: 備援降級 (Raw GitHub)
if not code or #code < 50 then
    print("Notice: Release asset unreachable, falling back to Raw GitHub branch...")
    local rawUrl = string.format("https://raw.githubusercontent.com/%s/%s/%s/%s", OWNER, REPO, BRANCH, ASSET_NAME)
    local okRaw, respRaw = pcall(function() return http.get(rawUrl, httpHeaders) end)
    if okRaw and respRaw then
        code = respRaw.readAll()
        respRaw.close()
        downloadSource = "Raw GitHub (" .. BRANCH .. ")"
    end
end

if not code or #code < 50 then
    printError("Error: Failed to download asset '" .. ASSET_NAME .. "'. Check internet connection.")
    return
end

print(string.format("Download completed via [%s] (%d bytes).", downloadSource, #code))

-- 5. 儲存至本機檔案 (若指定 install 或 target 為 turtle)
if isInstall or targetType == "turtle" then
    -- 儲存為特定程式檔名
    if SAVE_FILENAME ~= "startup.lua" then
        local f = fs.open(SAVE_FILENAME, "w")
        if f then
            f.write(code)
            f.close()
            print(string.format("[OK] Saved file to '%s'", SAVE_FILENAME))
        end
    end

    -- 儲存為開機自啟 startup.lua
    local fStart = fs.open("startup.lua", "w")
    if fStart then
        fStart.write(code)
        fStart.close()
        print(string.format("[OK] Successfully installed '%s' as 'startup.lua'!", ASSET_NAME))
    else
        printError("Error: Failed to write to startup.lua")
    end

    if targetType == "turtle" then
        print("\nRebooting turtle in 1 second...")
        sleep(1)
        os.reboot()
        return
    end

    print("\nInstallation finished! Launching program now...")
    sleep(0.5)
end

-- 6. 載入並執行程式碼
local unpacker = table.unpack or unpack

local fn, err
if setfenv then
    fn, err = (loadstring or load)(code, ASSET_NAME)
    if fn then setfenv(fn, _ENV or getfenv()) end
else
    fn, err = load(code, ASSET_NAME, "t", _ENV or _G)
end

if not fn then
    printError("Syntax error in downloaded code: " .. tostring(err))
    return
end

local success, runErr = pcall(fn, unpacker(forwardArgs))
if not success then
    printError("Runtime error: " .. tostring(runErr))
end
