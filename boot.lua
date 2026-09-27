--[[
    boot.lua — CC-AeroAutoPilot Universal Release Launcher & Auto-Updater
    ====================================================================
    自動從 GitHub Releases 下載最新版本並執行，支援 API 速率限制自動備援降級。

    使用方式:
      boot.lua                  (啟動最新版 一體化飛控 run.lua)
      boot.lua fcc              (啟動最新版 分散式飛控大腦 fcc.lua)
      boot.lua display [driver] (啟動最新版 駕駛艙螢幕 display.lua)
      boot.lua turtle           (安裝最新版 四軸動力烏龜 startup.lua)
      boot.lua install [target] (下載並儲存至本機磁碟，設為開機自啟)
--]]

local rawArgs = { ... }
local OWNER = "chiliasmstudio"
local REPO = "CC-AeroAutoPilot"
local BRANCH = "main"

if not http then
    printError("Error: HTTP API is disabled in ComputerCraft config.")
    return
end

-- 解析參數
local isInstall = false
local targetType = "run"
local forwardArgs = {}

for _, arg in ipairs(rawArgs) do
    local lArg = string.lower(arg)
    if lArg == "install" or lArg == "--install" or lArg == "-i" then
        isInstall = true
    elseif lArg == "fcc" or lArg == "fcc.lua" then
        targetType = "fcc"
    elseif lArg == "display" or lArg == "display.lua" or lArg == "screen" then
        targetType = "display"
    elseif lArg == "run" or lArg == "run.lua" or lArg == "all" then
        targetType = "run"
    elseif lArg == "turtle" or lArg == "turtle_startup.lua" then
        targetType = "turtle"
    else
        table.insert(forwardArgs, arg)
    end
end

local assetMap = {
    run     = { asset = "run.lua",            saveAs = "run.lua" },
    fcc     = { asset = "fcc.lua",            saveAs = "fcc.lua" },
    display = { asset = "display.lua",        saveAs = "display.lua" },
    turtle  = { asset = "turtle_startup.lua", saveAs = "startup.lua" }
}

local targetConfig = assetMap[targetType] or assetMap.run
local ASSET_NAME = targetConfig.asset
local SAVE_FILENAME = targetConfig.saveAs

local apiHeaders = {
    ["User-Agent"] = "ComputerCraft-Client",
    ["Accept"] = "application/vnd.github.v3+json",
    ["Cache-Control"] = "no-cache"
}

term.clear()
term.setCursorPos(1, 1)
print("========================================")
print("  CC-AeroAutoPilot Release Launcher")
print("========================================")
print(string.format("Target: %s (%s)", ASSET_NAME, isInstall and "INSTALL TO DISK" or "RUN LATEST"))
print("Checking GitHub for the latest release...")

local downloadUrl = nil
local releaseTag = "latest"

-- 1. 嘗試由 GitHub Releases API 獲取下載連結
local apiUrl = ("https://api.github.com/repos/%s/%s/releases/latest"):format(OWNER, REPO)
local okApi, releaseResponse = pcall(function() return http.get(apiUrl, apiHeaders) end)

if okApi and releaseResponse then
    local releaseDataRaw = releaseResponse.readAll()
    releaseResponse.close()
    local releaseData = textutils.unserializeJSON(releaseDataRaw)
    if releaseData and releaseData.assets then
        releaseTag = releaseData.tag_name or "latest"
        for _, asset in ipairs(releaseData.assets) do
            if asset.name == ASSET_NAME then
                downloadUrl = asset.browser_download_url
                break
            end
        end
    end
end

-- 2. 若 API 受到 Rate Limit 限制或找不到 Asset，自動降級使用 Raw GitHub
if not downloadUrl then
    print("Notice: API unlisted or rate-limited. Falling back to Raw GitHub...")
    downloadUrl = ("https://raw.githubusercontent.com/%s/%s/%s/%s"):format(OWNER, REPO, BRANCH, ASSET_NAME)
end

print(string.format("Downloading '%s' [%s]...", ASSET_NAME, releaseTag))

local fileResponse = http.get(downloadUrl, {
    ["User-Agent"] = "ComputerCraft-Client",
    ["Cache-Control"] = "no-cache"
})

if not fileResponse then
    printError("Error: Failed to download asset from " .. downloadUrl)
    return
end

local code = fileResponse.readAll()
fileResponse.close()

if not code or #code < 50 then
    printError("Error: Downloaded file is empty or corrupted.")
    return
end

-- 儲存至本機檔案 (若指定 install 或 target 為 turtle)
if isInstall or targetType == "turtle" then
    local f = fs.open(SAVE_FILENAME, "w")
    if f then
        f.write(code)
        f.close()
        print(string.format("Successfully installed to '%s'!", SAVE_FILENAME))
        if targetType == "turtle" then
            print("Rebooting turtle in 1 second...")
            sleep(1)
            os.reboot()
            return
        end
    else
        printError("Error: Failed to write to " .. SAVE_FILENAME)
    end
end

print("Executing " .. ASSET_NAME .. "...")
sleep(0.3)

-- 載入並執行程式碼
local loader = load or loadstring
local unpacker = table.unpack or unpack

local fn, err = loader(code, ASSET_NAME)
if not fn then
    printError("Syntax error in downloaded file: " .. tostring(err))
    return
end

local success, runErr = pcall(fn, unpacker(forwardArgs))
if not success then
    printError("Runtime error: " .. tostring(runErr))
end
