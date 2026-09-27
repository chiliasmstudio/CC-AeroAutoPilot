--[[
    fcc-cli.lua — VTOL Avionics Flight Control Command Line Interface (CLI)
    Version: v4.0.0
    
    支援單行指令與互動式 REPL 命令列終端，可遠端操控 FCC 飛控與動力烏龜。

    使用範例:
      fcc-cli go 100 200        -- 設定航點至 (100, 200)，容許半徑 20m
      fcc-cli go 100 200 15     -- 設定航點至 (100, 200)，容許半徑 15m
      fcc-cli alt 250           -- 設定目標高度 250m
      fcc-cli alt +50           -- 目標高度增加 50m
      fcc-cli calib             -- 啟動自動起飛校準
      fcc-cli hold              -- 啟動高度維持模式
      fcc-cli stop              -- 緊急停機
      fcc-cli hdg 90            -- 鎖定航向 90 度 (正東)
      fcc-cli update turtles    -- 一鍵 OTA 遠端更新所有動力烏龜程式碼
      fcc-cli reboot turtles    -- 遠端重啟所有動力烏龜
      fcc-cli status            -- 顯示當前即時飛控與導航遙測狀態
      fcc-cli watch             -- 即時動態監控儀表 (按 Q 退出)
      fcc-cli                   -- 進入互動式命令列終端 (REPL)
--]]

local VERSION = "v4.0.0"
local rawArgs = { ... }

-- 1. 初始化數據機
local modems = {}
for _, side in ipairs(peripheral.getNames()) do
    if peripheral.getType(side) == "modem" then
        local m = peripheral.wrap(side)
        if m then
            pcall(function() m.open(102) end) -- 監聽 FCC 遙測
            table.insert(modems, m)
        end
    end
end

if #modems == 0 then
    printError("Error: No Modem attached! Please attach a Wired or Wireless Modem.")
    return
end

-- 2. 通訊與指令廣播工具函數
local function sendFccCmd(cmdTable)
    for _, m in ipairs(modems) do
        pcall(function() m.transmit(103, 102, cmdTable) end)
    end
end

local function sendTurtleBroadcast(payload)
    for _, m in ipairs(modems) do
        pcall(function() m.transmit(100, 102, payload) end)
    end
end

local function getLatestTelemetry(timeout)
    timeout = timeout or 0.8
    local timerId = os.startTimer(timeout)
    while true do
        local eventData = { os.pullEvent() }
        local event = eventData[1]
        if event == "timer" and eventData[2] == timerId then
            return nil
        elseif event == "modem_message" then
            local side, ch, replyCh, msg = eventData[2], eventData[3], eventData[4], eventData[5]
            if ch == 102 and type(msg) == "table" and msg.type == "FCC_TELEMETRY" then
                return msg
            end
        end
    end
end

local function printStatus(tele)
    if not tele then
        tele = getLatestTelemetry(0.8)
    end
    if not tele then
        print(colors.yellow and "\27[33m" or "" .. "WARNING: No telemetry response from FCC (Channel 102). Is FCC running?")
        return
    end

    print("========================================")
    print(string.format("  VTOL Avionics Telemetry [%s]", tele.ver or VERSION))
    print("========================================")
    print(string.format(" Status   : %s", tele.statusMsg or "N/A"))
    print(string.format(" Mode     : %-10s  Alt: %.1fm (Tgt: %.1fm)", tele.mode or "IDLE", tele.currentAlt or 0, tele.targetAlt or 0))
    print(string.format(" V.Speed  : %+.2fm/s   Base Thrust: %.2f / 15.0", tele.vspeed or 0, tele.baseThrottle or 0))
    print(string.format(" Attitude : Pitch: %+3.1f*   Roll: %+3.1f*", tele.pitch or 0, tele.roll or 0))
    
    local nav = tele.nav or {}
    print(string.format(" Heading  : %03d* (Hold: %s, Tgt: %03d*)", math.floor(nav.yaw or 0), nav.headingHold and "ON" or "OFF", math.floor(nav.targetHeading or 0)))
    if nav.x and nav.z then
        print(string.format(" Nav Pos  : X:%.0f Y:%.0f Z:%.0f (Spd: %.1fm/s, Src: %s)", nav.x, nav.y or tele.currentAlt or 0, nav.z, nav.speed or 0, nav.source or "INS"))
        if nav.wpActive and nav.targetX and nav.targetZ then
            local dx = nav.targetX - nav.x
            local dz = nav.targetZ - nav.z
            local dist = math.sqrt(dx*dx + dz*dz)
            print(string.format(" Waypoint : -> (%d, %d) Dist: %.1fm (Accept R: %.0fm)", math.floor(nav.targetX), math.floor(nav.targetZ), dist, nav.arrivalRadius or 20))
        end
    else
        print(" Nav Pos  : No Coordinate Sensor (INS/GPS/NavTable)")
    end

    local qH = tele.quadHealth or {}
    local function fmtQ(r)
        local q = qH[r]
        if q then return string.format("%s(%d/%d)", r, q.online, q.total) end
        return r .. "(?)"
    end
    print(string.format(" Lift Eng : %s %s %s %s", fmtQ("FL"), fmtQ("FR"), fmtQ("BL"), fmtQ("BR")))
    print(string.format(" Horiz Eng: %s %s %s %s", fmtQ("FWD"), fmtQ("BWD"), fmtQ("LEFT"), fmtQ("RIGHT")))
    print("========================================")
end

local function printHelp()
    print("========================================")
    print("  FCC-CLI Command Reference")
    print("========================================")
    print(" 1. 導航與航點控制:")
    print("    go <x> <z> [r]     - 自動導航至目標座標 (預設半徑 20m)")
    print("    wp <x> <z> [r]     - go 指令別名")
    print("    cancel / stopnav   - 取消當前航點自動導航")
    print(" 2. 高度與升力控制:")
    print("    alt <altitude>     - 設定目標高度 (例: alt 200, alt +50, alt -10)")
    print("    hold               - 啟動高度維持模式 (HOLD_ALT)")
    print("    calib              - 啟動自動起飛校準 (CALIBRATE)")
    print("    lock               - 鎖定當前高度為目標高度")
    print("    stop               - 緊急關閉全部發動機 (IDLE)")
    print("    base <throttle>    - 調整基準油門 (例: base 8.5, base +0.5)")
    print(" 3. 航向與推進控制:")
    print("    hdg <angle>        - 鎖定航向角度 0~359 (例: hdg 90 為正東)")
    print("    synchdg            - 將目標航向同步至當前飛船船頭朝向")
    print("    togglehdg          - 切換航向鎖定開關 (ON / OFF)")
    print("    fwd <throttle>     - 手動前進/後退推力 (-15.0 ~ 15.0)")
    print("    turn <throttle>    - 手動轉向推力 (正為右轉，負為左轉)")
    print(" 4. 烏龜韌體與維護:")
    print("    update turtles     - 一鍵 OTA 遠端更新所有動力烏龜韌體")
    print("    reboot turtles     - 遠端重啟所有動力烏龜")
    print("    rescan             - 重新掃描硬體與外設")
    print(" 5. 監控與終端:")
    print("    status             - 顯示當前完整飛控遙測報告")
    print("    watch / monitor    - 即時動態儀表監控 (按 Q 退出)")
    print("    help               - 顯示此說明文件")
    print("    exit / quit        - 退出 CLI")
    print("========================================")
end

local function watchTelemetry()
    term.clear()
    print("Starting live telemetry watch... Press 'Q' or 'Ctrl+T' to stop.")
    sleep(0.5)
    while true do
        term.setCursorPos(1, 1)
        local tele = getLatestTelemetry(0.4)
        if tele then
            printStatus(tele)
        else
            print("Waiting for FCC telemetry broadcast (Channel 102)...")
        end
        
        -- 檢查鍵盤輸入
        local timerId = os.startTimer(0.2)
        local breakLoop = false
        while true do
            local eventData = { os.pullEvent() }
            if eventData[1] == "timer" and eventData[2] == timerId then
                break
            elseif eventData[1] == "char" and (eventData[2] == "q" or eventData[2] == "Q") then
                breakLoop = true
                break
            elseif eventData[1] == "key" and eventData[2] == keys.q then
                breakLoop = true
                break
            elseif eventData[1] == "terminate" then
                breakLoop = true
                break
            end
        end
        if breakLoop then
            print("\nExiting telemetry watch.")
            break
        end
    end
end

-- 3. 指令執行器核心
local function executeCommand(args)
    if #args == 0 then return true end
    local cmd = string.lower(args[1])

    -- 航點導航: go <x> <z> [radius]
    if cmd == "go" or cmd == "wp" or cmd == "waypoint" or cmd == "goto" then
        local x = tonumber(args[2])
        local z = tonumber(args[3])
        local r = tonumber(args[4]) or 20.0
        if not x or not z then
            printError("Usage: go <x> <z> [radius]")
            print("Example: go 100 200 (default radius 20m)")
            return false
        end
        sendFccCmd({ cmd = "SET_WAYPOINT", x = x, z = z, radius = r })
        print(string.format("[OK] Waypoint set to (%d, %d) with arrival radius %.0fm", x, z, r))
        return true

    -- 取消航點: cancel
    elseif cmd == "cancel" or cmd == "stopnav" or cmd == "stopwp" then
        sendFccCmd({ cmd = "CANCEL_WAYPOINT" })
        print("[OK] Waypoint navigation cancelled.")
        return true

    -- 目標高度: alt <targetAlt>
    elseif cmd == "alt" or cmd == "altitude" or cmd == "tgt" then
        local valStr = args[2]
        if not valStr then
            printError("Usage: alt <altitude> (e.g. alt 200, alt +50, alt -10)")
            return false
        end
        if valStr:sub(1, 1) == "+" or valStr:sub(1, 1) == "-" then
            local delta = tonumber(valStr)
            if delta then
                sendFccCmd({ cmd = "ADJ_ALT", delta = delta })
                print(string.format("[OK] Adjusting target altitude by %+dm", delta))
                return true
            end
        else
            local alt = tonumber(valStr)
            if alt then
                sendFccCmd({ cmd = "SET_ALT", alt = alt })
                print(string.format("[OK] Target altitude set to %.1fm", alt))
                return true
            end
        end
        printError("Invalid altitude value: " .. valStr)
        return false

    -- 高度維持: hold
    elseif cmd == "hold" or cmd == "hold_alt" or cmd == "hover" then
        sendFccCmd({ cmd = "HOLD_ALT" })
        print("[OK] Altitude Hold (HOLD_ALT) mode commanded.")
        return true

    -- 起飛校準: calib
    elseif cmd == "calib" or cmd == "calibrate" or cmd == "takeoff" then
        sendFccCmd({ cmd = "CALIB" })
        print("[OK] Liftoff calibration (CALIBRATE) commanded.")
        return true

    -- 鎖定當前高度: lock
    elseif cmd == "lock" or cmd == "lockalt" then
        sendFccCmd({ cmd = "LOCK_ALT" })
        print("[OK] Locked current altitude as target.")
        return true

    -- 緊急停機: stop
    elseif cmd == "stop" or cmd == "idle" or cmd == "off" then
        sendFccCmd({ cmd = "STOP" })
        print("[OK] STOP commanded. All engines set to IDLE (0).")
        return true

    -- 航向鎖定: hdg <heading>
    elseif cmd == "hdg" or cmd == "heading" or cmd == "yaw" then
        local val = tonumber(args[2])
        if not val then
            printError("Usage: hdg <0-359> (e.g. hdg 90 for East)")
            return false
        end
        local hdg = (math.floor(val) % 360 + 360) % 360
        sendFccCmd({ cmd = "SET_HEADING", hdg = hdg })
        print(string.format("[OK] Target heading locked to %03d*", hdg))
        return true

    -- 航向同步: synchdg
    elseif cmd == "synchdg" or cmd == "sync_heading" or cmd == "sync" then
        sendFccCmd({ cmd = "SYNC_HEADING" })
        print("[OK] Target heading synced to current ship heading.")
        return true

    -- 航向鎖定開關: togglehdg
    elseif cmd == "togglehdg" or cmd == "toggle_heading" then
        sendFccCmd({ cmd = "TOGGLE_HEADING" })
        print("[OK] Heading hold toggled.")
        return true

    -- 前進油門: fwd <throttle>
    elseif cmd == "fwd" or cmd == "forward" or cmd == "thrust" then
        local val = tonumber(args[2])
        if not val then
            printError("Usage: fwd <val> (-15.0 ~ 15.0)")
            return false
        end
        sendFccCmd({ cmd = "SET_FWD", val = val })
        print(string.format("[OK] Forward thrust set to %.1f", val))
        return true

    -- 轉向推力: turn <throttle>
    elseif cmd == "turn" or cmd == "steer" then
        local val = tonumber(args[2])
        if not val then
            printError("Usage: turn <val> (-15.0 to +15.0, positive=right, negative=left)")
            return false
        end
        sendFccCmd({ cmd = "SET_TURN", val = val })
        print(string.format("[OK] Turn thrust set to %.1f", val))
        return true

    -- 基準油門: base <throttle>
    elseif cmd == "base" or cmd == "basethrottle" then
        local valStr = args[2]
        if not valStr then
            printError("Usage: base <val> (e.g. base 8.5, base +0.5, base -0.5)")
            return false
        end
        if valStr:sub(1, 1) == "+" or valStr:sub(1, 1) == "-" then
            local delta = tonumber(valStr)
            if delta then
                sendFccCmd({ cmd = "ADJ_BASE", delta = delta })
                print(string.format("[OK] Adjusting base throttle by %+0.2f", delta))
                return true
            end
        else
            local delta = tonumber(valStr)
            if delta then
                sendFccCmd({ cmd = "ADJ_BASE", delta = delta })
                print(string.format("[OK] Adjusting base throttle by %0.2f", delta))
                return true
            end
        end
        printError("Invalid base throttle value: " .. valStr)
        return false

    -- 烏龜遠端重啟: reboot turtles / reboot all
    elseif cmd == "reboot" or cmd == "restart" then
        local sub = args[2] and string.lower(args[2]) or "turtles"
        if sub == "turtles" or sub == "turtle" or sub == "all" or sub == "nodes" then
            print("Broadcasting REBOOT command to all turtles on Channel 100...")
            sendTurtleBroadcast({ cmd = "REBOOT", type = "REBOOT", timestamp = os.epoch("utc") })
            sendFccCmd({ cmd = "REBOOT_TURTLES" })
            print("[OK] Reboot signal sent to all engine turtles.")
            return true
        end
        printError("Usage: reboot turtles")
        return false

    -- 烏龜遠端 OTA 更新: update turtles / update all
    elseif cmd == "update" or cmd == "ota" or cmd == "upgrade" then
        local sub = args[2] and string.lower(args[2]) or "turtles"
        if sub == "turtles" or sub == "turtle" or sub == "all" or sub == "nodes" then
            print("Broadcasting OTA UPDATE command to all turtles on Channel 100...")
            sendTurtleBroadcast({ cmd = "UPDATE", type = "OTA_UPDATE", timestamp = os.epoch("utc") })
            sendFccCmd({ cmd = "UPDATE_TURTLES" })
            print("[OK] OTA update signal sent! Turtles are downloading latest firmware and rebooting...")
            return true
        end
        printError("Usage: update turtles")
        return false

    -- 硬體重掃: rescan
    elseif cmd == "rescan" or cmd == "scan" or cmd == "hw" then
        sendFccCmd({ cmd = "RESCAN_HW" })
        print("[OK] Hardware rescan commanded.")
        return true

    -- 遙測狀態: status
    elseif cmd == "status" or cmd == "info" or cmd == "telemetry" or cmd == "tele" then
        printStatus()
        return true

    -- 動態監控: watch / monitor
    elseif cmd == "watch" or cmd == "monitor" or cmd == "top" then
        watchTelemetry()
        return true

    -- 說明文檔: help
    elseif cmd == "help" or cmd == "?" or cmd == "man" then
        printHelp()
        return true

    -- 退出: exit / quit
    elseif cmd == "exit" or cmd == "quit" or cmd == "q" then
        return false

    else
        printError("Unknown command: '" .. cmd .. "'. Type 'help' for available commands.")
        return true
    end
end

-- 4. 模式判定：單行指令執行 vs 互動式 REPL 終端
if #rawArgs > 0 then
    -- 單行 CLI 模式
    executeCommand(rawArgs)
else
    -- 互動式 REPL 終端
    term.clear()
    term.setCursorPos(1, 1)
    print("========================================")
    print(string.format("  VTOL Avionics FCC-CLI [%s]", VERSION))
    print("  Type 'help' for commands, 'exit' to quit.")
    print("========================================")

    while true do
        write("FCC> ")
        local input = read()
        if not input then break end
        input = input:gsub("^%s*(.-)%s*$", "%1")
        if input ~= "" then
            local tokens = {}
            for token in input:gmatch("%S+") do
                table.insert(tokens, token)
            end
            local keepRunning = executeCommand(tokens)
            if not keepRunning then
                print("Goodbye.")
                break
            end
        end
    end
end
