--[[
    fcc-cli.lua — Standalone Command Line Interface & Console for VTOL Avionics
    Version: v4.0.4
    
    特點:
    - 完全獨立於 FCC 飛控電腦，可在任何終端機 / Pocket Computer 上運行
    - 上方視窗即時動態顯示主要飛控資訊 (高度、垂直速度、航向、座標、航點、發動機健康度)
    - 下方命令視窗提供即時指令輸入 (CMD> )，兩者並行運作互不干擾
    - 支援單行快捷指令 (例如: fcc-cli go 100 100 / fcc-cli alt 250 / fcc-cli update turtles)

    使用範例:
      fcc-cli                   -- 啟動即時資訊儀表 ＆ 互動命令終端 (REPL)
      fcc-cli go 100 200        -- 設定航點至 (100, 200)，容許半徑 20m
      fcc-cli alt 250           -- 設定目標高度 250m
      fcc-cli alt +50           -- 目標高度增加 50m
      fcc-cli calib             -- 啟動自動起飛校準
      fcc-cli hold              -- 啟動高度維持模式
      fcc-cli stop              -- 緊急停機
      fcc-cli hdg 90            -- 鎖定航向 90 度 (正東)
      fcc-cli update turtles    -- 一鍵 OTA 遠端更新所有動力烏龜程式碼
      fcc-cli reboot turtles    -- 遠端重啟所有動力烏龜
--]]

local VERSION = "v4.0.4"
local rawArgs = { ... }

-- 1. 初始化數據機
local modems = {}
for _, side in ipairs(peripheral.getNames()) do
    if peripheral.getType(side) == "modem" then
        local m = peripheral.wrap(side)
        if m then
            pcall(function() m.open(102) end) -- 監聽 FCC 遙測廣播
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

-- 3. 指令執行邏輯
local function executeCommand(args, out)
    out = out or {
        print = function(s) print(s) end,
        printError = function(s) printError(s) end,
        setTextColor = function(c) if term.setTextColor then term.setTextColor(c) end end
    }

    if #args == 0 then return true end
    local cmd = string.lower(args[1])

    -- 航點導航: go <x> <z> [radius]
    if cmd == "go" or cmd == "wp" or cmd == "waypoint" or cmd == "goto" then
        local x = tonumber(args[2])
        local z = tonumber(args[3])
        local r = tonumber(args[4]) or 20.0
        if not x or not z then
            out.printError("Usage: go <x> <z> [radius]")
            out.print("Example: go 100 200 (default radius 20m)")
            return true
        end
        sendFccCmd({ cmd = "SET_WAYPOINT", x = x, z = z, radius = r })
        out.print(string.format("[OK] Waypoint set to (%d, %d) with arrival radius %.0fm", x, z, r))
        return true

    -- 取消航點: cancel
    elseif cmd == "cancel" or cmd == "stopnav" or cmd == "stopwp" then
        sendFccCmd({ cmd = "CANCEL_WAYPOINT" })
        out.print("[OK] Waypoint navigation cancelled.")
        return true

    -- 目標高度: alt <targetAlt>
    elseif cmd == "alt" or cmd == "altitude" or cmd == "tgt" then
        local valStr = args[2]
        if not valStr then
            out.printError("Usage: alt <altitude> (e.g. alt 200, alt +50, alt -10)")
            return true
        end
        if valStr:sub(1, 1) == "+" or valStr:sub(1, 1) == "-" then
            local delta = tonumber(valStr)
            if delta then
                sendFccCmd({ cmd = "ADJ_ALT", delta = delta })
                out.print(string.format("[OK] Adjusting target altitude by %+dm", delta))
                return true
            end
        else
            local alt = tonumber(valStr)
            if alt then
                sendFccCmd({ cmd = "SET_ALT", alt = alt })
                out.print(string.format("[OK] Target altitude set to %.1fm", alt))
                return true
            end
        end
        out.printError("Invalid altitude value: " .. valStr)
        return true

    -- 高度維持: hold
    elseif cmd == "hold" or cmd == "hold_alt" or cmd == "hover" then
        sendFccCmd({ cmd = "HOLD_ALT" })
        out.print("[OK] Altitude Hold (HOLD_ALT) mode commanded.")
        return true

    -- 起飛校準: calib
    elseif cmd == "calib" or cmd == "calibrate" or cmd == "takeoff" then
        sendFccCmd({ cmd = "CALIB" })
        out.print("[OK] Liftoff calibration (CALIBRATE) commanded.")
        return true

    -- 鎖定當前高度: lock
    elseif cmd == "lock" or cmd == "lockalt" then
        sendFccCmd({ cmd = "LOCK_ALT" })
        out.print("[OK] Locked current altitude as target.")
        return true

    -- 緊急停機: stop
    elseif cmd == "stop" or cmd == "idle" or cmd == "off" then
        sendFccCmd({ cmd = "STOP" })
        out.print("[OK] STOP commanded. All engines set to IDLE (0).")
        return true

    -- 航向鎖定: hdg <heading>
    elseif cmd == "hdg" or cmd == "heading" or cmd == "yaw" then
        local val = tonumber(args[2])
        if not val then
            out.printError("Usage: hdg <0-359> (e.g. hdg 90 for East)")
            return true
        end
        local hdg = (math.floor(val) % 360 + 360) % 360
        sendFccCmd({ cmd = "SET_HEADING", hdg = hdg })
        out.print(string.format("[OK] Target heading locked to %03d*", hdg))
        return true

    -- 航向同步: synchdg
    elseif cmd == "synchdg" or cmd == "sync_heading" or cmd == "sync" then
        sendFccCmd({ cmd = "SYNC_HEADING" })
        out.print("[OK] Target heading synced to current ship heading.")
        return true

    -- 航向鎖定開關: togglehdg
    elseif cmd == "togglehdg" or cmd == "toggle_heading" then
        sendFccCmd({ cmd = "TOGGLE_HEADING" })
        out.print("[OK] Heading hold toggled.")
        return true

    -- 前進油門: fwd <throttle>
    elseif cmd == "fwd" or cmd == "forward" or cmd == "thrust" then
        local val = tonumber(args[2])
        if not val then
            out.printError("Usage: fwd <val> (-15.0 ~ 15.0)")
            return true
        end
        sendFccCmd({ cmd = "SET_FWD", val = val })
        out.print(string.format("[OK] Forward thrust set to %.1f", val))
        return true

    -- 轉向推力: turn <throttle>
    elseif cmd == "turn" or cmd == "steer" then
        local val = tonumber(args[2])
        if not val then
            out.printError("Usage: turn <val> (-15.0 to +15.0, positive=right, negative=left)")
            return true
        end
        sendFccCmd({ cmd = "SET_TURN", val = val })
        out.print(string.format("[OK] Turn thrust set to %.1f", val))
        return true

    -- 基準油門: base <throttle>
    elseif cmd == "base" or cmd == "basethrottle" then
        local valStr = args[2]
        if not valStr then
            out.printError("Usage: base <val> (e.g. base 8.5, base +0.5, base -0.5)")
            return true
        end
        local delta = tonumber(valStr)
        if delta then
            sendFccCmd({ cmd = "ADJ_BASE", delta = delta })
            out.print(string.format("[OK] Base throttle adjustment: %0.2f", delta))
            return true
        end
        out.printError("Invalid base throttle value: " .. valStr)
        return true

    -- 烏龜遠端重啟: reboot turtles
    elseif cmd == "reboot" or cmd == "restart" then
        local sub = args[2] and string.lower(args[2]) or "turtles"
        if sub == "turtles" or sub == "turtle" or sub == "all" or sub == "nodes" then
            out.print("Broadcasting REBOOT command to all turtles on Channel 100...")
            sendTurtleBroadcast({ cmd = "REBOOT", type = "REBOOT", timestamp = os.epoch("utc") })
            sendFccCmd({ cmd = "REBOOT_TURTLES" })
            out.print("[OK] Reboot signal sent to all engine turtles.")
            return true
        end
        out.printError("Usage: reboot turtles")
        return true

    -- 烏龜遠端 OTA 更新: update turtles
    elseif cmd == "update" or cmd == "ota" or cmd == "upgrade" then
        local sub = args[2] and string.lower(args[2]) or "turtles"
        if sub == "turtles" or sub == "turtle" or sub == "all" or sub == "nodes" then
            out.print("Broadcasting OTA UPDATE command to all turtles on Channel 100...")
            sendTurtleBroadcast({ cmd = "UPDATE", type = "OTA_UPDATE", timestamp = os.epoch("utc") })
            sendFccCmd({ cmd = "UPDATE_TURTLES" })
            out.print("[OK] OTA update signal sent! Turtles are updating firmware and rebooting...")
            return true
        end
        out.printError("Usage: update turtles")
        return true

    -- 硬體重掃: rescan
    elseif cmd == "rescan" or cmd == "scan" or cmd == "hw" then
        sendFccCmd({ cmd = "RESCAN_HW" })
        out.print("[OK] Hardware rescan commanded.")
        return true

    -- 說明文件: help
    elseif cmd == "help" or cmd == "?" or cmd == "man" then
        out.print("=== Available Commands ===")
        out.print(" go <x> <z> [r]   - Navigate to coordinates (radius default 20m)")
        out.print(" cancel           - Cancel waypoint navigation")
        out.print(" alt <alt>        - Set target altitude (e.g. alt 200, alt +50)")
        out.print(" hold / calib     - Hold altitude / Liftoff auto calibration")
        out.print(" lock / stop      - Lock current altitude / Stop engines (IDLE)")
        out.print(" hdg <0-359>      - Lock target heading (e.g. hdg 90)")
        out.print(" synchdg          - Sync target heading to current yaw")
        out.print(" fwd / turn <val> - Forward/turn thrust (-15.0 ~ 15.0)")
        out.print(" update turtles   - One-click OTA firmware update all turtles")
        out.print(" reboot turtles   - Remote reboot all engine turtles")
        out.print(" clear            - Clear command console window")
        out.print(" exit / quit      - Exit FCC-CLI console")
        return true

    -- 清除命令視窗: clear
    elseif cmd == "clear" or cmd == "cls" then
        if out.clear then out.clear() end
        return true

    -- 退出: exit / quit
    elseif cmd == "exit" or cmd == "quit" or cmd == "q" then
        return false

    else
        out.printError("Unknown command: '" .. cmd .. "'. Type 'help' for available commands.")
        return true
    end
end

-- ========================================================
-- 模式 A: 單行指令直接執行 (Single Command Mode)
-- ========================================================
if #rawArgs > 0 then
    executeCommand(rawArgs)
    return
end

-- ========================================================
-- 模式 B: 雙視窗即時資訊 ＆ 互動命令終端 (Split-Screen REPL)
-- ========================================================
local termW, termH = term.getSize()
local dashHeight = math.max(8, math.min(10, math.floor(termH * 0.50)))
local cmdHeight = termH - dashHeight

local topWin = window.create(term.current(), 1, 1, termW, dashHeight, true)
local cmdWin = window.create(term.current(), 1, dashHeight + 1, termW, cmdHeight, true)

local latestTele = nil

local function renderTelemetryDashboard()
    if not topWin then return end
    topWin.setVisible(false)
    topWin.setBackgroundColor(colors.black)
    topWin.clear()

    -- 1. 標題列
    topWin.setCursorPos(1, 1)
    topWin.setTextColor(colors.white)
    topWin.setBackgroundColor(colors.blue)
    local title = string.format(" VTOL FCC COMMAND CONSOLE [%s] ", VERSION)
    topWin.write(title .. string.rep(" ", termW - #title))
    topWin.setBackgroundColor(colors.black)

    if not latestTele then
        topWin.setCursorPos(1, 3)
        topWin.setTextColor(colors.yellow)
        topWin.write(" [WAITING] Listening for FCC Telemetry (Ch 102)...")
        topWin.setCursorPos(1, 5)
        topWin.setTextColor(colors.lightGray)
        topWin.write(" Type commands below (e.g. 'go 100 100', 'alt 200', 'help')")
        topWin.setCursorPos(1, dashHeight)
        topWin.setTextColor(colors.gray)
        topWin.write(string.rep("-", termW))
        topWin.setVisible(true)
        return
    end

    local tele = latestTele
    local nav = tele.nav or {}

    -- 2. 高度與垂直狀態 (Alt & Vertical)
    topWin.setCursorPos(1, 2)
    topWin.setTextColor(colors.lightBlue)
    topWin.write(" ALT: ")
    topWin.setTextColor(colors.white)
    topWin.write(string.format("%6.1fm", tele.currentAlt or 0))
    topWin.setTextColor(colors.lightGray)
    topWin.write(string.format(" (Tgt:%5.1fm)", tele.targetAlt or 0))
    topWin.setTextColor(colors.yellow)
    topWin.write(string.format(" | V.S:%+5.2fm/s", tele.vspeed or 0))
    topWin.setTextColor(colors.lightBlue)
    topWin.write(" | MOD: ")
    local mCol = (tele.mode == "HOLD_ALT") and colors.green or ((tele.mode == "CALIBRATING") and colors.orange or colors.red)
    topWin.setTextColor(mCol)
    topWin.write(string.format("%-8s", tele.mode or "IDLE"))

    -- 3. 油門與姿態 (Throttle & Attitude)
    topWin.setCursorPos(1, 3)
    topWin.setTextColor(colors.lightBlue)
    topWin.write(" THR: ")
    topWin.setTextColor(colors.lime)
    topWin.write(string.format("%4.2f/15.0", tele.baseThrottle or 0))
    topWin.setTextColor(colors.lightGray)
    topWin.write(string.format(" | Pitch:%+4.1f* Roll:%+4.1f*", tele.pitch or 0, tele.roll or 0))
    topWin.setTextColor(colors.lightBlue)
    topWin.write(" | Gimbal: ")
    topWin.setTextColor(tele.gimbalAvailable and colors.green or colors.gray)
    topWin.write(tele.gimbalAvailable and "ON " or "N/A")

    -- 4. 航向與空速 (Heading & Speed)
    topWin.setCursorPos(1, 4)
    topWin.setTextColor(colors.lightBlue)
    topWin.write(" HDG: ")
    topWin.setTextColor(colors.white)
    topWin.write(string.format("%03d*", math.floor(nav.yaw or 0)))
    topWin.setTextColor(nav.headingHold and colors.green or colors.gray)
    topWin.write(string.format(" [Hold:%s, Tgt:%03d*]", nav.headingHold and "ON" or "OFF", math.floor(nav.targetHeading or 0)))
    topWin.setTextColor(colors.yellow)
    topWin.write(string.format(" | Spd: %4.1fm/s", nav.speed or 0))

    -- 5. 導航座標 (Position)
    topWin.setCursorPos(1, 5)
    topWin.setTextColor(colors.lightBlue)
    topWin.write(" POS: ")
    if nav.x and nav.z then
        topWin.setTextColor(colors.white)
        topWin.write(string.format("X:%-5.0f Y:%-4.0f Z:%-5.0f (%s)", nav.x, nav.y or tele.currentAlt or 0, nav.z, nav.source or "INS"))
    else
        topWin.setTextColor(colors.gray)
        topWin.write("No Position Sensor (INS/GPS/NavTable)")
    end

    -- 6. 航點自駕狀態 (Waypoint)
    topWin.setCursorPos(1, 6)
    topWin.setTextColor(colors.lightBlue)
    topWin.write(" NAV: ")
    if nav.wpActive and nav.targetX and nav.targetZ and nav.x and nav.z then
        local dx = nav.targetX - nav.x
        local dz = nav.targetZ - nav.z
        local dist = math.sqrt(dx*dx + dz*dz)
        topWin.setTextColor(colors.lime)
        topWin.write(string.format("-> (%d,%d) Dist:%.0fm (R:%.0fm)", math.floor(nav.targetX), math.floor(nav.targetZ), dist, nav.arrivalRadius or 20))
    elseif nav.wpActive and nav.targetX and nav.targetZ then
        topWin.setTextColor(colors.lime)
        topWin.write(string.format("-> (%d,%d) [Searching GPS/INS...]", math.floor(nav.targetX), math.floor(nav.targetZ)))
    else
        topWin.setTextColor(colors.gray)
        topWin.write("No Waypoint Active (Type 'go x z')")
    end

    -- 7. 發動機烏龜節點健康度 (Engines)
    local qH = tele.quadHealth or {}
    local function fmtQ(r)
        local q = qH[r]
        if q then
            local col = q.online > 0 and colors.green or colors.red
            return r, string.format("%d/%d", q.online, q.total), col
        end
        return r, "?/?", colors.gray
    end
    topWin.setCursorPos(1, 7)
    topWin.setTextColor(colors.lightBlue)
    topWin.write(" ENG: ")
    for _, r in ipairs({"FL", "FR", "BL", "BR", "FWD"}) do
        local rName, rStat, rCol = fmtQ(r)
        topWin.setTextColor(colors.white)
        topWin.write(rName .. ":")
        topWin.setTextColor(rCol)
        topWin.write(rStat .. " ")
    end

    -- 8. 狀態訊息 (Status Msg)
    topWin.setCursorPos(1, 8)
    topWin.setTextColor(colors.lightBlue)
    topWin.write(" MSG: ")
    topWin.setTextColor(colors.cyan)
    topWin.write((tele.statusMsg or "System Ready"):sub(1, termW - 8))

    -- 分隔線
    topWin.setCursorPos(1, dashHeight)
    topWin.setTextColor(colors.gray)
    topWin.write(string.rep("-", termW))

    topWin.setVisible(true)
end

-- 4. 遙測廣播背景接收線程
local function telemetryLoop()
    renderTelemetryDashboard()
    while true do
        local eventData = { os.pullEvent() }
        local event = eventData[1]
        if event == "modem_message" then
            local side, ch, replyCh, msg = eventData[2], eventData[3], eventData[4], eventData[5]
            if ch == 102 and type(msg) == "table" and msg.type == "FCC_TELEMETRY" then
                latestTele = msg
                renderTelemetryDashboard()
            end
        elseif event == "term_resize" then
            termW, termH = term.getSize()
            dashHeight = math.max(8, math.min(10, math.floor(termH * 0.50)))
            cmdHeight = termH - dashHeight
            topWin.reposition(1, 1, termW, dashHeight)
            cmdWin.reposition(1, dashHeight + 1, termW, cmdHeight)
            renderTelemetryDashboard()
        end
    end
end

-- 5. 命令終端互動輸入線程
local function commandConsoleLoop()
    local winOut = {
        print = function(s)
            cmdWin.setTextColor(colors.white)
            cmdWin.write(tostring(s) .. "\n")
        end,
        printError = function(s)
            cmdWin.setTextColor(colors.red)
            cmdWin.write(tostring(s) .. "\n")
        end,
        setTextColor = function(c)
            cmdWin.setTextColor(c)
        end,
        clear = function()
            cmdWin.clear()
            cmdWin.setCursorPos(1, 1)
        end
    }

    cmdWin.clear()
    cmdWin.setCursorPos(1, 1)
    cmdWin.setTextColor(colors.lightGray)
    cmdWin.write("Type 'help' for commands, 'go x z', 'update turtles', 'exit'\n")

    local history = {}
    while true do
        cmdWin.setTextColor(colors.yellow)
        cmdWin.write("CMD> ")
        cmdWin.setTextColor(colors.white)

        local oldTerm = term.redirect(cmdWin)
        local input = read(nil, history)
        term.redirect(oldTerm)

        if not input then break end
        input = input:gsub("^%s*(.-)%s*$", "%1")
        if input ~= "" then
            table.insert(history, input)
            local tokens = {}
            for token in input:gmatch("%S+") do
                table.insert(tokens, token)
            end
            local keepRunning = executeCommand(tokens, winOut)
            if not keepRunning then
                cmdWin.setTextColor(colors.lightGray)
                cmdWin.write("Console Closed. Goodbye.\n")
                break
            end
        end
    end
end

term.clear()
parallel.waitForAny(telemetryLoop, commandConsoleLoop)
term.setCursorPos(1, termH)
