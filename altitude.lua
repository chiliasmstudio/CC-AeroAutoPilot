--[[
    Create: Avionics & CC: Tweaked
    純紅石高度保持與推力校準飛控系統 (Altitude Controller)
--]]

local PID = require("pid")

-- ========================================================
-- 1. 硬體檢測與周邊配置
-- ========================================================
local function findPeripheral(pType)
    local p = peripheral.find(pType)
    if not p then
        print("[警告] 未檢測到周邊設備: " .. pType)
    else
        print("[OK] 已連接周邊設備: " .. pType)
    end
    return p
end

print("=== 正在初始化高度控制系統 ===")
local altiSensor    = findPeripheral("altitude_sensor")
local throttleLever = findPeripheral("throttle_lever")

-- 預設紅石輸出方向 (若未使用 throttle_lever，直接由電腦邊緣輸出)
local REDSTONE_SIDE = "top" -- 可依實際佈線修改: "top", "bottom", "left", "right", "front", "back"

-- ========================================================
-- 2. 飛控狀態與 PID 參數
-- ========================================================
local state = {
    mode = "IDLE",            -- "IDLE", "HOLD_ALT", "CALIBRATING"
    targetAlt = 100.0,        -- 目標高度 (Y)
    baseThrottle = 7,         -- 懸停基準推力 (0 ~ 15 紅石信號)
    currentOutput = 0
}

-- 高度 PID 控制器 (輸入: 高度誤差公尺, 輸出: 紅石增減修正量 -8 ~ +8)
local altPID = PID.new(0.6, 0.05, 0.8, -8, 8)

-- ========================================================
-- 3. 紅石推力輸出函式
-- ========================================================
local function setThrottleOutput(signal)
    signal = math.max(0, math.min(15, math.floor(signal + 0.5)))
    state.currentOutput = signal

    -- 優先使用 Throttle Lever (Avionics 周邊)
    if throttleLever then
        throttleLever.setSignal(signal)
    end
    -- 同步輸出至電腦本體的紅石端 (支援直接接紅石線)
    redstone.setAnalogOutput(REDSTONE_SIDE, signal)
end

-- ========================================================
-- 4. 推力自動校準流程 (Calibration Routine)
-- ========================================================
local function calibrateBaseThrottle()
    if not altiSensor then
        print("[錯誤] 缺少 altitude_sensor，無法進行推力校準！")
        return
    end

    print("\n[開始推力校準] 系統將逐級測試紅石推力，尋找垂直速度接近 0 的懸停點...")
    print("請確保周圍空曠且載具處於解鎖/運行狀態。")

    local bestSignal = 0
    local minVspeed = 999

    for sig = 0, 15 do
        setThrottleOutput(sig)
        write(string.format("測試推力檔位 [%2d/15] ... ", sig))
        sleep(1.0) -- 等待載具動態穩定

        local vspeed = altiSensor.getVerticalSpeed()
        local height = altiSensor.getHeight()
        print(string.format("垂直速度: %+6.2f m/s | 當前高度: %6.1f m", vspeed, height))

        -- 尋找垂直速度最接近 0 的檔位
        if math.abs(vspeed) < minVspeed then
            minVspeed = math.abs(vspeed)
            bestSignal = sig
        end

        -- 若垂直速度已經開始明顯上升，可提前鎖定
        if vspeed > 0.3 and sig > 0 then
            break
        end
    end

    state.baseThrottle = bestSignal
    setThrottleOutput(bestSignal)
    print(string.format("\n[校準完成] 懸停基準推力已設定為: %d (剩餘垂直速度殘差: %.2f m/s)\n", bestSignal, minVspeed))
end

-- ========================================================
-- 5. 高度控制主迴圈 (Control Loop)
-- ========================================================
local function altitudeControlLoop()
    while true do
        if state.mode == "HOLD_ALT" and altiSensor then
            local currentAlt = altiSensor.getHeight()
            local vspeed = altiSensor.getVerticalSpeed()
            local altError = state.targetAlt - currentAlt

            -- PID 計算紅石修正量
            local pidAdjustment = altPID:update(altError)
            
            -- 最終紅石訊號 = 懸停基準推力 + PID 修正量
            local finalSignal = state.baseThrottle + pidAdjustment
            setThrottleOutput(finalSignal)
        elseif state.mode == "IDLE" then
            setThrottleOutput(0)
        end
        sleep(0.05) -- 1 MC tick (20Hz) 高響應控制
    end
end

-- ========================================================
-- 6. 使用者指令介面 (CLI)
-- ========================================================
local function commandLoop()
    print("\n==========================================")
    print("      純紅石高度控制系統 (Altitude Hold)   ")
    print("==========================================")
    print("指令說明:")
    print("  init / calib       - 自動校準懸停所需基準推力")
    print("  setbase <0-15>     - 手動設定懸停基準推力")
    print("  alt <y>            - 啟動定高飛行 (例如: alt 150)")
    print("  status             - 檢視目前高度、垂直速度與推力")
    print("  stop               - 關閉推力並停機")
    print("  exit               - 退出程式\n")

    while true do
        write("Altitude> ")
        local input = read()
        if input then
            local parts = {}
            for w in string.gmatch(input, "%S+") do table.insert(parts, w) end
            local cmd = string.lower(parts[1] or "")

            if cmd == "init" or cmd == "calib" then
                local prevMode = state.mode
                state.mode = "CALIBRATING"
                calibrateBaseThrottle()
                state.mode = prevMode
            elseif cmd == "setbase" then
                local val = tonumber(parts[2])
                if val and val >= 0 and val <= 15 then
                    state.baseThrottle = val
                    print(string.format("[設定] 基準推力已手動設為: %d", val))
                else
                    print("[錯誤] 請輸入 0 到 15 之間的整數！")
                end
            elseif cmd == "alt" or cmd == "target" then
                local target = tonumber(parts[2])
                if target then
                    state.targetAlt = target
                    state.mode = "HOLD_ALT"
                    altPID:reset()
                    print(string.format("[啟動] 已鎖定目標高度: %.1f m (基準推力: %d)", state.targetAlt, state.baseThrottle))
                else
                    print("[錯誤] 請指定目標高度，例如: alt 120")
                end
            elseif cmd == "status" then
                if altiSensor then
                    print(string.format("當前高度: %6.1f m | 垂直速度: %+6.2f m/s", altiSensor.getHeight(), altiSensor.getVerticalSpeed()))
                end
                print(string.format("運行模式: %s | 目標高度: %.1f m | 基準推力: %d | 當前紅石輸出: %d",
                    state.mode, state.targetAlt, state.baseThrottle, state.currentOutput))
            elseif cmd == "stop" then
                state.mode = "IDLE"
                altPID:reset()
                setThrottleOutput(0)
                print("[停機] 推力已關閉，切換至待命模式。")
            elseif cmd == "exit" then
                state.mode = "IDLE"
                setThrottleOutput(0)
                return
            end
        end
    end
end

-- 平行執行控制迴圈與指令輸入
parallel.waitForAll(altitudeControlLoop, commandLoop)
