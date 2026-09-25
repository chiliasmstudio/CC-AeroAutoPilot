#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
bundle.py — VTOL Avionics Flight Computer Bundler & Validator
=============================================================
將 modules/ 目錄下的模組化原始碼打包合併為單一獨立執行的 run.lua，
並進行語法與區塊平衡自動校驗。

使用方式:
    python bundle.py
"""

import os
import re
import sys

PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))
MODULES_DIR = os.path.join(PROJECT_DIR, "modules")
DRIVERS_DIR = os.path.join(MODULES_DIR, "drivers")
OUTPUT_RUN_LUA = os.path.join(PROJECT_DIR, "run.lua")

VERSION = "v3.8.4"

def read_file(filepath):
    with open(filepath, "r", encoding="utf-8") as f:
        return f.read()

def strip_module_wrapper(content):
    """去除模組頂部的 require 與 return Driver 等包裝，提取核心 table 結構"""
    # 移除多行註解與 safeRequire 區塊
    lines = content.splitlines()
    body_lines = []
    skip = False
    for line in lines:
        if line.strip().startswith("--[["):
            skip = True
        if skip:
            if "]]" in line:
                skip = False
            continue
        if "local function safeRequire" in line or "package.loaded" in line or "pcall(require" in line:
            continue
        if line.strip().startswith("local VIEW_TITLES"):
            continue
        body_lines.append(line)
    
    clean_code = "\n".join(body_lines).strip()
    return clean_code

def bundle():
    print("=" * 60)
    print("🚀 VTOL Avionics Bundler: Compiling modules into run.lua...")
    print("=" * 60)

    # 1. 讀取各模組原始碼
    font_path = os.path.join(MODULES_DIR, "bitmap_font.lua")
    if not os.path.exists(font_path):
        font_path = os.path.join(MODULES_DIR, "font_5x7.lua")
    font_code = read_file(font_path)
    flight_core_code = read_file(os.path.join(MODULES_DIR, "flight_core.lua"))
    driver_direct_code = read_file(os.path.join(DRIVERS_DIR, "directgpu.lua"))
    driver_tom_code = read_file(os.path.join(DRIVERS_DIR, "tom.lua"))
    driver_normal_code = read_file(os.path.join(DRIVERS_DIR, "normal.lua"))

    # 提取 FONT_5X7 結構
    font_match = re.search(r"local FONT_5X7 = \{[\s\S]*?\n\}", font_code)
    font_table = font_match.group(0) if font_match else font_code

    # 提取 FlightCore 結構（去除 return FlightCore）
    flight_core_body = re.sub(r"\nreturn FlightCore\s*$", "", flight_core_code).strip()
    if flight_core_body.startswith("--[["):
        flight_core_body = flight_core_body[flight_core_body.find("]]")+2:].strip()

    # 提取 Drivers
    def extract_driver_body(code):
        # 尋找 local Driver = { ... }
        match = re.search(r"local Driver = (\{[\s\S]*\})\s*\n*return Driver", code)
        if match:
            return match.group(1).strip()
        # 若找不到 return Driver，尋找最外層的 { ... }
        start = code.find("{")
        end = code.rfind("}")
        return code[start:end+1].strip()

    direct_body = extract_driver_body(driver_direct_code)
    tom_body = extract_driver_body(driver_tom_code)
    normal_body = extract_driver_body(driver_normal_code)

    # 2. 組合主程式 run.lua
    header = f"""--[[
    Create: Avionics & CC: Tweaked
    Unified Multi-Engine Avionics Flight Computer (多軸模組化統一飛控大腦)
    Version: {VERSION} Modular Bundle
    
    螢幕尺寸自適應分類 (Dual Screen Size Mode):
    - 完整顯示螢幕 (>= 5x5): 啟動超大 A350 儀表、細緻多引擎遙測與 3 排完整控制面板。
    - 精簡版本螢幕 (3x3 ~ 4x5, 如 3x3, 4x4, 5x4, 4x5): 啟動 2x2 四象限精簡佈局，高度防重疊排版與雙排按鈕。

    6 大功能視圖 (6 Major Glass Cockpit Views):
      1. OVERVIEW (綜合駕駛艙: 姿態儀 + 四象限動力 + 精簡控制)
      2. PFD (主飛行儀表: 巨大人工地平線姿態儀 + 數位高度錶 + 升降速度)
      3. ECAM (發動機動力監控: 2x2 四象限直觀排布，支援單側多引擎即時監控)
      4. CTRL (飛行控制畫面: 完整獨立控制面板、高度微調、油門調整、模式切換)
      5. NAV (快速導航與預設高度: 0m/80m/150m/200m/300m/鎖定)
      6. SYS (系統診斷與硬體自檢: 4 象限烏龜即時心跳、延遲與 PWM 反饋)

    使用方式:
      run.lua              (自動檢測最佳顯示硬體: DirectGPU -> Tom's GPU -> Monitor -> Terminal)
      run.lua tom          (強制使用 Tom's Peripherals GPU 驅動，支援多 GPU 聯屏)
      run.lua directgpu    (強制使用 CC-DirectGPU-Mod 驅動)
      run.lua normal       (強制使用 CC: Tweaked 原生螢幕/終端機驅動，支援多螢幕)
--]]

local VERSION = "{VERSION}"
local args = {{...}}
local requestedDriver = args[1] and string.lower(args[1]) or "auto"

-- ========================================================
-- PART 1: 飛控與動力控制核心 (FLIGHT & PROPULSION CORE - BACKEND)
-- ========================================================
"""

    drivers_header = """
-- ========================================================
-- PART 2: 多前端顯示驅動層 (FRONTEND DISPLAY DRIVERS)
-- ========================================================
local VIEW_TITLES = {
    OVERVIEW = "FLIGHT MONITOR",
    PFD      = "PRIMARY FLIGHT",
    ECAM     = "ECAM 2x2",
    CTRL     = "FLIGHT CONTROLS",
    NAV      = "NAV PRESETS",
    SYS      = "SYSTEM STATUS"
}

local Drivers = {}

-- --------------------------------------------------------
-- 2.1 驅動 A: CC-DirectGPU-Mod (高解析硬體加速)
-- --------------------------------------------------------
Drivers.direct = """

    tom_header = """

-- --------------------------------------------------------
-- 2.2 驅動 B: Tom's Peripherals 全彩點陣向量儀表 (支援多 GPU 螢幕聯動)
-- --------------------------------------------------------
"""

    normal_header = """

-- --------------------------------------------------------
-- 2.3 驅動 C: CC 原生進階彩色螢幕 (CC: Tweaked Monitor / Terminal)
-- --------------------------------------------------------
Drivers.normal = """

    orchestrator = """

-- ========================================================
-- PART 3: 主事件循環與初始化 (SYSTEM ORCHESTRATOR)
-- ========================================================
local function selectBestDriver()
    if requestedDriver == "directgpu" then
        if Drivers.direct:init() then return Drivers.direct, "CC-DirectGPU-Mod (Forced)" end
    elseif requestedDriver == "tom" or requestedDriver == "toms" then
        if Drivers.tom:init() then return Drivers.tom, "Tom's Peripherals GPU (Forced)" end
    elseif requestedDriver == "normal" or requestedDriver == "monitor" then
        Drivers.normal:init()
        return Drivers.normal, "CC Native Monitor/Terminal (Forced)"
    end

    if Drivers.direct:init() then return Drivers.direct, "CC-DirectGPU-Mod (Hardware Fast)" end
    if Drivers.tom:init() then return Drivers.tom, "Tom's Peripherals GPU (Full Color)" end
    Drivers.normal:init()
    return Drivers.normal, "CC Native Monitor/Terminal (Standard)"
end

local activeDriver, driverName = selectBestDriver()

term.clear()
term.setCursorPos(1, 1)
print(string.format("== VTOL Airship Flight Computer [%s] ==", VERSION))
print("Driver Activated: " .. driverName)
print("Screens Attached: " .. tostring(#activeDriver.screens))
print("---------------------------------------------")

FlightCore.scanQuadTurtles()
print(string.format("FL Engines: %d node(s)", #FlightCore.engines.FL))
print(string.format("FR Engines: %d node(s)", #FlightCore.engines.FR))
print(string.format("BL Engines: %d node(s)", #FlightCore.engines.BL))
print(string.format("BR Engines: %d node(s)", #FlightCore.engines.BR))
print("---------------------------------------------")
print("Flight Computer Running. Press Ctrl+T to Terminate.")

-- Tom's Peripherals GPU 需要至少 1 tick 才能回傳正確解析度
-- 在啟動 parallel loop 之前重新掃描一次，確保尺寸正確
sleep(0.1)
pcall(function() activeDriver:refreshScreens() end)
print(string.format("Screens (after reinit): %d", #activeDriver.screens))

local function flightLoop()
    while true do
        FlightCore.updateFlightLogic()
        sleep(0.05)
    end
end

local function renderLoop()
    while true do
        activeDriver:draw()
        sleep(0.05)
    end
end

local function eventLoop()
    while true do
        local eventData = {os.pullEvent()}
        local event = eventData[1]
        if event == "modem_message" then
            FlightCore.handleModemMessage(eventData[2], eventData[3], eventData[4], eventData[5], eventData[6])
        elseif event == "peripheral" or event == "peripheral_detach" or event == "monitor_resize" or event == "tm_monitor_resize" or event == "directgpu_resize" then
            if activeDriver and activeDriver.refreshScreens then
                pcall(function() activeDriver:refreshScreens() end)
            end
            pcall(function() FlightCore.scanQuadTurtles() end)
        end
        activeDriver:handleEvent(table.unpack(eventData))
        if event == "terminate" then
            FlightCore.stopEngines()
            break
        end
    end
end

parallel.waitForAny(flightLoop, renderLoop, eventLoop)
"""

    bundled_content = (
        header + "\n" +
        flight_core_body + "\n" +
        drivers_header + direct_body + "\n" +
        tom_header + font_table + "\n\nDrivers.tom = " + tom_body + "\n" +
        normal_header + normal_body + "\n" +
        orchestrator
    )

    with open(OUTPUT_RUN_LUA, "w", encoding="utf-8") as f:
        f.write(bundled_content)

    print(f"✅ Successfully compiled {os.path.basename(OUTPUT_RUN_LUA)} ({len(bundled_content.splitlines())} lines)")

    # 3. 執行語法平衡與區塊校驗
    validate_syntax(OUTPUT_RUN_LUA)

def validate_syntax(filepath):
    with open(filepath, "r", encoding="utf-8") as f:
        lines = f.readlines()

    stack = []
    in_block_comment = False
    for i, line in enumerate(lines, 1):
        if "--[[" in line: in_block_comment = True
        if in_block_comment:
            if "]]" in line: in_block_comment = False
            continue
        l = re.sub(r"--.*$", "", line)
        l = re.sub(r'"(\\.|[^"\\])*"', '""', l)
        l = re.sub(r"'(\\.|[^'\\])*'", "''", l)
        l = re.sub(r"\belseif\b.*?\bthen\b", " ", l)
        tokens = re.findall(r"\b(function|then|do|repeat|until|end)\b", l)
        for t in tokens:
            if t in ("function", "then", "do", "repeat"):
                stack.append((t, i))
            elif t == "end":
                if stack: stack.pop()
                else: print(f"❌ Syntax Error: Unmatched 'end' at line {i}")
            elif t == "until":
                if stack: stack.pop()
                else: print(f"❌ Syntax Error: Unmatched 'until' at line {i}")

    if stack:
        print(f"❌ Syntax Error in {os.path.basename(filepath)}: Unclosed blocks {stack}")
        sys.exit(1)
    else:
        print(f"✅ Syntax Validation Passed: All blocks in {os.path.basename(filepath)} are perfectly balanced!")

def get_version():
    """從 run.lua 提取實際封裝版本，若不存在則回傳 VERSION 常數"""
    if os.path.exists(OUTPUT_RUN_LUA):
        content = read_file(OUTPUT_RUN_LUA)
        m = re.search(r'local\s+VERSION\s*=\s*"([^"]+)"', content)
        if m:
            return m.group(1)
    return VERSION

if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] in ("--version", "-v", "version"):
        print(get_version())
    else:
        bundle()
