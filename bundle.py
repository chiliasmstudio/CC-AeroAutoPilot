#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
bundle.py — VTOL Avionics Flight Computer Bundler & Validator
=============================================================
將 modules/ 目錄下的模組化原始碼打包合併為：
  1. run.lua     (單機一體化飛控大腦)
  2. fcc.lua     (分散式獨立飛控大腦 FCC - 0% 畫面開銷，20Hz 純計算)
  3. display.lua (分散式駕駛艙螢幕系統 CDS - 專職螢幕渲染與觸控)
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
OUTPUT_FCC_LUA = os.path.join(PROJECT_DIR, "fcc.lua")
OUTPUT_DISPLAY_LUA = os.path.join(PROJECT_DIR, "display.lua")
OUTPUT_CLI_LUA = os.path.join(PROJECT_DIR, "fcc-cli.lua")
OUTPUT_BOOT_LUA = os.path.join(PROJECT_DIR, "boot.lua")
OUTPUT_TURTLE_LUA = os.path.join(PROJECT_DIR, "turtle_startup.lua")
VERSION_XML_FILE = os.path.join(PROJECT_DIR, "version.xml")

import xml.etree.ElementTree as ET
from xml.dom import minidom

def read_file(filepath):
    with open(filepath, "r", encoding="utf-8") as f:
        return f.read()

def get_versions():
    """從 version.xml 讀取電腦與烏龜版本號，回傳 dict: {'computer': 'v4.0.4', 'turtle': 'v4.0.4'}"""
    default_versions = {"computer": "v4.0.4", "turtle": "v4.0.4"}
    if not os.path.exists(VERSION_XML_FILE):
        save_versions(default_versions)
        return default_versions
    try:
        tree = ET.parse(VERSION_XML_FILE)
        root = tree.getroot()
        comp = root.find("computer")
        turt = root.find("turtle")
        comp_ver = comp.text.strip() if comp is not None and comp.text else "v4.0.4"
        turt_ver = turt.text.strip() if turt is not None and turt.text else "v4.0.4"
        if not comp_ver.startswith("v"): comp_ver = f"v{comp_ver}"
        if not turt_ver.startswith("v"): turt_ver = f"v{turt_ver}"
        return {"computer": comp_ver, "turtle": turt_ver}
    except Exception as e:
        print(f"⚠️ Warning: Failed to parse version.xml ({e}), using defaults.")
        return default_versions

def save_versions(versions_dict):
    """寫入版本號至 version.xml，具備格式化縮排"""
    root = ET.Element("versions")
    comp_elem = ET.SubElement(root, "computer")
    comp_ver = versions_dict.get("computer", "v4.0.4")
    comp_elem.text = comp_ver if comp_ver.startswith("v") else f"v{comp_ver}"
    
    turt_elem = ET.SubElement(root, "turtle")
    turt_ver = versions_dict.get("turtle", "v4.0.4")
    turt_elem.text = turt_ver if turt_ver.startswith("v") else f"v{turt_ver}"
    
    xml_str = ET.tostring(root, encoding="utf-8")
    parsed = minidom.parseString(xml_str)
    pretty_xml = parsed.toprettyxml(indent="    ", encoding="utf-8").decode("utf-8")
    clean_lines = [l for l in pretty_xml.splitlines() if l.strip()]
    with open(VERSION_XML_FILE, "w", encoding="utf-8") as f:
        f.write("\n".join(clean_lines) + "\n")

def get_version(target="computer"):
    """相容性函數：取得指定或預設(電腦)版本號"""
    vers = get_versions()
    return vers.get(target, vers.get("computer", "v4.0.4"))

def set_version(new_ver, target="all"):
    """寫入新版本號至 version.xml"""
    if not new_ver.startswith("v"):
        new_ver = f"v{new_ver}"
    vers = get_versions()
    if target in ("all", "both"):
        vers["computer"] = new_ver
        vers["turtle"] = new_ver
    elif target == "computer":
        vers["computer"] = new_ver
    elif target == "turtle":
        vers["turtle"] = new_ver
    save_versions(vers)
    return vers

def bump_semver(ver_str, bump_type="patch"):
    """解析語義化版本號並遞增: patch (4.0.4 -> 4.0.5), minor (4.0.4 -> 4.1.0), major (4.0.4 -> 5.0.0)"""
    ver = ver_str.lstrip("v")
    parts = ver.split(".")
    while len(parts) < 3:
        parts.append("0")
    try:
        major, minor, patch = int(parts[0]), int(parts[1]), int(parts[2])
    except ValueError:
        major, minor, patch = 4, 0, 4

    if bump_type == "major":
        major += 1
        minor = 0
        patch = 0
    elif bump_type == "minor":
        minor += 1
        patch = 0
    else:
        patch += 1

    return f"v{major}.{minor}.{patch}"

def bump_versions(target="all", bump_type="patch"):
    """遞增版本號: target='all'|'computer'|'turtle'"""
    vers = get_versions()
    if target in ("all", "both"):
        vers["computer"] = bump_semver(vers["computer"], bump_type)
        vers["turtle"] = bump_semver(vers["turtle"], bump_type)
    elif target == "computer":
        vers["computer"] = bump_semver(vers["computer"], bump_type)
    elif target == "turtle":
        vers["turtle"] = bump_semver(vers["turtle"], bump_type)
    save_versions(vers)
    return vers

def update_file_version(filepath, version_str, role_type="computer"):
    """在現有原始碼檔案中替換版本佔位符與版本常數"""
    if not os.path.exists(filepath):
        return
    content = read_file(filepath)
    
    # 1. 替換通用與特定佔位符
    content = re.sub(r"<(?:VERSION|version)>", version_str, content)
    content = re.sub(r"\{\{(?:VERSION|version)\}\}", version_str, content)
    
    if role_type == "computer":
        content = re.sub(r"<(?:VERSION_COMPUTER|version_computer)>", version_str, content)
        content = re.sub(r"\{\{(?:VERSION_COMPUTER|version_computer)\}\}", version_str, content)
    elif role_type == "turtle":
        content = re.sub(r"<(?:VERSION_TURTLE|version_turtle)>", version_str, content)
        content = re.sub(r"\{\{(?:VERSION_TURTLE|version_turtle)\}\}", version_str, content)

    # 2. 替換 local VERSION = "..."
    content = re.sub(r'local\s+VERSION\s*=\s*"[^"]*"', f'local VERSION = "{version_str}"', content)

    # 3. 替換標頭註解中的 Version: vX.Y.Z 或 Version: <...>
    content = re.sub(r'(Version:\s*)(v\d+\.\d+\.\d+|<[^>]+>|\{\{[^}]+\}\})', rf'\g<1>{version_str}', content)

    with open(filepath, "w", encoding="utf-8") as f:
        f.write(content)

def bundle():
    vers = get_versions()
    COMP_VER = vers["computer"]
    TURT_VER = vers["turtle"]
    VERSION = COMP_VER
    print("=" * 60)
    print(f"🚀 VTOL Avionics Bundler [Computer: {COMP_VER} | Turtle: {TURT_VER}]: Compiling modules...")
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
        match = re.search(r"local Driver = (\{[\s\S]*\})\s*\n*return Driver", code)
        if match:
            return match.group(1).strip()
        start = code.find("{")
        end = code.rfind("}")
        return code[start:end+1].strip()

    direct_body = extract_driver_body(driver_direct_code)
    tom_body = extract_driver_body(driver_tom_code)
    normal_body = extract_driver_body(driver_normal_code)

    drivers_header = """
-- ========================================================
-- PART 2: 多前端顯示驅動層 (FRONTEND DISPLAY DRIVERS)
-- ========================================================
local VIEW_TITLES = {
    OVERVIEW = "FLIGHT MONITOR",
    PFD      = "PRIMARY FLIGHT",
    ECAM     = "ECAM 2x2",
    CTRL     = "FLIGHT CONTROLS",
    NAV      = "HEADING & NAV",
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

    # ========================================================
    # 2. 生成 run.lua (單機一體化)
    # ========================================================
    run_header = f"""--[[
    Create: Avionics & CC: Tweaked
    Unified Multi-Engine Avionics Flight Computer (多軸模組化統一飛控大腦)
    Version: {VERSION} Modular Bundle
    
    螢幕尺寸自適應分類 (Dual Screen Size Mode):
    - 完整顯示螢幕 (>= 5x5): 啟動超大 A350 儀表、細緻多引擎遙測與 3 排完整控制面板。
    - 精簡版本螢幕 (3x3 ~ 4x5, 如 3x3, 4x4, 5x4, 4x5): 啟動 2x2 四象限精簡佈局，高度防重疊排版與雙排按鈕。

    6 大功能視圖: OVERVIEW, PFD, ECAM, CTRL, NAV, SYS

    使用方式:
      run.lua              (自動檢測最佳顯示硬體: DirectGPU -> Tom's GPU -> Monitor -> Terminal)
      run.lua tom          (強制使用 Tom's Peripherals GPU 驅動)
      run.lua directgpu    (強制使用 CC-DirectGPU-Mod 驅動)
      run.lua normal       (強制使用 CC 原生螢幕/終端機驅動)
--]]

local VERSION = "{VERSION}"
local args = {{...}}
local requestedDriver = args[1] and string.lower(args[1]) or "auto"

-- ========================================================
-- PART 1: 飛控與動力控制核心 (FLIGHT & PROPULSION CORE - BACKEND)
-- ========================================================
"""

    run_orchestrator = """
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
        local eventData = {os.pullEventRaw()}
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

    bundled_run = (
        run_header + "\n" +
        flight_core_body + "\n" +
        drivers_header + direct_body + "\n" +
        tom_header + font_table + "\n\nDrivers.tom = " + tom_body + "\n" +
        normal_header + normal_body + "\n" +
        run_orchestrator
    )

    with open(OUTPUT_RUN_LUA, "w", encoding="utf-8") as f:
        f.write(bundled_run)
    print(f"✅ Compiled {os.path.basename(OUTPUT_RUN_LUA)} ({len(bundled_run.splitlines())} lines)")
    validate_syntax(OUTPUT_RUN_LUA)

    # ========================================================
    # 3. 生成 fcc.lua (分散式獨立飛控大腦)
    # ========================================================
    fcc_header = f"""--[[
    Create: Avionics & CC: Tweaked
    Standalone Flight Control Computer (FCC - 分散式純飛控大腦)
    Version: {VERSION}
    
    特點:
    - 0% 畫面渲染開銷 (No UI Rendering Overhead)，運算時間 < 0.2ms
    - 20Hz 高頻穩定 PD-V 垂直升力與高度控制，永不卡頓
    - 支援 Channel 100/101 烏龜推進動力同步
    - 支援 Channel 102 遙測狀態廣播 (10Hz) 與 Channel 103 駕駛艙螢幕指令接收
    - 徹底根除 "Too long without yielding"
--]]

local VERSION = "{VERSION}"

-- ========================================================
-- PART 1: 飛控與動力控制核心 (FLIGHT & PROPULSION CORE)
-- ========================================================
"""

    fcc_orchestrator = """
-- ========================================================
-- PART 2: FCC 遙測廣播與駕駛艙指令通訊協議 (FCC TELEMETRY & COMMANDS)
-- ========================================================
local function handleFccCommand(side, ch, replyCh, msg, dist)
    if ch == 103 and type(msg) == "table" and msg.cmd then
        local cmd = msg.cmd
        if cmd == "HOLD_ALT" then FlightCore.holdAltitude()
        elseif cmd == "CALIB" then FlightCore.startCalibration()
        elseif cmd == "STOP" then FlightCore.stopEngines()
        elseif cmd == "LOCK_ALT" then FlightCore.lockCurrentAlt()
        elseif cmd == "ADJ_ALT" then FlightCore.adjustTargetAlt(msg.delta or 0)
        elseif cmd == "SET_ALT" then FlightCore.setTargetAlt(msg.alt or 0)
        elseif cmd == "ADJ_BASE" then FlightCore.adjustBaseThrottle(msg.delta or 0)
        elseif cmd == "ADJ_FWD" then FlightCore.adjustForwardThrottle(msg.delta or 0)
        elseif cmd == "SET_FWD" then FlightCore.setForwardThrottle(msg.val or 0)
        elseif cmd == "ADJ_TURN" then FlightCore.adjustTurnThrottle(msg.delta or 0)
        elseif cmd == "SET_TURN" then FlightCore.setTurnThrottle(msg.val or 0)
        elseif cmd == "STOP_HORIZ" then FlightCore.stopHorizontalThrust()
        elseif cmd == "SET_HEADING" then FlightCore.setTargetHeading(msg.hdg or 0)
        elseif cmd == "ADJ_HEADING" then FlightCore.adjustTargetHeading(msg.delta or 0)
        elseif cmd == "TOGGLE_HEADING" then FlightCore.toggleHeadingHold()
        elseif cmd == "SYNC_HEADING" then FlightCore.syncHeading()
        elseif cmd == "SET_WAYPOINT" then FlightCore.setWaypoint(msg.x, msg.z, msg.radius)
        elseif cmd == "CANCEL_WAYPOINT" then FlightCore.cancelWaypoint()
        elseif cmd == "RESCAN_HW" then FlightCore.scanQuadTurtles()
        end
    end
end

local function broadcastTelemetry()
    local currentAlt = (FlightCore.altiSensor and FlightCore.altiSensor.getHeight and FlightCore.altiSensor.getHeight()) or FlightCore.nav.y or 0
    local vspeed = (FlightCore.altiSensor and FlightCore.altiSensor.getVerticalSpeed and FlightCore.altiSensor.getVerticalSpeed()) or FlightCore.filteredVspeed or 0
    local currPitch, currRoll = FlightCore.getGimbalData()

    local teleMsg = {
        type = "FCC_TELEMETRY",
        ver = VERSION,
        mode = FlightCore.state.mode,
        targetAlt = FlightCore.state.targetAlt,
        virtualAlt = FlightCore.state.virtualAlt,
        baseThrottle = FlightCore.state.baseThrottle,
        statusMsg = FlightCore.state.statusMsg,
        stableTimer = FlightCore.state.stableTimer,
        calibPhase = FlightCore.state.calibPhase,
        fwdThrottle = FlightCore.state.fwdThrottle,
        turnThrottle = FlightCore.state.turnThrottle,
        strafeThrottle = FlightCore.state.strafeThrottle,
        vspeed = vspeed,
        currentAlt = currentAlt,
        pitch = currPitch,
        roll = currRoll,
        nav = {
            x = FlightCore.nav.x,
            y = FlightCore.nav.y,
            z = FlightCore.nav.z,
            yaw = FlightCore.nav.yaw,
            pitch = FlightCore.nav.pitch,
            roll = FlightCore.nav.roll,
            speed = FlightCore.nav.speed,
            source = FlightCore.nav.source,
            targetHeading = FlightCore.nav.targetHeading,
            headingHold = FlightCore.nav.headingHold,
            targetX = FlightCore.nav.targetX,
            targetZ = FlightCore.nav.targetZ,
            wpActive = FlightCore.nav.wpActive,
            arrivalRadius = FlightCore.nav.arrivalRadius,
            diag = (FlightCore.getNavDiagnostic and FlightCore.getNavDiagnostic()) or "Searching..."
        },
        outputs = FlightCore.virtualOutputs,
        engineOutputs = FlightCore.engineOutputs,
        turtles = FlightCore.turtles,
        quadHealth = {
            FL = FlightCore.getQuadHealth("FL"),
            FR = FlightCore.getQuadHealth("FR"),
            BL = FlightCore.getQuadHealth("BL"),
            BR = FlightCore.getQuadHealth("BR"),
            FWD = FlightCore.getQuadHealth("FWD"),
            BWD = FlightCore.getQuadHealth("BWD"),
            LEFT = FlightCore.getQuadHealth("LEFT"),
            RIGHT = FlightCore.getQuadHealth("RIGHT")
        },
        hw = (FlightCore.getHardwareInventory and FlightCore.getHardwareInventory()) or nil,
        gimbalAvailable = FlightCore.gimbalAvailable,
        timestamp = os.epoch("utc")
    }

    for _, m in ipairs(FlightCore.modems) do
        pcall(function() m.transmit(102, 103, teleMsg) end)
    end
end

-- ========================================================
-- PART 3: FCC 主事件循環與終端監控 (FCC ORCHESTRATOR)
-- ========================================================
FlightCore.scanQuadTurtles()
for _, m in ipairs(FlightCore.modems) do
    pcall(function() m.open(101) end)
    pcall(function() m.open(103) end)
end

local teleTick = 0
local function flightLoop()
    while true do
        FlightCore.updateFlightLogic()
        teleTick = teleTick + 1
        if teleTick % 2 == 0 then
            broadcastTelemetry()
        end
        sleep(0.05)
    end
end

local function eventLoop()
    while true do
        local eventData = {os.pullEventRaw()}
        local event = eventData[1]
        if event == "modem_message" then
            local side, ch, replyCh, msg, dist = eventData[2], eventData[3], eventData[4], eventData[5], eventData[6]
            if ch == 101 then
                FlightCore.handleModemMessage(side, ch, replyCh, msg, dist)
            elseif ch == 103 then
                handleFccCommand(side, ch, replyCh, msg, dist)
            end
        elseif event == "peripheral" or event == "peripheral_detach" then
            pcall(function() FlightCore.scanQuadTurtles() end)
            for _, m in ipairs(FlightCore.modems) do
                pcall(function() m.open(101) end)
                pcall(function() m.open(103) end)
            end
        elseif event == "terminate" then
            FlightCore.stopEngines()
            break
        end
    end
end

local function uiLoop()
    while true do
        term.clear()
        term.setCursorPos(1, 1)
        local currentAlt = (FlightCore.altiSensor and FlightCore.altiSensor.getHeight and FlightCore.altiSensor.getHeight()) or FlightCore.nav.y or 0
        local vspeed = (FlightCore.altiSensor and FlightCore.altiSensor.getVerticalSpeed and FlightCore.altiSensor.getVerticalSpeed()) or FlightCore.filteredVspeed or 0
        local currPitch, currRoll = FlightCore.getGimbalData()
        local qFL = FlightCore.getQuadHealth("FL")
        local qFR = FlightCore.getQuadHealth("FR")
        local qBL = FlightCore.getQuadHealth("BL")
        local qBR = FlightCore.getQuadHealth("BR")
        local qFWD = FlightCore.getQuadHealth("FWD")

        print("========================================")
        print("  VTOL Flight Control Computer (FCC)   ")
        print(string.format("  Version: %s [DISTRIBUTED MODE]  ", VERSION))
        print("========================================")
        print(string.format(" Status : %s", FlightCore.state.statusMsg or "Ready"))
        print(string.format(" Mode   : %-10s  Alt: %.1fm (Tgt: %.1fm)", FlightCore.state.mode, currentAlt, FlightCore.state.targetAlt))
        print(string.format(" Hover  : Base: %.2f  V.S: %+.2fm/s", FlightCore.state.baseThrottle, vspeed))
        print(string.format(" Gimbal : Pitch: %+3.1f*  Roll: %+3.1f*", currPitch, currRoll))
        print(string.format(" Heading: %03d* (Hold: %s, Tgt: %03d*)", math.floor(FlightCore.nav.yaw or 0), FlightCore.nav.headingHold and "ON" or "OFF", math.floor(FlightCore.nav.targetHeading or 0)))
        if FlightCore.nav.x then
            print(string.format(" Nav Pos: X:%.0f Y:%.0f Z:%.0f (Spd:%.1f)", FlightCore.nav.x, FlightCore.nav.y or currentAlt, FlightCore.nav.z, FlightCore.nav.speed or 0))
        else
            local diag = (FlightCore.getNavDiagnostic and FlightCore.getNavDiagnostic()) or "Searching..."
            print(" Nav Pos: " .. diag)
        end
        print(string.format(" Lift   : FL:%.1f FR:%.1f BL:%.1f BR:%.1f", FlightCore.virtualOutputs.FL or 0, FlightCore.virtualOutputs.FR or 0, FlightCore.virtualOutputs.BL or 0, FlightCore.virtualOutputs.BR or 0))
        print(string.format(" Cruis  : FWD:%.1f BWD:%.1f L:%.1f R:%.1f", FlightCore.virtualOutputs.FWD or 0, FlightCore.virtualOutputs.BWD or 0, FlightCore.virtualOutputs.LEFT or 0, FlightCore.virtualOutputs.RIGHT or 0))
        print(string.format(" Turtles: FL(%d) FR(%d) BL(%d) BR(%d) FWD(%d)", qFL.online, qFR.online, qBL.online, qBR.online, qFWD.online))
        print(" Net Ch : 100(Turtles) 102(Tele) 103(Cmd)")
        print("========================================")
        print(" FCC Core Running (0% CPU, No Render)")
        print(" Press Ctrl+T to Terminate.")
        sleep(0.5)
    end
end

parallel.waitForAny(flightLoop, eventLoop, uiLoop)
"""

    bundled_fcc = (
        fcc_header + "\n" +
        flight_core_body + "\n" +
        fcc_orchestrator
    )

    with open(OUTPUT_FCC_LUA, "w", encoding="utf-8") as f:
        f.write(bundled_fcc)
    print(f"✅ Compiled {os.path.basename(OUTPUT_FCC_LUA)} ({len(bundled_fcc.splitlines())} lines)")
    validate_syntax(OUTPUT_FCC_LUA)

    # ========================================================
    # 4. 生成 display.lua (分散式駕駛艙螢幕系統)
    # ========================================================
    display_header = f"""--[[
    Create: Avionics & CC: Tweaked
    Standalone Cockpit Display System (CDS / Glass Cockpit)
    Version: {VERSION}
    
    特點:
    - 專門負責駕駛艙螢幕繪圖渲染與觸控按鈕交互
    - 透過數據機 Channel 102 即時接收來自 FCC 飛控大腦的遙測數據
    - 透過數據機 Channel 103 發送駕駛操作指令給 FCC
    - 支援 1 台或多台螢幕電腦同時連線同一艘飛船
    - 即使渲染再繁重，也 100% 不會拖慢飛船飛控運算
--]]

local VERSION = "{VERSION}"
local args = {{...}}
local requestedDriver = args[1] and string.lower(args[1]) or "auto"

-- ========================================================
-- PART 1: 虛擬飛控代理端 (FLIGHT CORE PROXY - NETWORKING)
-- ========================================================
local FlightCore = {{
    state = {{
        mode = "IDLE",
        targetAlt = 200.0,
        virtualAlt = 100.0,
        baseThrottle = 1.0,
        statusMsg = "Waiting for FCC...",
        stableTimer = 0,
        calibStartAlt = 0,
        calibTargetAlt = 0,
        calibPhase = "GROUND_SEARCH",
        calibThrottle = 0.0,
        fwdThrottle = 0.0,
        turnThrottle = 0.0,
        strafeThrottle = 0.0,
        currentAlt = 0.0,
        vspeed = 0.0,
        pitch = 0.0,
        roll = 0.0
    }},
    nav = {{
        x = nil, y = nil, z = nil,
        yaw = 0.0, pitch = 0.0, roll = 0.0, speed = 0.0,
        source = "BARO",
        targetHeading = 0.0, headingHold = false,
        targetX = nil, targetZ = nil, wpActive = false,
        arrivalRadius = 20.0
    }},
    virtualOutputs = {{
        FL = 0.0, FR = 0.0, BL = 0.0, BR = 0.0,
        FWD = 0.0, BWD = 0.0, LEFT = 0.0, RIGHT = 0.0
    }},
    engineOutputs = {{
        FL = 0, FR = 0, BL = 0, BR = 0,
        FWD = 0, BWD = 0, LEFT = 0, RIGHT = 0
    }},
    engines = {{
        FL = {{}}, FR = {{}}, BL = {{}}, BR = {{}},
        FWD = {{}}, BWD = {{}}, LEFT = {{}}, RIGHT = {{}}
    }},
    turtles = {{}},
    quadHealth = {{}},
    hw = {{
        displays = {{ hasDirectGpu=false, tomsCount=0, monitorsCount=0, totalScreens=0 }},
        engines = {{ lift={{online=0, total=0}}, cruise={{online=0, total=0}}, steer={{online=0, total=0}}, totalOnline=0, totalCount=0 }},
        avionics = {{ onBody=false, aic=false, ins=false, navTable=false, alti=false, gimbal=false, modems=0, navSource="NONE" }}
    }},
    gimbalAvailable = false,
    modems = {{}},
    lastPacketTime = 0,
    fccConnected = false
}}

function FlightCore.initModems()
    FlightCore.modems = {{}}
    for _, name in ipairs(peripheral.getNames()) do
        if peripheral.getType(name) == "modem" then
            local m = peripheral.wrap(name)
            if m then
                pcall(function() m.open(102) end)
                table.insert(FlightCore.modems, m)
            end
        end
    end
end

function FlightCore.sendCmd(cmdTable)
    for _, m in ipairs(FlightCore.modems) do
        pcall(function() m.transmit(103, 102, cmdTable) end)
    end
end

function FlightCore.holdAltitude()
    FlightCore.state.mode = "HOLD_ALT"
    FlightCore.sendCmd({{cmd = "HOLD_ALT"}})
end

function FlightCore.startCalibration()
    FlightCore.state.mode = "CALIBRATING"
    FlightCore.sendCmd({{cmd = "CALIB"}})
end

function FlightCore.stopEngines()
    FlightCore.state.mode = "IDLE"
    FlightCore.sendCmd({{cmd = "STOP"}})
end

function FlightCore.setTargetAlt(alt)
    FlightCore.state.targetAlt = alt
    FlightCore.sendCmd({{cmd = "SET_ALT", alt = alt}})
end

function FlightCore.adjustTargetAlt(delta)
    FlightCore.state.targetAlt = (FlightCore.state.targetAlt or 200) + delta
    FlightCore.sendCmd({{cmd = "ADJ_ALT", delta = delta}})
end

function FlightCore.lockCurrentAlt()
    FlightCore.sendCmd({{cmd = "LOCK_ALT"}})
end

function FlightCore.adjustBaseThrottle(delta)
    FlightCore.state.baseThrottle = math.max(0.01, math.min(15.0, (FlightCore.state.baseThrottle or 1.0) + delta))
    FlightCore.sendCmd({{cmd = "ADJ_BASE", delta = delta}})
end

function FlightCore.setForwardThrottle(val)
    FlightCore.state.fwdThrottle = val
    FlightCore.sendCmd({{cmd = "SET_FWD", val = val}})
end

function FlightCore.adjustForwardThrottle(delta)
    FlightCore.state.fwdThrottle = (FlightCore.state.fwdThrottle or 0) + delta
    FlightCore.sendCmd({{cmd = "ADJ_FWD", delta = delta}})
end

function FlightCore.setTurnThrottle(val)
    FlightCore.state.turnThrottle = val
    FlightCore.sendCmd({{cmd = "SET_TURN", val = val}})
end

function FlightCore.adjustTurnThrottle(delta)
    FlightCore.state.turnThrottle = (FlightCore.state.turnThrottle or 0) + delta
    FlightCore.sendCmd({{cmd = "ADJ_TURN", delta = delta}})
end

function FlightCore.setStrafeThrottle(val)
    FlightCore.setTurnThrottle(val)
end

function FlightCore.adjustStrafeThrottle(delta)
    FlightCore.adjustTurnThrottle(delta)
end

function FlightCore.stopHorizontalThrust()
    FlightCore.state.fwdThrottle = 0
    FlightCore.state.turnThrottle = 0
    FlightCore.state.strafeThrottle = 0
    FlightCore.sendCmd({{cmd = "STOP_HORIZ"}})
end

function FlightCore.setTargetHeading(hdg)
    FlightCore.nav.targetHeading = hdg
    FlightCore.nav.headingHold = true
    FlightCore.sendCmd({{cmd = "SET_HEADING", hdg = hdg}})
end

function FlightCore.adjustTargetHeading(delta)
    FlightCore.nav.targetHeading = (math.floor((FlightCore.nav.targetHeading or 0) + delta) % 360 + 360) % 360
    FlightCore.sendCmd({{cmd = "ADJ_HEADING", delta = delta}})
end

function FlightCore.toggleHeadingHold()
    FlightCore.nav.headingHold = not FlightCore.nav.headingHold
    FlightCore.sendCmd({{cmd = "TOGGLE_HEADING"}})
end

function FlightCore.syncHeading()
    FlightCore.nav.targetHeading = math.floor(FlightCore.nav.yaw or 0)
    FlightCore.nav.headingHold = true
    FlightCore.sendCmd({{cmd = "SYNC_HEADING"}})
end

function FlightCore.setWaypoint(x, z, radius)
    FlightCore.nav.targetX = tonumber(x)
    FlightCore.nav.targetZ = tonumber(z)
    FlightCore.nav.arrivalRadius = tonumber(radius) or 20.0
    FlightCore.nav.wpActive = true
    FlightCore.sendCmd({{cmd = "SET_WAYPOINT", x = x, z = z, radius = radius}})
end

function FlightCore.cancelWaypoint()
    FlightCore.nav.wpActive = false
    FlightCore.sendCmd({{cmd = "CANCEL_WAYPOINT"}})
end

function FlightCore.scanQuadTurtles()
    FlightCore.sendCmd({{cmd = "RESCAN_HW"}})
end

FlightCore.altiSensor = {{
    getHeight = function() return FlightCore.state.currentAlt or 0 end,
    getVerticalSpeed = function() return FlightCore.state.vspeed or 0 end
}}

function FlightCore.getGimbalData()
    return FlightCore.state.pitch or 0, FlightCore.state.roll or 0
end

function FlightCore.getQuadHealth(role)
    if FlightCore.quadHealth and FlightCore.quadHealth[role] then
        return FlightCore.quadHealth[role]
    end
    local now = os.epoch("utc")
    local quadTurtles = {{}}
    local onlineCount = 0

    for id, t in pairs(FlightCore.turtles or {{}}) do
        if t.role == role then
            local isOnline = (now - (t.lastSeen or now)) < 3500
            table.insert(quadTurtles, {{
                id = id,
                label = t.label or ("T#" .. id),
                online = isOnline,
                sig = t.sig or 0,
                age = (now - (t.lastSeen or now)) / 1000
            }})
            if isOnline then onlineCount = onlineCount + 1 end
        end
    end

    return {{
        total = math.max(1, #quadTurtles),
        online = onlineCount,
        nodes = quadTurtles
    }}
end

function FlightCore.getHardwareInventory()
    return FlightCore.hw or {{
        displays = {{ hasDirectGpu=false, tomsCount=0, monitorsCount=0, totalScreens=0 }},
        engines = {{ lift={{online=0, total=0}}, cruise={{online=0, total=0}}, steer={{online=0, total=0}}, totalOnline=0, totalCount=0 }},
        avionics = {{ onBody=false, aic=false, ins=false, navTable=false, alti=false, gimbal=false, modems=0, navSource="NONE" }}
    }}
end

function FlightCore.handleModemMessage(side, ch, replyCh, msg, dist)
    if ch == 102 and type(msg) == "table" and msg.type == "FCC_TELEMETRY" then
        FlightCore.lastPacketTime = os.epoch("utc")
        FlightCore.fccConnected = true
        FlightCore.state.mode = msg.mode or FlightCore.state.mode
        FlightCore.state.targetAlt = msg.targetAlt or FlightCore.state.targetAlt
        FlightCore.state.virtualAlt = msg.virtualAlt or FlightCore.state.virtualAlt
        FlightCore.state.baseThrottle = msg.baseThrottle or FlightCore.state.baseThrottle
        FlightCore.state.statusMsg = msg.statusMsg or FlightCore.state.statusMsg
        FlightCore.state.stableTimer = msg.stableTimer or 0
        FlightCore.state.calibPhase = msg.calibPhase or FlightCore.state.calibPhase
        FlightCore.state.fwdThrottle = msg.fwdThrottle or 0
        FlightCore.state.turnThrottle = msg.turnThrottle or 0
        FlightCore.state.strafeThrottle = msg.strafeThrottle or 0
        FlightCore.state.currentAlt = msg.currentAlt or 0
        FlightCore.state.vspeed = msg.vspeed or 0
        FlightCore.state.pitch = msg.pitch or 0
        FlightCore.state.roll = msg.roll or 0

        if msg.nav then
            FlightCore.nav = msg.nav
        end
        if msg.outputs then
            FlightCore.virtualOutputs = msg.outputs
        end
        if msg.engineOutputs then
            FlightCore.engineOutputs = msg.engineOutputs
        end
        if msg.turtles then
            FlightCore.turtles = msg.turtles
        end
        if msg.quadHealth then
            FlightCore.quadHealth = msg.quadHealth
        end
        if msg.hw then
            FlightCore.hw = msg.hw
        end
        FlightCore.gimbalAvailable = msg.gimbalAvailable or false
    end
end
"""

    display_orchestrator = """
-- ========================================================
-- PART 3: 主事件循環與初始化 (DISPLAY ORCHESTRATOR)
-- ========================================================
FlightCore.initModems()

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
print(string.format("== Cockpit Display System [%s] ==", VERSION))
print("Driver Activated: " .. driverName)
print("Screens Attached: " .. tostring(#activeDriver.screens))
print("Connecting to FCC on Modem Channel 102...")
print("---------------------------------------------")

sleep(0.1)
pcall(function() activeDriver:refreshScreens() end)

local function renderLoop()
    while true do
        if os.epoch("utc") - FlightCore.lastPacketTime > 3000 then
            FlightCore.fccConnected = false
            FlightCore.state.statusMsg = "[ NO FCC LINK - CH 102 ]"
        end
        activeDriver:draw()
        sleep(0.05)
    end
end

local function eventLoop()
    while true do
        local eventData = {os.pullEventRaw()}
        local event = eventData[1]
        if event == "modem_message" then
            FlightCore.handleModemMessage(eventData[2], eventData[3], eventData[4], eventData[5], eventData[6])
        elseif event == "peripheral" or event == "peripheral_detach" or event == "monitor_resize" or event == "tm_monitor_resize" or event == "directgpu_resize" then
            if activeDriver and activeDriver.refreshScreens then
                pcall(function() activeDriver:refreshScreens() end)
            end
            FlightCore.initModems()
        end
        activeDriver:handleEvent(table.unpack(eventData))
        if event == "terminate" then
            break
        end
    end
end

parallel.waitForAny(renderLoop, eventLoop)
"""

    bundled_display = (
        display_header + "\n" +
        drivers_header + direct_body + "\n" +
        tom_header + font_table + "\n\nDrivers.tom = " + tom_body + "\n" +
        normal_header + normal_body + "\n" +
        display_orchestrator
    )

    with open(OUTPUT_DISPLAY_LUA, "w", encoding="utf-8") as f:
        f.write(bundled_display)
    print(f"✅ Compiled {os.path.basename(OUTPUT_DISPLAY_LUA)} ({len(bundled_display.splitlines())} lines)")

    # 4. 同步版本號至 Standalone 程式 (fcc-cli.lua, boot.lua, turtle_startup.lua)
    update_file_version(OUTPUT_CLI_LUA, COMP_VER, "computer")
    print(f"🔄 Synchronized {os.path.basename(OUTPUT_CLI_LUA)} -> {COMP_VER}")
    
    update_file_version(OUTPUT_BOOT_LUA, COMP_VER, "computer")
    print(f"🔄 Synchronized {os.path.basename(OUTPUT_BOOT_LUA)} -> {COMP_VER}")

    update_file_version(OUTPUT_TURTLE_LUA, TURT_VER, "turtle")
    print(f"🔄 Synchronized {os.path.basename(OUTPUT_TURTLE_LUA)} -> {TURT_VER}")

    # 5. 語法檢查所有發布檔案
    print("\n🔍 Validating syntax across all release assets...")
    for label, path in [
        ("run.lua (All-in-One)", OUTPUT_RUN_LUA),
        ("fcc.lua (FCC Brain)", OUTPUT_FCC_LUA),
        ("display.lua (CDS Display)", OUTPUT_DISPLAY_LUA),
        ("fcc-cli.lua (Command Console)", OUTPUT_CLI_LUA),
        ("turtle_startup.lua (Turtle Node)", OUTPUT_TURTLE_LUA),
        ("boot.lua (Universal Launcher)", OUTPUT_BOOT_LUA)
    ]:
        if os.path.exists(path):
            validate_syntax(path)

    print("\n" + "=" * 60)
    print("✨ Build & Version Synchronization Complete!")
    print(f"   🖥️  Computer Suite : {COMP_VER} (run.lua, fcc.lua, display.lua, fcc-cli.lua, boot.lua)")
    print(f"   🐢 Turtle Firmware: {TURT_VER} (turtle_startup.lua)")
    print("=" * 60)

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
        l = re.sub(r'"(\\.|[^"\\])*"', '""', line)
        l = re.sub(r"'(\\.|[^'\\])*'", "''", l)
        l = re.sub(r"--.*$", "", l)
        l = re.sub(r"\belseif\b.*?\bthen\b", " ", l)
        tokens = re.findall(r"\b(function|then|do|repeat|until|end)\b", l)
        for t in tokens:
            if t in ("function", "then", "do", "repeat"):
                stack.append((t, i))
            elif t == "end":
                if stack: stack.pop()
                else:
                    print(f"❌ Syntax Error: Unmatched 'end' in {os.path.basename(filepath)} at line {i}")
                    sys.exit(1)
            elif t == "until":
                if stack: stack.pop()
                else:
                    print(f"❌ Syntax Error: Unmatched 'until' in {os.path.basename(filepath)} at line {i}")
                    sys.exit(1)

    if stack:
        print(f"❌ Syntax Error in {os.path.basename(filepath)}: Unclosed blocks {stack}")
        sys.exit(1)
    else:
        print(f"✅ Syntax Validation Passed: All blocks in {os.path.basename(filepath)} are perfectly balanced!")

if __name__ == "__main__":
    if len(sys.argv) > 1:
        cmd = sys.argv[1].lower()
        if cmd in ("--version", "-v", "version"):
            print(get_version("computer"))
            sys.exit(0)
        elif cmd in ("--version-computer", "--version-comp"):
            print(get_version("computer"))
            sys.exit(0)
        elif cmd in ("--version-turtle", "--version-turt"):
            print(get_version("turtle"))
            sys.exit(0)
        elif cmd in ("--version-all", "-va"):
            vers = get_versions()
            print(f"Computer: {vers['computer']} | Turtle: {vers['turtle']}")
            sys.exit(0)
        elif cmd in ("--bump", "-b", "bump"):
            b_type = sys.argv[2].lower() if len(sys.argv) > 2 else "patch"
            new_vers = bump_versions("all", b_type)
            print(f"📦 Versions bumped to Computer: {new_vers['computer']} | Turtle: {new_vers['turtle']}")
            bundle()
            sys.exit(0)
        elif cmd in ("--bump-computer", "--bump-comp"):
            b_type = sys.argv[2].lower() if len(sys.argv) > 2 else "patch"
            new_vers = bump_versions("computer", b_type)
            print(f"📦 Computer version bumped to {new_vers['computer']}")
            bundle()
            sys.exit(0)
        elif cmd in ("--bump-turtle", "--bump-turt"):
            b_type = sys.argv[2].lower() if len(sys.argv) > 2 else "patch"
            new_vers = bump_versions("turtle", b_type)
            print(f"📦 Turtle version bumped to {new_vers['turtle']}")
            bundle()
            sys.exit(0)
        elif cmd in ("--set", "-s", "set"):
            if len(sys.argv) > 2:
                new_vers = set_version(sys.argv[2], "all")
                print(f"📦 Versions set to Computer: {new_vers['computer']} | Turtle: {new_vers['turtle']}")
                bundle()
                sys.exit(0)
            else:
                print("Error: Missing version argument. Usage: python bundle.py --set vX.Y.Z")
                sys.exit(1)
        elif cmd in ("--set-computer", "--set-comp"):
            if len(sys.argv) > 2:
                new_vers = set_version(sys.argv[2], "computer")
                print(f"📦 Computer version set to {new_vers['computer']}")
                bundle()
                sys.exit(0)
            else:
                print("Error: Missing version argument. Usage: python bundle.py --set-computer vX.Y.Z")
                sys.exit(1)
        elif cmd in ("--set-turtle", "--set-turt"):
            if len(sys.argv) > 2:
                new_vers = set_version(sys.argv[2], "turtle")
                print(f"📦 Turtle version set to {new_vers['turtle']}")
                bundle()
                sys.exit(0)
            else:
                print("Error: Missing version argument. Usage: python bundle.py --set-turtle vX.Y.Z")
                sys.exit(1)
    bundle()
