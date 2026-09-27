# 🚀 CC-AeroAutoPilot: 四軸航空自動導航與多螢幕飛控系統

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

> [!NOTE]
> **🤖 AI 生成聲明 (AI-Generated Notice)**：
> 本專案中**絕大多數代碼（包括飛控演算法、多螢幕顯示驅動、點陣字型庫、打包工具與技術手冊）皆由 AI 生成與輔助迭代開發**。

**CC-AeroAutoPilot** 是專為 **Minecraft (Create: Aeronautics / Simulated + CC: Tweaked)** 設計的下一代垂直起降 (VTOL) 四軸飛艇航空級自動駕駛儀與儀表系統。

採用 **「駕駛艙主電腦單一大腦 + 4 隻有線烏龜 (FL, FR, BL, BR) 動力輸出」** 的高可靠性架構，支援 **Tom's Peripherals GPU**、**CC-DirectGPU-Mod** 與 **CC 原生進階螢幕** 3 大顯示技術，並具備全自動多螢幕並行渲染與觸控操作功能。

---

## ✨ 核心特色

1. **三合一通用顯示技術 (Universal 3-in-1 Driver)**：
   - 🎨 **Tom's Peripherals GPU**：高解析 32-bit ARGB 繪圖、抗鋸齒平滑儀表刻度盤與 5x7 點陣字型。
   - ⚡ **CC-DirectGPU-Mod**：硬體加速 24-bit True RGB 向量圖形渲染與原生觸控。
   - 🖥️ **CC: Tweaked Advanced Monitor**：16 色經典終端與外接高級螢幕鏡像。
2. **智慧雙尺寸自適應佈局 (Dual-Size Adaptive UI)**：
   - **精簡模式 (<5x5, 3x3 螢幕)**：專為緊湊空間優化，大按鈕與精準數值標籤。
   - **專業全景模式 (≥5x5 螢幕)**：雙即時儀表盤（高度升降剖面 + 姿態動力總覽）與 3 排完整飛行管理控制台。
3. **多螢幕獨立並行調度 (Multi-Screen Concurrency)**：
   - 駕駛艙可同時連接多個不同尺寸與類型的螢幕，各螢幕可獨立切換分頁與互動。
4. **四軸 PID 自動高度鎖定與姿態自穩 (Attitude & Altitude Hold)**：
   - 結合高度計 (Altitude Sensor) 與陀螺儀 (Gimbal Sensor)，自動計算四角引擎差動補償。
5. **一鍵自動起飛校準 (One-Touch Auto Calibration)**：
   - 自動遞增推力尋找懸停基準檔位，起飛無需手動計算重量。

---

## 🛠️ 硬體配置與接線指南

```text
                    ┌────────────────────────────────────────┐
                    │       駕駛艙 Advanced Computer         │
                    │  * 運行飛控主程式: run.lua             │
                    │  * 連接 Monitor / GPU 螢幕 (觸控儀表)  │
                    │  * 連接 Altitude Sensor (高度計)       │
                    │  * 連接 Gimbal Sensor (陀螺儀 - 可選)  │
                    └───────────────────┬────────────────────┘
                                        │ (Wired Modem 有線數據機)
        ╔═══════════════════════════════╧═══════════════════════════════╗
        ║                  CC Networking Cable (網路纜線)                ║
        ╚══════════╦════════════════╦════════════════╦════════════════╦═╝
                   ║                ║                ║                ║
                   ▼                ▼                ▼                ▼
           ┌──────────────┐ ┌──────────────┐ ┌──────────────┐ ┌──────────────┐
           │ Turtle (FL)  │ │ Turtle (FR)  │ │ Turtle (BL)  │ │ Turtle (BR)  │
           │ 標籤: FL     │ │ 標籤: FR     │ │ 標籤: BL     │ │ 標籤: BR     │
           │ (左前升降)   │ │ (右前升降)   │ │ (左後升降)   │ │ (右後升降)   │
           └──────┬───────┘ └──────┬───────┘ └──────┬───────┘ └──────┬───────┘
                  │ 0~15 紅石      │ 0~15 紅石      │ 0~15 紅石      │ 0~15 紅石
                  ▼                ▼                ▼                ▼
             左前動力機構     右前動力機構     左後動力機構     右後動力機構
            (變速箱/離合器)  (變速箱/離合器)  (變速箱/離合器)  (變速箱/離合器)
```

---

## 🚀 快速安裝與使用步驟

### 步驟 1：配置 4 隻動力烏龜
1. 在飛艇四角的升降動力機構正上方或側面放置 1 隻普通 Turtle。
2. 在每隻烏龜側面貼上 **Wired Modem**，並**右鍵點擊數據機**（周圍亮起紅圈代表已聯網）。
3. 進入每隻烏龜的終端機，打上位置標籤並設定啟動程式：
   * **左前烏龜**：
     ```bash
     label set FL
     edit startup.lua   <-- 貼入專案內的 turtle_startup.lua 內容
     reboot
     ```
   * **右前烏龜**：
     ```bash
     label set FR
     edit startup.lua   <-- 貼入 turtle_startup.lua
     reboot
     ```
   * **左後烏龜**：
     ```bash
     label set BL
     edit startup.lua   <-- 貼入 turtle_startup.lua
     reboot
     ```
   * **右後烏龜**：
     ```bash
     label set BR
     edit startup.lua   <-- 貼入 turtle_startup.lua
     reboot
     ```
   *(註：烏龜啟動後會顯示當前推力與角色，開機後即由主電腦接管！)*

### 步驟 2：連接網路纜線
* 使用 **CC Networking Cable** 將 4 隻烏龜的 Wired Modem、高度計、陀螺儀與外接螢幕全部拉線連接至駕駛艙的 **Advanced Computer**。

## 🚀 安裝與啟動 (萬能啟動器 boot.lua)

在 CC 電腦或烏龜上**只需執行 `boot.lua`** 即可完成全部模式的安裝與即時運行：

```bash
# 萬能指令 (直接載入互動選單):
wget run https://raw.githubusercontent.com/chiliasmstudio/CC-AeroAutoPilot/main/boot.lua
```
或下載後自由選擇：
```bash
wget https://raw.githubusercontent.com/chiliasmstudio/CC-AeroAutoPilot/main/boot.lua boot
boot
```

---

### 📋 互動選單架構

若執行 `boot` 未帶參數，將顯示清晰的階層式選擇介面：

```text
========================================
   CC-AeroAutoPilot Universal Booter    
========================================
 Select Action:
  [1] Install to Disk  (寫入磁碟 / 開機自啟 startup.lua)
  [2] Run Directly     (直接從 GitHub Release 執行)
========================================
```

- **選擇 [1] Install to Disk (寫入本機)**：
  - `[1] FCC Flight Core`：安裝分散式飛控大腦為 `startup.lua` (20Hz 純計算)
  - `[2] FCC-CLI Console`：安裝獨立命令終端為 `startup.lua` (輸入 `go 100 100` 等)
  - `[3] Cockpit Display`：安裝駕駛艙螢幕系統為 `startup.lua` (DirectGPU / 觸控儀表)
  - `[4] All-In-One Run`：安裝單機一體化全功能飛控為 `startup.lua`
  - `[5] Turtle Node`：安裝動力烏龜韌體為 `startup.lua` 並自動重啟

- **選擇 [2] Run Directly (直接運行最新版)**：
  - `[1] FCC-CLI Console` (命令終端)
  - `[2] FCC Flight Core` (飛控大腦)
  - `[3] Cockpit Display` (駕駛艙螢幕)
  - `[4] All-In-One Run` (單機一體化飛控)

---

### ⚡ 快捷指令（免進選單直接執行）

```bash
# 1. 一鍵安裝為開機自啟程式 (startup.lua):
boot install fcc      # 安裝 分散式飛控大腦
boot install cli      # 安裝 獨立命令列終端
boot install display  # 安裝 駕駛艙螢幕
boot install run      # 安裝 一體化飛控
boot install turtle   # 安裝 動力烏龜韌體並自動重啟

# 2. 直接從 GitHub Release 下載最新版運行:
boot cli              # 啟動 FCC-CLI 終端
boot fcc              # 啟動 FCC 飛控大腦
boot display          # 啟動 駕駛艙螢幕
boot run              # 啟動 一體化飛控
```

---

## 🎮 駕駛艙儀表操作指南

系統支援 6 大功能分頁（點擊頂部導航列即可切換）：

| 分頁代碼 | 頁面名稱 | 功能與儀表說明 |
| :---: | :--- | :--- |
| **`OVERVIEW`** | 主飛行概覽 | 顯示即時高度、垂直升降速度、姿態人工地平線與四象限動力小卡。 |
| **`PFD`** | 主飛行儀表 | 超大 A350 風格人工地平線、升降速帶與數位高度錶。 |
| **`ECAM`** | 發動機監控 | 四角升降與水平推進動力監控，顯示在線狀態與即時 PWM 輸出。 |
| **`CTRL`** | 飛行管理台 | 垂直高度微調、基礎懸停油門調整、模式切換控制。 |
| **`NAV`** | 航向與自駕 | 航向角鎖定 (Heading Hold)、航向微調、XZ 航點導航。 |
| **`SYS`** | 系統狀態診斷 | 檢視所有連接之螢幕型號、烏龜硬體在線率與感測器通訊診斷。 |

### 常用控制按鈕一覽
- **`[ CALIB HOVER ]`**：原處微升 +3m 自動測量懸停平衡推力並鎖定高度。
- **`[ HOLD ALT ]`**：以 PD-V 垂直速度阻尼演算法鎖定當前高度。
- **`+10m` / `-10m` / `+1m` / `-1m`**：增減目標飛行高度。
- **`BASE +0.1` / `BASE -0.1`**：微調基礎懸停油門。
- **`HDG +10` / `HDG -10`**：調整目標飛行航向角。
- **`[ HDG HOLD ]`**：啟動航向自動鎖定。
- **`[ STOP IDLE ]`**：緊急停機，立即將所有引擎推力歸零。

---

## 📦 開發與編譯打包 (`bundle.py`)

本專案採模組化開發架構，原始碼存於 `modules/` 目錄中。執行 `bundle.py` 會自動編譯打包並校驗所有發布程式：

```bash
python bundle.py
```
- **`run.lua`**：單機一體化飛控大腦。
- **`fcc.lua`**：分散式獨立飛控大腦 (FCC)。
- **`display.lua`**：分散式駕駛艙螢幕系統 (CDS)。
- **`boot.lua`**：通用 Release 自動下載與安裝啟動器。

---

## 📄 授權條款 (License)

本專案採用 [MIT License](LICENSE) 開源授權。任何人皆可自由使用、修改、分發與商業應用，惟須保留原著作權與授權聲明。

