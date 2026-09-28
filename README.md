# 🚀 CC-AeroAutoPilot: 四軸航空自動導航與分散式多螢幕飛控系統

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Computer Suite: v4.0.12](https://img.shields.io/badge/Computer_Suite-v4.0.12-blue.svg)](https://github.com/chiliasmstudio/CC-AeroAutoPilot/releases/latest)
[![Turtle Firmware: v4.0.12](https://img.shields.io/badge/Turtle_Firmware-v4.0.12-green.svg)](https://github.com/chiliasmstudio/CC-AeroAutoPilot/releases/latest)

> [!NOTE]
> **🤖 AI 生成聲明 (AI-Generated Notice)**：
> 本專案中**絕大多數代碼（包括飛控演算法、多螢幕顯示驅動、點陣字型庫、打包工具與技術手冊）皆由 AI 生成與輔助迭代開發**。

**CC-AeroAutoPilot** 是專為 **Minecraft (Create: Aeronautics / Simulated + CC: Tweaked)** 設計的下一代垂直起降 (VTOL) 飛艇航空級自動駕駛儀、分散式飛控大腦與全方位儀表系統。

本專案支援 **「分散式多節點架構 (FCC + CDS + CLI)」** 與 **「單機一體化 (All-In-One)」** 兩種部署模式。全面整合 **CCPE (CC Peripheral Extender / FMC / AIC)** 物理實體感知系統、**全向發動機烏龜叢集 (升降 FL/FR/BL/BR、巡航 FWD/BWD、側推 LEFT/RIGHT)**，並支援 **Tom's Peripherals GPU**、**CC-DirectGPU-Mod** 與 **CC 原生進階螢幕** 3 大顯示技術。

---

## ✨ 核心特色

1. 🌐 **次世代分散式 3 層架構 (Distributed Multi-Node Architecture)**：
   - **FCC (Flight Control Computer)**：獨立純計算後台飛控大腦，0% 畫面渲染開銷，20Hz 極速高頻穩定 PD-V 垂直升力與姿態運算，絕不卡頓。
   - **FCC-CLI (Flight Command Console)**：可運行於任何 Pocket Computer 或終端機的獨立命令列控制台，上半部即時動態儀表，下半部雙向指令互動。
   - **CDS (Cockpit Display System)**：分散式駕駛艙螢幕系統，專職 GPU / 螢幕多工渲染與觸控操作，可外接多個獨立螢幕。
   - **All-In-One (單機一體化)**：亦可使用單台主電腦兼任飛控運算與螢幕渲染。
2. 🧭 **CCPE 物理實體感知與全自動導航 (CCPE / FMC / AIC Integration)**：
   - 完美相容 **CC Peripheral Extender (`ccpe.sensor_system`)**，實時讀取飛艇實體的絕對世界物理座標 $(X, Y, Z)$、三軸向量速度與歐拉角姿態（Pitch, Yaw, Roll）。
   - 支援 `go <x> <z>` 全自動航向對準、巡航推力控制、自適應進場減速與到達判定。
   - 具備多源導航融合與自動降級機制：優先 AIC/CCPE $\to$ 導航桌 (Navigation Table) $\to$ GPS $\to$ 航位推算 (INS)。
3. 🚢 **全向推進發動機叢集 (Full-Axis Propulsion & Cluster Redundancy)**：
   - **升降動力 (4-Quad Lift)**：`FL` (左前), `FR` (右前), `BL` (左後), `BR` (右後) 差動升力與四象限姿態自穩。
   - **巡航動力 (Cruise Thrust)**：`FWD` (前進推進), `BWD` (後退反推)。
   - **側推轉向動力 (Lateral & Steering)**：`LEFT` (左側推), `RIGHT` (右側推)。
   - 支援同角色多烏龜負載均衡（例如配置 2 隻 FWD 烏龜自動同步分擔推力）。
4. 🖥️ **三合一通用顯示技術與觸控支援 (Universal 3-in-1 Display Driver)**：
   - 🎨 **Tom's Peripherals GPU**：高解析 32-bit ARGB 繪圖、抗鋸齒平滑儀表刻度盤與 5x7 點陣字型。
   - ⚡ **CC-DirectGPU-Mod**：硬體加速 24-bit True RGB 向量圖形渲染與原生觸控。
   - 📺 **CC: Tweaked Advanced Monitor**：16 色經典終端與外接高級螢幕鏡像。
5. 📱 **雙尺寸自適應佈局 (Dual-Size Adaptive UI)**：
   - **精簡模式 (<5x5, 3x3 螢幕)**：專為緊湊空間優化，大按鈕與精準數值標籤。
   - **專業全景模式 (≥5x5 螢幕)**：雙即時儀表盤（高度升降剖面 + 姿態動力總覽）與 3 排完整飛行管理控制台。
6. 🔍 **即時硬體與設備庫存檢視 (Installed Systems & Hardware Inventory)**：
   - 在 GUI 螢幕 (`SYS` 分頁) 或終端命令 (`sys` / `hw`) 中即時檢視所有已安裝設備：DirectGPU 卡、Tom's GPU 螢幕數、各發動機在線數 (FL/FR/BL/BR/FWD/BWD/LEFT/RIGHT)、CCPE AIC 物理連結狀態、感測器狀態等。
7. 📡 **無線 OTA 韌體遠端更新與維護 (Over-The-Air OTA Updates)**：
   - 支援 `update turtles` 一鍵遠端無線 OTA 推送最新韌體至全艦所有烏龜。
   - 支援 `reboot turtles` 遠端一鍵重啟所有發動機節點。
8. 🚀 **萬能開機啟動與自動安裝器 (`boot.lua`)**：
   - 自動自 GitHub Releases 下載最新版本並支援階層式互動選單與一鍵安裝開機自啟。

---

## 🛠️ 硬體架構與接線圖

```text
       ┌────────────────────────────────────────────────────────┐
       │             分散式飛行控制系統架構總覽                 │
       └───────────────────────────┬────────────────────────────┘
                                   │
     ┌─────────────────────────────┼─────────────────────────────┐
     ▼                             ▼                             ▼
┌────────────────────────┐  ┌────────────────────────┐  ┌────────────────────────┐
│  FCC 獨立飛控大腦      │  │  CDS 駕駛艙螢幕系統    │  │  FCC-CLI 命令列終端    │
│  * 運行 fcc.lua        │  │  * 運行 display.lua    │  │  * 運行 fcc-cli.lua    │
│  * 20Hz 純計算後台     │  │  * 接 DirectGPU / Tom's│  │  * Pocket Computer /   │
│  * 連接 CCPE / 感測器  │  │  * 觸控儀表與即時圖形  │  │    終端機 (go x z 指令) │
│  * 廣播 Ch 102 遙測    │  │  * 接收 Ch 102 / 發 103│  │  * 接收 Ch 102 / 發 103│
└───────────┬────────────┘  └───────────┬────────────┘  └───────────┬────────────┘
            │                           │                           │
════════════╧═══════════════════════════╧═══════════════════════════╧═════════════════
                CC Wired Modem (有線網路纜線) 或 Wireless Modem (無線數據機)
════════════╦═══════════════════════════╦═══════════════════════════╦═════════════════
            │                           │                           │
            ▼                           ▼                           ▼
 ┌──────────────────────┐    ┌──────────────────────┐    ┌──────────────────────┐
 │  四軸升降動力烏龜組  │    │  巡航推進動力烏龜組  │    │  側推轉向動力烏龜組  │
 │  * FL (左前升降)     │    │  * FWD (前進推力)    │    │  * LEFT (左側推力)   │
 │  * FR (右前升降)     │    │  * BWD (後退反推)    │    │  * RIGHT (右側推力)  │
 │  * BL (左後升降)     │    │                      │    │                      │
 │  * BR (右後升降)     │    │                      │    │                      │
 └──────────────────────┘    └──────────────────────┘    └──────────────────────┘
```

---

## 🚀 快速安裝指南 (萬能啟動器 `boot.lua`)

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

### 📋 互動選單說明

執行 `boot` 後將顯示清晰的選單：

```text
========================================
   CC-AeroAutoPilot Universal Booter    
========================================
 Select Action:
  [1] Install to Disk  (寫入磁碟 / 開機自啟 startup.lua)
  [2] Run Directly     (直接從 GitHub Release 執行)
========================================
```

- **選擇 [1] Install to Disk (安裝為本機 `startup.lua`)**：
  - `[1] FCC Flight Core`：安裝分散式飛控大腦 (20Hz 純計算後台)
  - `[2] FCC-CLI Console`：安裝獨立命令終端 (輸入 `go 100 100` 等)
  - `[3] Cockpit Display`：安裝駕駛艙螢幕系統 (DirectGPU / 觸控儀表)
  - `[4] All-In-One Run`：安裝單機一體化全功能飛控
  - `[5] Turtle Node`：安裝動力烏龜韌體並自動重啟

---

### ⚡ 快捷指令（免進入選單直接執行）

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

## 🐢 發動機烏龜配置與標籤設定

在各發動機烏龜上設定標籤（Label）後，執行 `boot install turtle` 即可：

| 烏龜標籤 (Label) | 發動機角色 | 說明 |
| :--- | :--- | :--- |
| **`FL`** | 左前升降發動機 (Front-Left Lift) | 控制左前方垂直升降動力 |
| **`FR`** | 右前升降發動機 (Front-Right Lift) | 控制右前方垂直升降動力 |
| **`BL`** | 左後升降發動機 (Back-Left Lift) | 控制左後方垂直升降動力 |
| **`BR`** | 右後升降發動機 (Back-Right Lift) | 控制右後方垂直升降動力 |
| **`FWD`** | 前進推進發動機 (Forward Thrust) | 控制前進水平推力（自駕航點與前進加速） |
| **`BWD`** | 後退反推發動機 (Backward Thrust) | 控制後退反向推力與減速制動 |
| **`LEFT`** | 左側側推發動機 (Left Lateral Thrust) | 控制船體向左平移動力 |
| **`RIGHT`** | 右側側推發動機 (Right Lateral Thrust) | 控制船體向右平移動力 |

> [!TIP]
> 烏龜標籤設定方式：進入烏龜終端輸入 `label set FL`（或對應角色名稱），隨後執行 `boot install turtle` 即可自動完成韌體安裝並重啟接管。

---

## 💻 FCC-CLI 終端指令參考手冊

在 `fcc-cli` 終端中輸入以下指令即可對飛艇進行全方位控制：

```text
=== 核心飛控指令表 ===
go <x> <z> [radius]  - 自動導航至指定 X Z 座標 (預設到達半徑 20m)
cancel               - 取消目前自動導航航點
alt <alt>            - 設定目標高度 (例如: alt 200 或相對高度 alt +50, alt -20)
hold                 - 鎖定當前高度 (啟動 PD-V 高度維持模式)
calib                - 啟動原處起飛自動校準 (尋找懸停平衡推力)
lock                 - 快速鎖定當前高度
stop                 - 緊急停機 (所有引擎推力立即歸零)
hdg <0-359>          - 鎖定目標航向角 (例如: hdg 90 代表正東)
synchdg              - 將目標航向同步至當前實際朝向
fwd <val>            - 手動設定前進/後退推力 (-15.0 ~ 15.0)
turn <val>           - 手動設定偏航轉向推力 (-15.0 ~ 15.0)
sys / hw             - 檢視全艦已安裝硬體設備清單 (GPU/螢幕/烏龜/CCPE/感測器)
rescan               - 命令飛控電腦重新掃描所有硬體與感測器
update turtles       - 一鍵無線 OTA 遠端更新全艦所有烏龜韌體
reboot turtles       - 遠端一鍵重啟全艦所有發動機烏龜
clear                - 清空下方命令列視窗
exit                 - 退出終端
```

---

## 🎮 駕駛艙儀表操作指南 (CDS GUI)

系統支援 6 大功能分頁（點擊頂部導航列即可切換）：

| 分頁代碼 | 頁面名稱 | 功能與儀表說明 |
| :---: | :--- | :--- |
| **`OVERVIEW`** | 主飛行概覽 | 顯示即時高度、垂直升降速度、姿態人工地平線與四象限動力狀態卡。 |
| **`PFD`** | 主飛行儀表 | 超大 A350 風格人工地平線、升降速帶與數位高度錶。 |
| **`ECAM`** | 發動機監控 | 四角升降、前後巡航與左右側推動力監控，顯示在線健康度與即時 PWM 輸出。 |
| **`CTRL`** | 飛行管理台 | 垂直高度微調、基礎懸停油門調整、模式切換控制。 |
| **`NAV`** | 航向與自駕 | 航向角鎖定 (Heading Hold)、航向微調、XZ 航點導航與座標顯示。 |
| **`SYS`** | 設備庫存與診斷 | **全新設備清單介面**：顯示 DirectGPU 卡、Tom's GPU 數、原生螢幕數、各發動機在線數與 CCPE AIC 物理實體連結狀態。 |

---

## 📦 開發與編譯打包 (`bundle.py`)

本專案採模組化開發架構，原始碼存於 `modules/` 目錄中。執行 `bundle.py` 會自動編譯打包並校驗所有發布程式：

```bash
python bundle.py
```
編譯產出之發布資產：
- **`run.lua`**：單機一體化飛控大腦。
- **`fcc.lua`**：分散式獨立飛控大腦 (FCC)。
- **`display.lua`**：分散式駕駛艙螢幕系統 (CDS)。
- **`fcc-cli.lua`**：獨立命令列終端與儀表 (FCC-CLI)。
- **`turtle_startup.lua`**：發動機烏龜通用韌體。
- **`boot.lua`**：通用 Release 自動下載與安裝啟動器。

---

## 📄 授權條款 (License)

本專案採用 [MIT License](LICENSE) 開源授權。任何人皆可自由使用、修改、分發與商業應用，惟須保留原著作權與授權聲明。
