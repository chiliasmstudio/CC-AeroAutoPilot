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

### 步驟 3：啟動飛控系統
將專案中的 [`run.lua`](run.lua) 放入主電腦中，直接執行：
```bash
run
```
*(系統會自動偵測所有已連接的 GPU 螢幕、一般螢幕與烏龜硬體，並啟動全彩航電儀表！)*

---

## 🎮 駕駛艙儀表操作指南

系統支援 5 大切換分頁（點擊頂部導航列即可切換）：

| 分頁代碼 | 頁面名稱 | 功能與儀表說明 |
| :---: | :--- | :--- |
| **`OVERVIEW`** | 主飛行概覽 | 顯示即時高度、垂直升降速度、陀螺儀姿態角與四象限動力小卡。 |
| **`ECAM`** | 發動機監控 | 四角動力大型圓形/長條動態儀表盤，顯示在線狀態與各馬達即時 PWM 輸出。 |
| **`CTRL`** | 飛行管理台 | 雙資訊卡（高度升降剖面 + 姿態平衡總覽）與 3 排飛行控制按鈕群。 |
| **`NAV`** | 預設高度導航 | 一鍵快速飛往預設高度（0m 著陸、80m 樹梢、150m 巡航、200m 校準、300m 高空）。 |
| **`SYS`** | 系統狀態診斷 | 檢視所有連接之螢幕型號、烏龜硬體在線率與感測器通訊診斷。 |

### 常用控制按鈕一覽
- **`[ AUTO CALIBRATE ]`**：初次起飛時點擊，系統自動遞增推力找到浮力平衡點並切入高度鎖定。
- **`+50m` / `+10m` / `+1m` / `-1m` / `-10m`**：快速增減目標飛行高度。
- **`BASE +1.0` / `BASE -1.0` / `BASE +0.1` / `BASE -0.1`**：手動微調四角引擎基礎油門檔位。
- **`[ HOLD ALTITUDE ]`**：啟動高度 PID 與姿態自穩平衡。
- **`[ STOP / IDLE ]`**：緊急停機，立即將所有引擎推力歸零。

---

## 📦 開發與編譯打包 (`bundle.py`)

本專案採模組化開發架構，原始碼存於 `modules/` 目錄中。若修改了模組原始碼，可透過隨附的 Python 建置腳本一鍵編譯合併為單一獨立執行的 `run.lua`：

```bash
# 在專案根目錄執行編譯與語法校驗
python bundle.py
```

- **詳細技術手冊、模組分工與 API 規格**：請參閱 [`wiki.md`](wiki.md)。
- **外部開源參考資料與技術規格**：請參閱 [`info/README.md`](info/README.md)。

---

## 📄 授權條款 (License)

本專案採用 [MIT License](LICENSE) 開源授權。任何人皆可自由使用、修改、分發與商業應用，惟須保留原著作權與授權聲明。
