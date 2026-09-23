# 🚀 VTOL Airship Flight Computer & Avionics Reference Document (`agent.md`)

本文檔彙整本專案（`E:\playground\auto pilot\`）之**開發與 Git 規範、官方參考資料、檔案模組化與打包編譯原理、模組硬體規格、顯示驅動 API、通訊協議、感測器驅動、物理混控演算法與雙尺寸自適應模型**。

---

## 📌 專案維護核心規範 (Mandatory Development & Git Policy)

> [!IMPORTANT]
> **每次修改皆必須提交 Git (Mandatory Git Commit on Every Modification)**：
> 1. 本專案目錄 `E:\playground\auto pilot\` 受 Git 版本控制管理。
> 2. **任何代碼、模組檔案（`modules/`）、打包主程式（`run.lua`）或文檔（`agent.md`）的變更與修復，在完成驗證後必須立即執行 `git add` 與 `git commit`。**
> 3. Commit 訊息應清楚說明變更的組件（如 `feat(avionics)`, `fix(renderer)`, `docs(agent)`）與修復重點。
> 4. 若編輯了 `modules/` 內的原始碼，必須執行 `python bundle.py` 重新生成並校驗 `run.lua`，確認語法平衡後一併提交。

---

## 📚 官方參考資料與文件庫 (Official Documentation & References)

本專案深度整合並依賴以下開源專案與官方技術文件：

| 專案 / 模組名稱 | 官方文件 / 儲存庫連結 | 核心用途與規格重點 |
| :--- | :--- | :--- |
| **Tom's Peripherals** | [Home · tom5454/Toms-Peripherals Wiki](https://github.com/tom5454/Toms-Peripherals/wiki) | • `tm_gpu` / `tm_monitor` / `gpu` 硬體全彩顯示周邊。<br>• 關鍵初始化時序：`setSize(64)` $\to$ `refreshSize()` $\to$ `sleep(0.1)` $\to$ `getSize()`。<br>• 支援 `lineS` 抗鋸齒平滑線條與 32-bit ARGB 緩衝區。 |
| **CC: Tweaked** | [CC: Tweaked Official Documentation](https://tweaked.cc/) | • ComputerCraft: Tweaked 官方標準 API 手冊。<br>• `peripheral` 尋標、`monitor` 終端與高級螢幕 `blit` 渲染。<br>• `parallel` 協程並行、`redstone` 類比/數位訊號輸出、`modem` 通訊協議。 |
| **CC-DirectGPU-Mod** | [tiktop101/CC-DirectGPU-Mod](https://github.com/tiktop101/CC-DirectGPU-Mod) | • 高效能硬體加速 RGB 向量圖形渲染（每方塊高達 164×164 像素解析度）。<br>• 支援全彩多顯示器 `getDisplayInfo`、`fillRect`、`drawCircle`、`drawPolylines`、`drawText` 及 `directgpu_touch` 原生觸控事件。 |

---

## 🗂️ 模組化檔案結構 (Modular Project Architecture)

專案結構劃分為**單檔獨立執行層 (`run.lua`)**、**模組原始碼層 (`modules/`)** 與 **打包建置腳本 (`bundle.py`)**：

```text
E:\playground\auto pilot\
├── run.lua                      # 🚀 統一主執行檔 (單檔直接執行，含完整後端與 3 大前端驅動)
├── bundle.py                    # 🛠️ 模組編譯打包與語法驗證工具 (Compile & Validate)
├── agent.md                     # 📖 技術規格、官方文件連結、合併原理與 API 參考手冊
├── README.md                    # 📖 專案說明與四軸有線烏龜配置指南
├── turtle_startup.lua           # 🐢 動力烏龜專用啟動程式 (FL, FR, BL, BR)
└── modules\                     # 📦 模組化原始碼目錄 (開發與維護層)
    ├── init.lua                 # 總模組載入器 (Unified Package Loader)
    ├── font_5x7.lua             # 5x7 點陣字體表 (ASCII 32~127)
    ├── flight_core.lua          # 飛控核心 (PID、高度/陀螺儀感測器、烏龜掃描、狀態機)
    └── drivers\                 # 顯示驅動層
        ├── directgpu.lua        # CC-DirectGPU-Mod 高解析全彩向量驅動
        ├── tom.lua              # Tom's Peripherals GPU 全彩點陣/向量驅動
        └── normal.lua           # CC: Tweaked 原生 Advanced Monitor & Terminal 驅動
```

---

## ⚙️ 檔案合併為 `run.lua` 之編譯原理 (`bundle.py`)

在 Minecraft CC: Tweaked 環境中，單一獨立 `.lua` 檔案（All-in-One Bundle）最方便透過指令或磁碟直接分發與執行；而在開發維護上，分散的模組化結構（Modular Files）能降低耦合度並提升可讀性。

本專案透過根目錄的 [`bundle.py`](file:///E:/playground/auto%20pilot/bundle.py) 實現自動化編譯與打包：

### 1. 合併與提取邏輯
1. **讀取 `modules/font_5x7.lua`**：提取字體點陣表 `FONT_5X7`。
2. **讀取 `modules/flight_core.lua`**：去除最底部的 `return FlightCore`，保留所有 PID、感測器、四軸混控與狀態機邏輯作為全域/局部後端。
3. **讀取 `modules/drivers/*.lua`**：
   * 去除各驅動頂部的 `require` 與 `package.loaded` 包裝。
   * 將各驅動本體分別綁定至 `Drivers.direct`、`Drivers.tom` 與 `Drivers.normal`。
4. **注入 Part 3 Orchestrator**：包含多工事件循環 `flightLoop`（20Hz 飛控）、`renderLoop`（20Hz 畫面刷新）、`eventLoop`（熱插拔與觸控事件路由）與 `selectBestDriver()` 最佳驅動探測器。
5. **輸出至 `run.lua`**：合併後直接寫入 `run.lua`。

### 2. 語法平衡與區塊自動校驗
`bundle.py` 內建 Lua Lexer 檢查器，在打包後自動解析 `function`, `if ... then`, `do`, `repeat ... until`, `end` 之區塊堆疊深度，確保編譯產出的 `run.lua` 100% 語法平衡且無遺漏 `end`。

---

## 📑 目錄
1. [系統整體架構 (System Architecture)](#1-系統整體架構-system-architecture)
2. [支援模組與環境清單 (Supported Mod Environment)](#2-支援模組與環境清單-supported-mod-environment)
3. [三級顯示驅動 API 規範 (Display Driver APIs)](#3-三級顯示驅動-api-規範-display-driver-apis)
   - [3.1 CC-DirectGPU-Mod 驅動 (`Drivers.direct`)](#31-cc-directgpu-mod-驅動-driversdirect)
   - [3.2 Tom's Peripherals GPU 驅動 (`Drivers.tom`)](#32-toms-peripherals-gpu-驅動-driverstom)
   - [3.3 CC: Tweaked 原生螢幕驅動 (`Drivers.normal`)](#33-cc-tweaked-原生螢幕驅動-driversnormal)
   - [3.4 雙尺寸自適應引擎 (Dual-Size Responsive Engine)](#34-雙尺寸自適應引擎-dual-size-responsive-engine)
4. [感測器整合 API 規範 (Sensor APIs)](#4-感測器整合-api-規範-sensor-apis)
   - [4.1 高度計與垂直速度計 (Altimeter / V.Speed)](#41-高度計與垂直速度計-altimeter--vspeed)
   - [4.2 姿態陀螺儀 / 萬向節 (Gimbal / Gyroscope / Compass)](#42-姿態陀螺儀--萬向節-gimbal--gyroscope--compass)
5. [四象限動力節點通訊協議 (Quad-Quadrant Modem Protocol)](#5-四象限動力節點通訊協議-quad-quadrant-modem-protocol)
   - [5.1 網路拓撲與通道分配](#51-網路拓撲與通道分配)
   - [5.2 封包格式 (Packet Schema)](#52-封包格式-packet-schema)
   - [5.3 動態掃描與心跳機制 (Heartbeat & Discovery)](#53-動態掃描與心跳機制-heartbeat--discovery)
6. [四軸物理 PID 混控演算法 (Flight Core Control Math)](#6-四軸物理-pid-混控演算法-flight-core-control-math)
   - [6.1 高度控制 PID 方程式](#61-高度控制-pid-方程式)
   - [6.2 姿態修正與四象限混控矩陣](#62-姿態修正與四象限混控矩陣)
   - [6.3 懸停基準推力自校準模型 (Auto-Calibration)](#63-懸停基準推力自校準模型-auto-calibration)
7. [多螢幕熱插拔與事件路由 (Hotplug & Event Routing)](#7-多螢幕熱插拔與事件路由-hotplug--event-routing)

---

## 1. 系統整體架構 (System Architecture)

本飛控系統採用 **「駕駛艙主電腦單一大腦 + 4 象限動力執行烏龜 (FL, FR, BL, BR)」** 的分散式容錯架構：

```text
                                  ┌──────────────────────────────────────────────┐
                                  │           駕駛艙 Advanced Computer           │
                                  │  * 運行 run.lua (協同多任務並行)             │
                                  │  * 渲染多螢幕儀表 (DirectGPU/Tom's/CC Mon)   │
                                  │  * 讀取高度計、姿態儀、計算四軸 PID 混控     │
                                  └──────────────────────┬───────────────────────┘
                                                         │ (Wired / Wireless Modem)
         ╔═══════════════════════════════════════════════╧═══════════════════════════════════════════════╗
         ║                         CC 網路通訊匯流排 (Networking Bus / Channels)                         ║
         ╚══════════════════╦════════════════════╦════════════════════╦════════════════════╦═════════════╝
                            ║                    ║                    ║                    ║
                            ▼                    ▼                    ▼                    ▼
                    ┌──────────────┐     ┌──────────────┐     ┌──────────────┐     ┌──────────────┐
                    │ Turtle (FL)  │     │ Turtle (FR)  │     │ Turtle (BL)  │     │ Turtle (BR)  │
                    │ 標籤: FL     │     │ 標籤: FR     │     │ 標籤: BL     │     │ 標籤: BR     │
                    │ (左前升降)   │     │ (右前升降)   │     │ (左後升降)   │     │ (右後升降)   │
                    └──────┬───────┘     └──────┬───────┘     └──────┬───────┘     └──────┬───────┘
                           │ 0~15 PWM           │ 0~15 PWM           │ 0~15 PWM           │ 0~15 PWM
                           ▼                    ▼                    ▼                    ▼
                      左前升降機構         右前升降機構         左後升降機構         右後升降機構
                     (變速箱/離合器)      (變速箱/離合器)      (變速箱/離合器)      (變速箱/離合器)
```

---

## 2. 支援模組與環境清單 (Supported Mod Environment)

| 模組名稱 | 最低版本 / 相容性 | 在本系統中的角色 |
| :--- | :--- | :--- |
| **CC: Tweaked** | 1.18.2+ / 1.20.1+ | 核心計算機、烏龜、紅石 API、有線/無線通訊、原生文字螢幕。 |
| **CC-DirectGPU-Mod** | 1.0+ | 高效能 24-bit RGB 向量圖形繪製、字體渲染、觸控事件處理。 |
| **Tom's Peripherals (Tom's Simple Storage / Peripherals)** | 0.8+ | 提供 `tm_gpu` / `tm_monitor` 全彩硬體螢幕、`lineS` 平滑線條、ARGB 緩衝。 |
| **Create: Aeronautics / Create: Simulated** | 任意相容版本 | 飛艇物理實體、旋翼/螺旋槳升降動力機構。 |
| **Create: Avionics / Plethora / Advanced Peripherals** | 任意相容版本 | 航空高度感測器 (Altimeter)、陀螺儀 (Gimbal / Inertial Sensor)。 |

---

## 3. 三級顯示驅動 API 規範 (Display Driver APIs)

系統啟動時由 `selectBestDriver()` 自動探測並優先啟用最高階的硬體驅動：
`DirectGPU` $\to$ `Tom's Peripherals GPU` $\to$ `CC: Tweaked Native Monitor/Terminal`。

### 3.1 CC-DirectGPU-Mod 驅動 (`Drivers.direct`)

DirectGPU 支援現代 24-bit RGB 圖形渲染，透過 `peripheral.wrap(name)` 取得 GPU 物件。

#### 核心 API 清單：
* `gpu.getDisplayInfo(displayId)`:
  * **回傳**：`{ pixelWidth = number, pixelHeight = number, ... }`
* `gpu.clear(displayId, r, g, b)`: 清空螢幕為指定 RGB 色彩。
* `gpu.fillRect(displayId, x, y, w, h, r, g, b)`: 填滿矩形區塊。
* `gpu.drawRect(displayId, x, y, w, h, r, g, b)`: 繪製空心矩形外框。
* `gpu.drawLine(displayId, x1, y1, x2, y2, r, g, b)`: 繪製直線。
* `gpu.drawCircle(displayId, cx, cy, r, r, g, b, filled)`: 繪製實心或空心圓形。
* `gpu.drawPolylines(displayId, points, r, g, b)`:
  * `points`: 座標陣列 `{{x1, y1}, {x2, y2}, ...}`。
* `gpu.drawText(displayId, text, x, y, r, g, b, fontName, fontSize, fontStyle)`:
  * `fontName`: `"Arial"` / `"Default"`
  * `fontSize`: 字體像素大小（如 `8`, `10`, `14`）
  * `fontStyle`: `"plain"` / `"bold"`

#### 事件監聽：
* `directgpu_touch`: `(p1: displayId, p2: x, p3: y, p4: button)`
* `directgpu_resize`: 觸發重新調整解析度。

---

### 3.2 Tom's Peripherals GPU 驅動 (`Drivers.tom`)

Tom's Peripherals 採用 32-bit ARGB 數值格式與內部顯存同步。

#### 色彩換算規格 (ARGB 32-bit Signed Handling)：
```lua
function toARGB(hexColor)
    -- hexColor: 0xRRGGBB 或 {r, g, b, [a]}
    local a = 255
    local r = math.floor(hexColor / 65536) % 256
    local g = math.floor(hexColor / 256) % 256
    local b = hexColor % 256
    local val = a * 16777216 + r * 65536 + g * 256 + b
    if val >= 2147483648 then val = val - 4294967296 end -- 轉為 Lua signed 32-bit int
    return val
end
```

#### 關鍵硬體初始化序列 (Hardware Initialization Sequence)：
Tom's GPU 在初次連接或包裝時，必須按照嚴格順序呼叫以取得真實像素解析度，否則 `getSize()` 會回傳未初始化的舊值（如 64×64 或區塊數）：
```lua
-- 1. 設定字元/圖塊尺寸基準
pcall(function() if gpu.setSize then gpu.setSize(64) end end)
-- 2. 刷新硬體顯存尺寸
pcall(function() if gpu.refreshSize then gpu.refreshSize() end end)
-- 3. 等待硬體內部更新完成 (至少 1 tick / 0.05~0.1s)
sleep(0.1)
-- 4. 讀取真實像素尺寸 (若小於 32 則依 128px/block 換算)
local ok, gw, gh = pcall(function() return gpu.getSize() end)
```

#### 硬體檢測與周邊包裝：
* 偵測類型：`tm_gpu`、`toms_gpu`、`gpu`、`tm_monitor`。
* 核心 API 驗證：檢查 `gpu.fill` 及 `gpu.filledRectangle`（部分版本無 `rectangle` 邊框 API）。

#### 核心 API 清單：
* `gpu.getSize()`:
  * **回傳**：`width, height`（像素解析度，如 $320\times 240$ 或 $200\times 200$）。
* `gpu.fill(argb)`: 填滿整個畫布。
* `gpu.filledRectangle(x, y, w, h, argb)`: 填滿實心矩形。
* `gpu.rectangle(x, y, w, h, argb)`: 繪製矩形邊框。
* `gpu.line(x1, y1, x2, y2, argb)`: 繪製一般直線。
* `gpu.lineS(x1, y1, x2, y2, argb)`: 繪製抗鋸齒/平滑直線。
* `gpu.sync()`: 將緩衝區畫面刷新至實體螢幕。

#### 內建點陣字體渲染 (`FONT_5X7`)：
Tom's GPU 無內建文字 API，系統內建 $5\times 7$ 點陣字體字典（ASCII 32~127），支援倍數縮放 (`scale`):
* 單字元寬度：$5 \times \text{scale}$ 像素，字距 $1 \times \text{scale}$ 像素（總步進 $6 \times \text{scale}$）。
* 單字元高度：$7 \times \text{scale}$ 像素。

#### 事件監聽：
* `tm_monitor_touch`: `(p1: monitor_name, p2: x, p3: y)`
* `tm_monitor_resize`: 螢幕尺寸改變事件。

---

### 3.3 CC: Tweaked 原生螢幕驅動 (`Drivers.normal`)

適用於 CC: Tweaked 官方的 Advanced Monitor 或主電腦 Terminal 終端機。

#### 核心 API 清單：
* `mon.setTextScale(scale)`: 設定字體縮放（本系統固定為 `0.5` 以提供最高資訊密度）。
* `mon.getSize()`: 回傳字元行列數 `(w, h)`。
* `mon.setBackgroundColor(color)` / `mon.setTextColor(color)`: 設定當前調色盤色彩。
* `mon.clear()` / `mon.setCursorPos(x, y)`: 清屏與游標定位。
* `mon.blit(text, textColors, backgroundColors)`: 高效能彩色字元批次渲染（16 色十六進位字串 `"0"`~`"f"`）。

#### 安全邊界 Blit (`safeBlit`) 規範：
自動截斷負座標及超出螢幕邊界的字元，防止 CC 報錯崩潰：
```lua
safeBlit(scr, x, y, text, fg, bg)
```

#### 事件監聽：
* `monitor_touch`: `(p1: monitor_side/name, p2: x, p3: y)`（右鍵點擊觸控）
* `mouse_click`: `(p1: button, p2: x, p3: y)`（終端機滑鼠點擊）
* `monitor_resize`: 螢幕結構變更。

---

### 3.4 雙尺寸自適應引擎 (Dual-Size Responsive Engine)

依據螢幕實體大小嚴格分為**精簡版本 (Compact Edition)** 與 **完整航電版本 (Full Avionics A350 Edition)**：

| 驅動類型 | 3x3 ~ 4x5 尺寸解析度 (精簡版本 Compact) | $\ge 5\times 5$ 尺寸解析度 (完整航電 Full) | `isFull` 判定門檻 |
| :--- | :--- | :--- | :--- |
| **Tom's Peripherals GPU** | • $3\times 3$: $192\times 192\text{px}$<br>• $4\times 4$: $256\times 256\text{px}$<br>• $5\times 4$: $320\times 256\text{px}$<br>• $4\times 5$: $256\times 320\text{px}$ | • $5\times 5$: $320\times 320\text{px}$<br>• $6\times 6+$: $\ge 384\times 384\text{px}$ | `sw >= 300 and sh >= 300` |
| **CC-DirectGPU-Mod** | • $3\times 3$: $492\times 492\text{px}$<br>• $4\times 4$: $656\times 656\text{px}$<br>• $5\times 4$: $820\times 656\text{px}$<br>• $4\times 5$: $656\times 820\text{px}$ | • $5\times 5$: $820\times 820\text{px}$<br>• $6\times 6+$: $\ge 984\times 984\text{px}$ | `sw >= 750 and sh >= 750` |
| **CC Native Monitor** | • $3\times 3$: $86\times 36$ 字元<br>• $4\times 4$: $116\times 48$ 字元<br>• $5\times 4$: $145\times 48$ 字元<br>• $4\times 5$: $116\times 60$ 字元 | • $5\times 5$: $145\times 60$ 字元<br>• $6\times 6+$: $\ge 174\times 72$ 字元 | `w >= 140 and h >= 55` |

#### 模式特性差異：
* **精簡版本 (Compact Edition)**：
  * $3\times 3$ 螢幕（欄寬 $colW \approx 90\text{px}$）：啟用緊湊卡片、雙排按鈕、簡寫狀態，絕不發生文字重疊或按鈕被截斷。
  * 底部按鈕鎖定為 2 排（第 1 排 4 個：`+10`, `-10`, `B+`, `B-`；第 2 排 2 個：`HOLD`, `STOP`）。
* **完整航電版本 (Full Avionics Edition, $\ge 5\times 5$)**：
  * 啟用大型 A350 刻度儀表盤、完整姿態水平球、四象限發動機獨立遙測。
  * 底部 3 排按鈕群（第 1 排 6 個快速升降鍵、第 2 排 4 個基準油門調整鍵、第 3 排 2 個操作模式鍵）。

---

## 4. 感測器整合 API 規範 (Sensor APIs)

系統於啟動及熱插拔時自動掃描周邊匯流排，綁定對應感測器。

### 4.1 高度計與垂直速度計 (Altimeter / V.Speed)
* **尋找方式**：`peripheral.find("altitude_sensor")` 或包裝名稱含有 `altimeter`, `sensor` 之周邊。
* **讀取方法**：
  * `altiSensor.getHeight()` 或 `altiSensor.getAltitude()` 或 `altiSensor.getY()`：取得當前海平面高度 (公尺)。
  * `altiSensor.getVerticalSpeed()` 或 `altiSensor.getVelocity()`：取得當前垂直升降速率 ($m/s$)。若硬體無直接提供，系統以時間差分 $\Delta y / \Delta t$ 動態計算。

### 4.2 姿態陀螺儀 / 萬向節 (Gimbal / Gyroscope / Compass)
* **尋找方式**：`peripheral.find("gimbal_sensor")` 或 `ship_helm`, `inertial_sensor`。
* **讀取方法**：
  * `gimbal.getPitch()`：俯仰角 (Pitch，單位：度，$\pm 90^\circ$)。
  * `gimbal.getRoll()`：滾轉角 (Roll，單位：度，$\pm 180^\circ$)。
  * `gimbal.getYaw()`：偏航角 (Yaw，單位：度，$0 \sim 360^\circ$)。

---

## 5. 四象限動力節點通訊協議 (Quad-Quadrant Modem Protocol)

### 5.1 網路拓撲與通道分配
* **控制廣播通道 (Master $\to$ Slaves)**：預設 `Channel 100`（或有線周邊直接呼叫 `callRemote`）。
* **狀態回報通道 (Slaves $\to$ Master)**：預設 `Channel 101`。
* **四象限代號**：
  * `FL` (Front-Left, 左前動力)
  * `FR` (Front-Right, 右前動力)
  * `BL` (Back-Left, 左後動力)
  * `BR` (Back-Right, 右後動力)

### 5.2 封包格式 (Packet Schema)

#### 1. 主電腦動力推力廣播指令 (Master $\to$ Slaves)
```json
{
  "protocol": "AVIONICS_V3",
  "type": "THRUST_CMD",
  "timestamp": 12485.25,
  "outputs": {
    "FL": 8.45,
    "FR": 8.45,
    "BL": 8.45,
    "BR": 8.45
  },
  "discretePWM": {
    "FL": 8,
    "FR": 8,
    "BL": 8,
    "BR": 8
  }
}
```

#### 2. 動力烏龜節點狀態回報 (Slave $\to$ Master)
```json
{
  "protocol": "AVIONICS_V3",
  "type": "NODE_HEARTBEAT",
  "slot": "FL",
  "turtleId": 12,
  "online": true,
  "appliedPWM": 8,
  "engineCount": 1,
  "timestamp": 12485.20
}
```

### 5.3 動態掃描與心跳機制 (Heartbeat & Discovery)
* 主電腦定期（每 $0.05\text{s}$）發送心跳探測與推力輸出。
* 超過 $1.5\text{s}$ 未收到某象限烏龜之回應，狀態標記為 `OFFLINE`（紅色警報），並在儀表上顯示降級告警。

---

## 6. 四軸物理 PID 混控演算法 (Flight Core Control Math)

### 6.1 高度控制 PID 方程式
高度誤差計算公式：
$$e(t) = Alt_{\text{target}} - Alt_{\text{current}}$$

垂直推力調整量計算：
$$\Delta T = K_p \cdot e(t) + K_i \int e(t)\,dt - K_d \cdot V_{\text{vertical}}$$

預設增益參數：
* $K_p = 0.35$（比例增益）
* $K_i = 0.02$（積分增益，具抗積分飽和 Clamp）
* $K_d = 0.85$（微分阻尼增益，有效抑制超調）

### 6.2 姿態修正與四象限混控矩陣
姿態平衡 PID 補償量：
$$P_{\text{pitch}} = K_{p,\text{gyro}} \cdot \text{Pitch} + K_{d,\text{gyro}} \cdot \dot{\text{Pitch}}$$
$$P_{\text{roll}} = K_{p,\text{gyro}} \cdot \text{Roll} + K_{d,\text{gyro}} \cdot \dot{\text{Roll}}$$

四象限虛擬推力混控分配公式（輸出範圍 $0.0 \sim 15.0$）：
$$\begin{aligned}
T_{\text{FL}} &= \text{clamp}(T_{\text{base}} + \Delta T - P_{\text{pitch}} - P_{\text{roll}}, 0, 15) \\
T_{\text{FR}} &= \text{clamp}(T_{\text{base}} + \Delta T - P_{\text{pitch}} + P_{\text{roll}}, 0, 15) \\
T_{\text{BL}} &= \text{clamp}(T_{\text{base}} + \Delta T + P_{\text{pitch}} - P_{\text{roll}}, 0, 15) \\
T_{\text{BR}} &= \text{clamp}(T_{\text{base}} + \Delta T + P_{\text{pitch}} + P_{\text{roll}}, 0, 15)
\end{aligned}$$

離散紅石整數輸出：
$$\text{PWM}_i = \text{round}(T_i) \quad (0 \le \text{PWM}_i \le 15)$$

### 6.3 懸停基準推力自校準模型 (Auto-Calibration)
當玩家按下 `[ CALIB 200m ]` 或執行校準時：
1. 飛艇爬升至校準高度（如 $200\text{m}$）。
2. 當垂直速度 $|V_{\text{vertical}}| \le 0.08\text{m/s}$ 連續維持 $2\text{s}$，記錄當前平衡推力為懸停基準推力 $T_{\text{base}}$。
3. 自動將當前高度鎖定為目標高度並切換至 `HOLD_ALT` 模式。

---

## 7. 多螢幕熱插拔與事件路由 (Hotplug & Event Routing)

主事件循環使用 `parallel.waitForAny(flightLoop, renderLoop, eventLoop)`：

```text
┌─────────────────┐       ┌─────────────────┐       ┌─────────────────┐
│   flightLoop    │       │   renderLoop    │       │    eventLoop    │
│  (20Hz 飛控循環)│       │  (20Hz 畫面刷新)│       │ (事件與觸控路由)│
└────────┬────────┘       └────────┬────────┘       └────────┬────────┘
         │                         │                         │
         ▼                         ▼                         ▼
   PID 混控與通訊            多螢幕平行渲染           熱插拔 & 座標命中判定
```

### 熱插拔生命週期保障：
1. 監聽 `peripheral`, `peripheral_detach`, `monitor_resize`, `tm_monitor_resize`, `directgpu_resize` 事件。
2. 觸發 `activeDriver:refreshScreens()`，重新獲取所有螢幕硬體物件與最新解析度。
3. **保留舊狀態**：在重建螢幕清單時，將原有螢幕 ID 對應的 `currentView` (當前視圖) 及 `isMenuOpen` (選單狀態) 繼承至新物件，避免玩家操作中斷。
4. 重新觸發 `FlightCore.scanQuadTurtles()` 確保動力節點連線無縫重建。

---
*文檔更新時間：2026-09-24 | 飛控核心版本：VTOL Avionics v3.8.2 Modular*
