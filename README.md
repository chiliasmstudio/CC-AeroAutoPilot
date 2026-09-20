# 🚀 Create: Avionics 四軸有線飛控系統 (Quad-Engine Avionics)

本專案專為 **Minecraft (Create: Aeronautics / Simulated + CC: Tweaked + Create: Avionics)** 設計。
採用 **「駕駛艙主電腦單一大腦 + 4 隻純硬體有線烏龜 (FL, FR, BL, BR 免跑程式)」** 的高可靠性四軸混控架構。

---

## 🌟 雙版本飛控程式 (最新發布)

本倉庫精簡保留兩個最完善的四軸飛控版本：

| 程式檔案 | 顯示技術 | 特色與推薦情境 |
| :--- | :--- | :--- |
| **[`quad_altitude.lua`](quad_altitude.lua)** | **CC 原生 Advanced Monitor** | **【主力推薦】**<br>• 100% 原生相容，自帶深色航空調色盤。<br>• **絕不干擾玩家滑鼠滾輪**（自由切換快捷欄）。<br>• 右鍵點擊螢幕按鈕即時觸控。 |
| **[`directgpu_quad_altitude.lua`](directgpu_quad_altitude.lua)** | **CC-DirectGPU-Mod** | **【高畫質全彩版】**<br>• 24-bit True RGB 真全彩渲染。<br>• 圓形高度儀表盤與平滑動態推力條。 |

---

## 🛠️ 硬體架構與四角佈局

```
                    ┌────────────────────────────────────────┐
                    │       駕駛艙 Advanced Computer         │
                    │  * 運行飛控主程式 (二選一)             │
                    │  * 連接 Monitor 螢幕 (觸控儀表)        │
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

## ⚙️ 快速安裝與使用步驟

### 步驟 1：放置與配置 4 隻烏龜 (執行 startup.lua)
1. 在飛艇四個角落的升降動力機構（變速箱/離合器）正上方或側面，各放置 1 隻普通 Turtle。
2. 在每隻烏龜側面貼上 **Wired Modem**，並**右鍵點擊數據機**（周圍亮起紅圈代表已聯網）。
3. 進入每隻烏龜的終端機，為牠們打上位置標籤並下載/建立 `startup.lua`：
   * **左前烏龜**：
     ```bash
     label set FL
     edit startup.lua   <-- 將本專案的 turtle_startup.lua 貼入儲存
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
   *(註：烏龜啟動後會顯示當前角色與推力，開機後永不需再觸碰！)*

### 步驟 2：連接網路纜線
* 使用 **Networking Cable** 將 4 隻烏龜的 Wired Modem 全部拉線連接到駕駛艙的 **主電腦 (Advanced Computer)**。

### 步驟 3：啟動飛控系統
在駕駛艙主電腦執行以下任一程式：
* **原生螢幕版（推薦）**：
  ```bash
  quad_altitude
  ```
* **DirectGPU 全彩版**：
  ```bash
  directgpu_quad_altitude
  ```

---

## 🎮 駕駛艙儀表操作指南

```text
┌────────────────────────────────────────────────────────┐
│              QUAD-ENGINE AVIONICS MASTER               │
├──────────────────────────┬─────────────────────────────┤
│ ALTITUDE & ATTITUDE      │ QUAD ENGINES (0-15)         │
│ ALT:   120.5 m           │ FL: 8   FR: 8               │
│ V.SPD:  +0.0 m/s         │ BL: 8   BR: 8               │
│ P: +0.0*  R: +0.0*       │ MODE: HOLD_ALT  BASE: 7     │
├──────────────────────────┴─────────────────────────────┤
│ STATUS: Holding Altitude & Balance                     │
├────────────────────────────────────────────────────────┤
│  [ +10m ]    [ +1m ]        [ -1m ]       [ -10m ]     │
│  [BASE +1]  [BASE -1]      [    AUTO CALIBRATE   ]     │
│                                                        │
│     [   HOLD ALT   ]             [  STOP / IDLE  ]     │
└────────────────────────────────────────────────────────┘
```

* **`AUTO CALIBRATE`**：初次起飛時點擊，系統會自動遞增推力，找到飛艇垂直速度為 0 的懸停基準檔位。
* **`+10m` / `+1m` / `-1m` / `-10m`**：調整目標飛行高度。
* **`BASE +1` / `BASE -1`**：手動微調四角基礎推力。
* **`HOLD ALT`**：啟動四軸 PID 高度鎖定與姿態自穩平衡。
* **`STOP / IDLE`**：緊急停機，立即關閉四角引擎推力（紅石歸 0）。
