# 📚 參考資料與周邊模組文件庫 (Reference Materials & Peripherals)

本目錄彙整本專案所依賴與參考之外部開源模組、周邊硬體原始碼與官方技術文件。

---

## 🌐 官方文件與線上參考 (Official Documentation Links)

| 項目 / 模組名稱 | 官方連結 | 核心用途與規格說明 |
| :--- | :--- | :--- |
| **Tom's Peripherals** | [Tom's Peripherals Wiki](https://github.com/tom5454/Toms-Peripherals/wiki) | • `tm_gpu` / `tm_monitor` / `gpu` 硬體全彩顯示周邊。<br>• 關鍵初始化時序：`setSize(64)` $\to$ `refreshSize()` $\to$ `sleep(0.1)` $\to$ `getSize()`。<br>• 支援 32-bit ARGB 緩衝區與 `lineS` 抗鋸齒平滑線條。 |
| **CC: Tweaked** | [CC: Tweaked Official Docs](https://tweaked.cc/) | • ComputerCraft: Tweaked 官方標準 API 手冊。<br>• `peripheral` 尋標、`monitor` 終端與高級螢幕 `blit` 渲染。<br>• `parallel` 協程並行、`redstone` 類比/數位訊號輸出、`modem` 通訊協議。 |
| **CC-DirectGPU-Mod** | [tiktop101/CC-DirectGPU-Mod](https://github.com/tiktop101/CC-DirectGPU-Mod) | • 高效能硬體加速 RGB 向量圖形渲染（每方塊高達 164×164 像素解析度）。<br>• 支援全彩多顯示器 `getDisplayInfo`、`fillRect`、`drawCircle`、`drawPolylines`、`drawText` 及 `directgpu_touch` 原生觸控事件。 |

---

## 🗂️ 本地參考庫 (Local Reference Clones)

- **`info/CC-DirectGPU-Mod/`**: DirectGPU Mod 開源倉庫，包含 DirectGPU API 實作與範例代碼。
- **`info/CreateAvionics/`**: Create 模組飛控與儀表參考代碼庫。
