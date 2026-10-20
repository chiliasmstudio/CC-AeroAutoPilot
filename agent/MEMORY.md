# 航點導航故障記錄

## 症狀

`go 1000 100 1000` 不會前往預期的三維座標。舊 CLI 將其解析為 X=1000、Z=100、到達半徑=1000；此外，飛控在計算目標方位時呼叫不存在的 `math.atan2` 也會中止導航迴圈。高度懸停不會經過方位計算，因此仍可能正常。

## 根因

航點控制曾呼叫 `math.atan2(dx, -dz)`。CC:Tweaked 使用 Cobalt Lua；其 Lua 5.3 相容文件列出 `math.atan2` 已由支援雙參數形式的 `math.atan(y, x)` 取代。若執行環境沒有 `math.atan2`，第一個有效位置更新後便會在飛控迴圈中呼叫 nil。

另外，舊 CLI 僅提供 `go <x> <z> [radius]`，故三個數字的命令會把第三個數字當作到達半徑，且不會控制 Y 高度。

方位慣例：`0°` 為北（世界 `-Z`），`90°` 為東（世界 `+X`），因此兩參數反正切輸入為 `(dx, -dz)`。

## 修正

`modules/flight_core.lua` 使用 `local atan2 = math.atan2 or math.atan`，以兼容提供舊 API 的 Lua，以及 CC:Tweaked 的雙參數 `math.atan`。CLI 現在將 `go <x> <y> <z>` 解讀為三維座標並設定 Y 高度；`go2 <x> <z> [radius]` 保留二維航點與半徑設定。修改模組或 bundler 後必須執行 `python bundle.py`，以同步產生發布檔。

## 部署診斷

1. 確認正在執行新版本的 `fcc.lua`（分散式模式）或 `run.lua`（整合模式），而不是舊版 `startup.lua` 副本。
2. CLI 儀表應顯示有效 `POS` 座標與航點 `NAV` 狀態；沒有定位來源時，航點導航不會開始移動。
3. 若使用 CC:Tweaked GPS，需有可互相通訊且座標配置有效的四台 GPS host；官方文件建議先在控制器上執行 `gps locate` 確認定位。
4. 在遊戲中確認船體航向感測值會隨船轉動，並確認 `FWD`、`BWD`、`LEFT`、`RIGHT` 節點對應實際推進方向。

## 參考

- CC:Tweaked Lua 5.2/5.3 相容性：<https://www.tweaked.cc/reference/feature_compat.html>
- CC:Tweaked GPS API：<https://tweaked.cc/module/gps.html>
- CC:Tweaked GPS 設定指南：<https://tweaked.cc/guide/gps_setup.html>
