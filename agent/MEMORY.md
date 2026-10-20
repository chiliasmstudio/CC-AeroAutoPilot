# 導航控制設計記錄

## 控制邊界

高度懸停控制沿用 `updateFlightLogic()` 的四軸升力 PID；導航只計算水平 `FWD`、`BWD`、`LEFT`、`RIGHT` 角色的推力，不直接改寫 `FL`、`FR`、`BL`、`BR`。電腦本體六個紅石面保持 0，避免升力信號外溢到未標記的引擎。

## 導航輸入與座標慣例

- 優先使用 CCPE `sensor_system.getBodyPosition()` / `getPosition()`、`getVelocity()` 與 `getAngles().yaw`。
- CCPE INS 航向以世界北/-Z 為 0°、順時針為正；內部 bearing 使用同一慣例：北 0°、東 90°。
- Create: Avionics `navigation_table.getHeading()` 回傳 `atan2(worldX, worldZ)`；讀取時轉成 `180° - heading`，得到北向 0°、順時針增加的內部慣例。
- GPS 只作低頻定位後備；水平速度以世界速度或位置差分估計。

## 導航行為與保護

- 先啟用高度保持才執行水平控制。航向誤差大於 12° 時只轉向，不前進。
- 導航速度限制在 3.5 格/秒，並根據距離和沿線速度提前減速；進入到點半徑後反向煞停，速度低於 0.3 格/秒才完成。
- CCPE 位置／航向超過 0.5 秒未更新時，或 GPS 位置超過 1.5 秒未更新時，水平輸出立即歸零。維持導航目標，恢復定位後續行。
- `go x y z` 同時設定高度；`go2 x z [radius]` 保持既有高度，預設半徑 12 格。
- `go n/e/s/w` 取消航點並原地鎖定北/東/南/西方位（0/90/180/270°），清掉前進與殘留轉向指令；需先處於 `HOLD_ALT`。
- CLI 手動巡航推力上限為 ±4，轉向上限為 ±2.5。

## 維護與部署

- 修改電腦端行為時依 `agent.md` 遞增 Computer Suite 版本；本導航屬功能增加，使用 minor 升版，再執行 `python bundle.py --bump-computer minor`。
- `run.lua`、`fcc.lua` 與 FCC 顯示／命令用戶端均需更新。若只更新 FCC 大腦而客戶端仍是舊版，遙測與命令文字可能不一致。
- 建置區塊平衡檢查通過仍不能替代遊戲內試飛；先低速、短距離確認座標軸、航向正負方向及各角色接線。
