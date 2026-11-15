# vo_flight_capture_fix / fix1 / fix2 比較

三支程式是逐步演進的關係，每一版都包含前一版的全部內容，舊版保持不動。**下一次飛行建議使用 fix2。**

## 功能比較

| | **fix** | **fix1** | **fix2** |
|---|---|---|---|
| **定位** | 可靠的資料記錄基準版 | 修正 VO 數學 | fix1 + 連拍挑最佳照片 |
| **相機降噪（TNR）** | Argus 預設（開啟），造成 11/14 的拖影 | 預設**關閉**（`--tnr-mode 0`），另新增 `--ee-mode` | 同 fix1 |
| **每次拍幾張** | 1 張 | 1 張 | **`--burst N`**（預設 3 張），挑最清晰的（`--pick sharpness`）或轉動最少的（`--pick gyro`） |
| **航向** | 起飛時取羅盤一次，之後逐段累加 VO 偏航角；會漂移，失敗的段落也看不到實際轉向（11/14 最後偏了 95°） | **每一張都直接用羅盤（ATTITUDE yaw）**，不再累加 | 同 fix1 |
| **傾斜（roll/pitch）** | 忽略，假設相機正對地面 | **補償**：用每張照片自己的 roll、pitch、航向與氣壓高度，把特徵點投影到地面 | 同 fix1，並使用被選中那張照片自己的時間戳 |
| **位移計算方式** | 影像中心的像素位移換算成公尺，再依 VO 航向旋轉 | 直接在地面座標（北/東，公尺）上擬合，並檢查比例 ≈ 1、旋轉 ≈ 0 | 同 fix1 |
| **讀取的遙測** | GLOBAL_POSITION_INT、VFR_HUD | 另加 **ATTITUDE**，20 Hz（`--attitude-hz`），取樣時間為拍照時間減去 `--camera-latency-ms` | 另加 ATTITUDE 的**角速度** |
| **送給 Pixhawk 的指令** | request_data_stream | 另加暫時性的 SET_MESSAGE_INTERVAL 要求 ATTITUDE（Pixhawk 重開機即清除，不寫入參數） | 同 fix1 |
| **VO 高度** | 由影像比例累加 | 氣壓高度：錨點高度 + AGL 變化 | 同 fix1 |
| **每段狀態判定** | OK / LOW-CONF / FAIL（依 inlier 數量與比例） | 另外：地面比例 ≠ 1 或旋轉 > 8° 時標為 LOW-CONF | 同 fix1 |
| **輸出檔案** | 照片、`vo_log.csv`、`run_args.txt` | 另加 `attitude_log.csv`，以及 `vo_log.csv` 新欄位（見下列） | 另加：`attitude_log.csv` 角速度欄位、連拍相關欄位、可選的 `burst/` 資料夾（`--save-burst`） |
| **CSV 新增欄位** | — | `roll_deg`、`pitch_deg`、`att_yaw_deg`、`ground_scale`、`ground_rot_deg`、舊方法結果（`raw_*`） | 另加 `burst_n`、`burst_pick`、`sharpness`、`gyro_dps`、`burst_sharpness`、`burst_gyro_dps` |
| **每張照片的 log 內容** | 結果 + `photo:` 檔名 | 結果 + roll/pitch/航向 + `photo:` 檔名 | 另加：選了第幾張、清晰度分數、`photo:` 檔名，以及沒被選用的連拍照片 |
| **啟動腳本** | `run_vo_flight_capture.sh` | 無（直接執行） | `run_vo_flight_capture_fix2.sh`（預設 `--interval 1 --burst 3 --save-burst`） |
| **啟動時必須收到 ATTITUDE** | 否 | **是**：10 秒內收不到就結束 | 是 |
| **每張照片的處理時間** | 1 張 + SIFT | 同左 + 地面投影（數毫秒） | 每多拍一張約多 0.07 秒 + 清晰度評分（數毫秒） |

## 各版本的測試狀況

| | **fix** | **fix1** | **fix2** |
|---|---|---|---|
| 實際飛行 | ✅ 11/14 | ❌ 尚未 | ❌ 尚未 |
| 合成幾何測試（已知傾斜、航向、位移） | — | ✅ 誤差約 0.01 m；同樣是純下降，舊方法會產生 7.4 m 的假位移 | 程式碼同 fix1 |
| 用 11/14 照片重播 | 實際飛行結果：最終誤差 17.4 m，最大 89.7 m | 由 GPS 估算傾斜：最終 15.9 m，最大 75.8 m | — |
| 端到端模擬（假相機 + 假遙測） | — | — | ✅ 兩種挑選方式，在 15 組可驗證的連拍中都選中了清晰的那張 |

## 參數用法

✔ = 該版本有此參數，— = 沒有。

### 飛行與連線

| 參數 | 說明 | 預設值 | fix | fix1 | fix2 |
|---|---|---|---|---|---|
| `--interval 秒` | 拍照間隔 | `2.0`（fix2 啟動腳本為 `1`） | ✔ | ✔ | ✔ |
| `--takeoff-altitude-m 公尺` | 離地高度超過此值才開始拍攝，並在此時重新設定錨點 | `30` | ✔ | ✔ | ✔ |
| `--land-altitude-m 公尺` | 起飛後離地高度連續 3 次低於此值即判定降落、停止程式 | `2` | ✔ | ✔ | ✔ |
| `--port` / `--baud` | Pixhawk 序列埠與鮑率 | `/dev/ttyTHS1` / `57600` | ✔ | ✔ | ✔ |
| `--sensor-id` | CSI 相機編號 | `0` | ✔ | ✔ | ✔ |
| `--out-dir 路徑` | 存放照片的上層資料夾，每次執行建立時間戳子資料夾 | `photo_obtained/` | ✔ | ✔ | ✔ |
| `--wave-seconds 秒` | 啟動自我檢查時，要晃動相機的秒數 | `3` | ✔ | ✔ | ✔ |

### 相機曝光與 ISP

| 參數 | 說明 | 預設值 | fix | fix1 | fix2 |
|---|---|---|---|---|---|
| `--exposure-ms LO HI` | 快門（曝光時間）範圍，單位 ms；LO = HI 為固定快門。硬體範圍 0.013–683.709 | `1 5` | ✔ | ✔ | ✔ |
| `--gain LO HI` | 感光元件類比增益範圍，1.0–10.625 | `1 10.625` | ✔ | ✔ | ✔ |
| `--isp-gain LO HI` | ISP 數位增益範圍，1.0–256.0 | `1 4` | ✔ | ✔ | ✔ |
| `--ev 值` | 曝光補償 -2.0 到 +2.0 | 不設定 | ✔ | ✔ | ✔ |
| `--tnr-mode 0/1/2` | TNR 時域降噪：0 = 關閉、1 = 快速、2 = 高品質 | `0`（關閉） | — | ✔ | ✔ |
| `--tnr-strength 值` | TNR 強度 -1.0 到 1.0（TNR 開啟時才有作用） | 不設定 | — | ✔ | ✔ |
| `--ee-mode 0/1/2` | 邊緣強化：0 = 關閉、1 = 快速、2 = 高品質 | 不設定（Argus 預設） | — | ✔ | ✔ |
| `--ee-strength 值` | 邊緣強化強度 -1.0 到 1.0 | 不設定 | — | ✔ | ✔ |

### 姿態（ATTITUDE）

| 參數 | 說明 | 預設值 | fix | fix1 | fix2 |
|---|---|---|---|---|---|
| `--attitude-hz 值` | 要求 Pixhawk 傳送 ATTITUDE 的頻率 | `20` | — | ✔ | ✔ |
| `--camera-latency-ms 值` | 相機延遲：姿態取樣時間 = 拍照時間減去此值。尚未校正，可之後用 `attitude_log.csv` 離線調整 | `0` | — | ✔ | ✔ |

### 連拍（只有 fix2）

| 參數 | 說明 | 預設值 | fix | fix1 | fix2 |
|---|---|---|---|---|---|
| `--burst N` | 每次拍幾張，挑一張做 VO；1 = 不連拍（等同 fix1），最多 10 | `3` | — | — | ✔ |
| `--pick sharpness/gyro` | 挑選方式：`sharpness` = 最清晰（Laplacian 變異數），`gyro` = 機身轉動最少（ATTITUDE 角速度） | `sharpness` | — | — | ✔ |
| `--save-burst` | 沒被選用的連拍照片也存到 `burst/` 子資料夾 | 不存（fix2 啟動腳本預設會存） | — | — | ✔ |

### 啟動腳本 `run_vo_flight_capture_fix2.sh`

- 內建預設：`--port /dev/ttyTHS1 --baud 57600 --interval 1 --burst 3 --pick sharpness --save-burst`
- 後面加的參數會覆蓋預設值。
- `--no-save-burst`：不存連拍中沒被選用的照片（這是啟動腳本自己的選項）。
- `status` 查看狀態、`stop` 停止。

### 使用範例

```bash
# fix（用舊的啟動腳本）
./run_vo_flight_capture.sh --interval 2 --exposure-ms 1 3

# fix1（直接執行）
python3 vo_flight_capture_fix1.py --port /dev/ttyTHS1 --interval 1 --tnr-mode 0
python3 vo_flight_capture_fix1.py --camera-latency-ms 60 --attitude-hz 25

# fix2（用新的啟動腳本）
./run_vo_flight_capture_fix2.sh                          # 建議設定
./run_vo_flight_capture_fix2.sh --interval 2             # 處理太慢時拉長間隔
./run_vo_flight_capture_fix2.sh --pick gyro --burst 5
./run_vo_flight_capture_fix2.sh --exposure-ms 2 2 --no-save-burst
```

## 該用哪一個

- **fix2**：下一次飛行使用，執行 `./run_vo_flight_capture_fix2.sh`，包含所有修正。
- **fix1**：想排除連拍挑選的影響，或 `--interval 1` 時處理時間太長的時候使用。效果等同 fix2 加 `--burst 1`。
- **fix**：只當作比較基準，或 Pixhawk 無法送出 ATTITUDE 時使用。

## 共同限制

三支程式都還假設相機未經校正：水平視角 77°、光學中心在影像正中央、沒有做鏡頭畸變修正。如果實際視角不同，所有距離都會差同一個比例。這次飛行之後，下一步提升精度的工作是相機校正（`FX_PIXELS`）。
