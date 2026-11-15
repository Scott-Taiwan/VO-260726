# photo_capture.py 使用說明

單純的「拍照 + 記錄 GPS」程式：**沒有 VO 計算、不等起飛高度**，相機一打開就開始連續拍攝，直到手動停止為止。

- 程式：`photo_capture.py`
- 啟動腳本：`run_photo_capture.sh`（不影響其他 `run_*.sh`）

## 1. 第一次使用（在 Jetson 上）

1. 把 `photo_capture.py` 和 `run_photo_capture.sh` 複製到和其他程式同一個資料夾（`gpsless_VO_image/`，需要裡面的 `config.py`）。
2. 給啟動腳本執行權限：
   ```bash
   chmod +x run_photo_capture.sh
   ```

## 2. 基本操作

```bash
./run_photo_capture.sh            # 用預設值開始拍攝（每 0.5 秒一張、TNR 關閉）
./run_photo_capture.sh status     # 查看是否在執行、log 檔在哪裡
./run_photo_capture.sh stop       # 停止拍攝（會正常釋放相機）
./run_photo_capture.sh help       # 列出所有參數
tail -f logs/photo_capture_*.log  # 即時看拍攝紀錄
```

- 程式在背景執行，SSH 斷線也不會停止。
- 如果有其他拍攝程式（`vo_flight_capture*`）正在執行，腳本會拒絕啟動，因為相機和序列埠只能被一個程式使用。請先用該程式自己的 `run_*.sh stop` 停止。

## 3. 參數

在啟動腳本後面加上參數即可，會覆蓋預設值：

```bash
./run_photo_capture.sh --exposure-ms 1 3                # 快門範圍 1–3 ms
./run_photo_capture.sh --exposure-ms 2 2                # 固定快門 2 ms（1/500 秒）
./run_photo_capture.sh --tnr-mode 1                     # 打開 TNR（降噪）
./run_photo_capture.sh --exposure-ms 2 2 --tnr-mode 0 --interval 1
```

| 參數 | 說明 | 預設值 |
|---|---|---|
| `--interval 秒` | 每張照片的間隔 | `0.5` |
| `--duration-s 秒` | 拍幾秒後自動停止；0 = 一直拍到手動停止 | `0` |
| `--exposure-ms LO HI` | **快門（曝光時間）範圍**，單位 ms；LO = HI 代表固定快門。硬體範圍 0.013–683.709 | `1 5` |
| `--gain LO HI` | 感光元件類比增益範圍，硬體範圍 1.0–10.625 | `1 10.625` |
| `--isp-gain LO HI` | ISP 數位增益範圍，1.0–256.0 | `1 4` |
| `--ev 值` | 曝光補償，-2.0 到 +2.0 | 不設定 |
| `--tnr-mode 0/1/2` | **TNR 時域降噪開關**：0 = 關閉、1 = 快速、2 = 高品質。移動拍攝建議關閉，否則會拖影 | `0`（關閉） |
| `--tnr-strength 值` | TNR 強度 -1.0 到 1.0（TNR 開啟時才有作用） | 不設定 |
| `--ee-mode 0/1/2` | 邊緣強化：0 = 關閉、1 = 快速、2 = 高品質 | 不設定（Argus 預設） |
| `--ee-strength 值` | 邊緣強化強度 -1.0 到 1.0 | 不設定 |
| `--format png/jpg` | 存檔格式：png 無損（約 1.5 MB/張），jpg 較小（約 0.3 MB/張） | `png` |
| `--jpeg-quality 值` | jpg 品質 1–100 | `95` |
| `--width` / `--height` | 影像解析度 | `1280` / `720` |
| `--port` / `--baud` | Pixhawk 序列埠與鮑率 | `/dev/ttyTHS1` / `57600` |
| `--no-mavlink` | 不連接 Pixhawk，只拍照（檔名為 `nogps`） | — |
| `--min-free-gb 值` | 剩餘磁碟空間低於此值時自動停止 | `2` |
| `--out-dir 路徑` | 存放照片的上層資料夾 | `photo_obtained/` |
| `--sensor-id` | CSI 相機編號 | `0` |

## 4. 輸出檔案

每次執行會建立一個新資料夾：`photo_obtained/<日期_時間>_capture/`

| 檔案 | 內容 |
|---|---|
| `CAP_*.png` | 照片（見下方檔名格式） |
| `capture_log.csv` | 每張照片一列：時間、檔名、GPS 經緯度、高度（MSL 與離地 AGL）、GPS 資料延遲、羅盤航向、roll/pitch/yaw、存檔耗時 |
| `run_args.txt` | 這次執行的全部參數與 GStreamer pipeline |

另外，終端機輸出會同時存到 `logs/photo_capture_<日期_時間>.log`，每張照片一行，例如：

```
[5] CAP_20261114_125606_457_25.0430281_121.4739233_67.19.png  25.0430281,121.4739233 msl=67.2m agl=60.0m  save=67ms
```

### 檔名格式

```
CAP_<年月日>_<時分秒>_<毫秒>_<緯度>_<經度>_<海拔高度MSL>.png
例：CAP_20261114_092227_500_25.0430236_121.4739233_37.32.png
```

- 還沒收到 GPS 時：`CAP_20261114_092227_500_nogps.png`
- GPS 資料超過 2 秒沒更新時，檔名仍用最後一筆位置，但 `capture_log.csv` 的 `gps_fresh` 欄位會是 0，log 也會標示 `STALE`。

## 5. 注意事項

- **磁碟用量**：png 每 0.5 秒一張，約 3 MB/秒、**每小時約 11 GB**。長時間拍攝建議用 `--format jpg`。剩餘空間低於 `--min-free-gb` 時程式會自動停止。
- **Pixhawk 沒有回應**時程式不會停，照樣拍照，只是檔名改為 `nogps`。
- **跟不上排程**：若存檔太慢，log 會出現 `running ... behind schedule` 警告，可改用 `--format jpg` 或拉長 `--interval`。
- **相機卡住**：連續 20 次抓不到影像會自動停止，請執行 `sudo systemctl restart nvargus-daemon` 後再試。
- 程式只讀取 Pixhawk 資料（加上和其他程式相同的 request_data_stream），不會寫入任何參數。
