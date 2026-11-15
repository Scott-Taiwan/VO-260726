# vo_flight_capture_fix3 使用說明

`vo_flight_capture_fix3.py` 是飛行中的 VO(視覺里程計)資料記錄程式。它從 Pixhawk 讀取 GPS、高度與姿態,用朝下的 CSI 相機定時拍照,計算相鄰兩張照片之間的位移,並把照片與每一段的計算結果存下來,供飛行後和真實 GPS 比對。

程式只讀取 Pixhawk 的遙測,不會把位置送回 Pixhawk,也不會寫入任何參數。飛行仍由真實 GPS 控制。

fix3 是 fix2 加上 2026-11-15 code review(`Fable_review.md`)找到的問題修正。fix、fix1、fix2 都保持原樣沒有更動。

**驗證狀況**:fix3 目前只在 PC 上用 11/14 的照片加模擬的相機與 Pixhawk 跑過完整流程,還沒有在 Jetson 上實際執行或飛行過。第一次使用請先做第 2 節的地面測試。

---

## 1. 快速開始

在 Jetson 上,進入程式資料夾後執行:

```bash
cd ~/claude-project/gpsless_VO_run1/gpsless_VO_image
./run_vo_flight_capture_fix3.sh
```

啟動腳本內建的設定是 `--port /dev/ttyTHS1 --baud 57600 --interval 1 --burst 3 --pick sharpness --save-burst`。

啟動後的流程:

1. 連線 Pixhawk,確認有 GPS 位置與 ATTITUDE 訊息,並量測 ATTITUDE 的實際頻率(約 2 秒)。
2. 開啟相機,做相機自我檢查。畫面出現 `Now ROTATE / MOVE the camera for 3 seconds...` 時,**請用手晃動或轉動機身**,讓相機看到不同的畫面。
3. 出現 `Camera self-check PASSED` 後,程式等待起飛。
4. 離地高度超過 30 m 時,以當下的 GPS 位置與航向作為起點,開始拍照。
5. 離地高度連續 3 次低於 2 m 時判定降落,自動停止。
6. 看到下面這一行之後才可以斷電:

```
All files are synced to disk -- safe to power off.
```

其他指令:

```bash
./run_vo_flight_capture_fix3.sh status   # 是否在執行、PID、log 檔位置
./run_vo_flight_capture_fix3.sh stop     # 提前停止(會等照片寫完,最多 60 秒)
tail -f logs/vo_flight_capture_fix3_*.log
```

---

## 2. 第一次使用前的地面測試

fix3 還沒在 Jetson 上跑過,建議飛行前先在地面確認三件事。用較低的起飛高度讓程式在地面就開始拍:

```bash
./run_vo_flight_capture_fix3.sh --takeoff-altitude-m -5 --land-altitude-m -10
```

讓它跑一分鐘左右,手持機身慢慢移動,然後檢查:

| 檢查項目 | 怎麼看 | 不正常時 |
|---|---|---|
| ATTITUDE 頻率 | 啟動時的 `ATTITUDE rate: xx Hz received (requested 20 Hz)` 應接近 20 | 見第 6 節 |
| 處理速度 | log 不應反覆出現 `processing this frame took ... longer than --interval` | 加 `--sift-features 2000`,或把 `--interval` 拉長 |
| 停止是否正常 | 執行 `./run_vo_flight_capture_fix3.sh stop`,應在幾秒內顯示 `Stopped.`,log 最後有 `safe to power off` | 如果出現 `Force-killed`,請回報 |

測試完到 `photo_obtained/<時間>/` 確認照片大小都不是 0。

---

## 3. fix3 相對於 fix2 的修改

編號 B1–B20 對應 `Fable_review.md` 的項目。

| # | 問題 | fix2 的行為 | fix3 的修改 |
|---|---|---|---|
| B1 | 飛行最後幾秒的照片是 0 byte | 寫檔後沒有同步到磁碟,降落後馬上斷電就遺失(三趟飛行各掉 5–16 張) | 照片由背景執行緒寫入,每 10 張執行一次 `os.sync()`,結束時再同步一次,並印出可以斷電的訊息 |
| B2 | 明顯錯誤的航段照樣套用到位置 | 地面比例或旋轉不合理的航段標為 `LOW-CONF`,位置仍然更新;沒有速度檢查;12 個 inlier 就算可用 | 速度超過 `--max-speed-mps`、地面比例不在 0.85–1.15、或地面旋轉超過 8 度的航段標為 **`REJECT`**,位置不動,只增加 `hacc`。最少 inlier 數改為 `--min-inliers`(預設 20) |
| B3 | `stop` 停不下來,最後都被 `kill -9` | 腳本在背景啟動的程式會忽略 SIGINT,相機沒有正常釋放 | 程式自己處理 SIGINT 與 SIGTERM。啟動腳本的 `stop` 改送 SIGTERM,等待時間由 10 秒改為 60 秒 |
| B5 | 低對比照片找不到特徵點 | SIFT 用 OpenCV 預設門檻 0.04,11/14 的照片每張只有約 500 個特徵點 | 門檻改為 `--sift-contrast`,預設 0.01 |
| B6 | 相機視角寫死在程式裡 | 要改視角必須改程式 | 新增 `--hfov-deg`(預設仍是 77 度,尚未校正) |
| B7 | 高度為 0 或負值時算錯 | RANSAC 的門檻會變成負數 | 計算用的高度下限固定為 1 m |
| B8 | 同一秒的兩張照片排序會顛倒 | 檔名時間只到秒,而且是存檔時間 | 檔名時間加上毫秒,並改用該張照片自己的拍攝時間 |
| B9 | ATTITUDE 頻率沒有確認 | 只確認「有收到」 | 啟動時量測實際頻率,低於要求值的一半就印警告 |
| B10 | 每張照片的運算太多 | 前一張照片的特徵點每次都重算一遍;存檔會卡住拍攝迴圈 | 每張照片的特徵點只算一次並重用;偵測器只建立一次;存檔移到背景執行緒 |
| B20 | 啟動等待中按 Ctrl+C 沒有關檔 | `attitude_log.csv` 與遙測執行緒沒有收尾 | 會正常關閉 |
| — | 檔名中的高度會跳動 | 檔名用海平面高度,帶有 GPS 垂直誤差 | 檔名中的兩個高度改為相對起飛點的高度(見 5.1 節);`vo_log.csv` 不變 |

用 11/14 第二趟(093945)的照片重跑特徵匹配的結果:

| 設定 | 可用航段 |
|---|---|
| 原本(門檻 0.04、至少 12 個 inlier) | 31/72(43%) |
| fix3 預設(門檻 0.01、至少 20 個 inlier) | 51/72(71%) |

這個數字只代表「能算出位移的航段變多」,不代表位移更準。

**fix3 沒有處理的項目**

- 照片與姿態的時間戳仍用 Jetson 收到資料的時間,相機延遲 `--camera-latency-ms` 預設為 0,尚未校正(B9、B21)。
- 相機自我檢查仍需要有人晃動相機,所以不能用 systemd service 開機自動啟動(B4)。
- 相機視角(`FX_PIXELS`)尚未校正,所有距離可能有一個固定比例的誤差(B6)。
- 速度檢查擋不住機身傾斜造成的假位移(1 秒內 10 m 的假位移只有 10 m/s,和正常巡航分不開)。這要靠傾斜補償,而傾斜補償至今沒有實飛驗證過。
- `live_vo_gps.py` 與其他舊程式、離線分析工具的問題都沒有動。

---

## 4. 參數

所有參數都可以接在啟動腳本後面,會覆蓋腳本的內建設定。例如 `./run_vo_flight_capture_fix3.sh --interval 2`。

### 4.1 fix3 新增的參數

| 參數 | 說明 | 預設值 |
|---|---|---|
| `--sift-contrast 值` | SIFT 對比門檻。越低,在低對比地面上找到的特徵點越多 | `0.01` |
| `--sift-features 數量` | 每張照片最多取幾個特徵點。數量越多越慢 | `4000` |
| `--min-inliers 數量` | 一段位移至少要有幾個 inlier 才算可用 | `20` |
| `--max-speed-mps 值` | 換算速度超過此值的航段判為 `REJECT`。設 `0` 關閉檢查 | `20` |
| `--hfov-deg 角度` | 1280x720 影像的水平視角 | `77` |

### 4.2 飛行與連線

| 參數 | 說明 | 預設值 |
|---|---|---|
| `--interval 秒` | 拍照間隔 | `2`(啟動腳本為 `1`) |
| `--takeoff-altitude-m 公尺` | 離地高度超過此值才開始拍,並以當時位置為起點 | `30` |
| `--land-altitude-m 公尺` | 離地高度連續 3 次低於此值即停止 | `2` |
| `--port` / `--baud` | Pixhawk 序列埠與鮑率 | `/dev/ttyTHS1` / `57600` |
| `--sensor-id` | CSI 相機編號 | `0` |
| `--out-dir 路徑` | 存放照片的上層資料夾 | `photo_obtained/` |
| `--wave-seconds 秒` | 自我檢查時晃動相機的秒數 | `3` |

### 4.3 姿態與連拍

| 參數 | 說明 | 預設值 |
|---|---|---|
| `--attitude-hz 值` | 要求 Pixhawk 送 ATTITUDE 的頻率 | `20` |
| `--camera-latency-ms 值` | 相機延遲。姿態取樣時間 = 拍照時間減去此值 | `0` |
| `--burst N` | 每次連拍幾張,挑一張做 VO(1–10,1 為不連拍) | `3` |
| `--pick sharpness/gyro` | 挑選方式:最清晰,或機身角速度最小 | `sharpness` |
| `--save-burst` | 沒被選中的連拍照片也存到 `burst/` | 腳本預設開啟 |
| `--no-save-burst` | 不存沒被選中的連拍照片(啟動腳本專用) | — |

### 4.4 相機曝光與 ISP

| 參數 | 說明 | 預設值 |
|---|---|---|
| `--exposure-ms LO HI` | 快門時間範圍(ms)。LO 等於 HI 即固定快門 | `1 5` |
| `--gain LO HI` | 感光元件類比增益範圍,1.0–10.625 | `1 10.625` |
| `--isp-gain LO HI` | ISP 數位增益範圍,1.0–256.0 | `1 4` |
| `--ev 值` | 曝光補償,-2.0 到 +2.0 | 不設定 |
| `--tnr-mode 0/1/2` | 時域降噪:0 關閉、1 快速、2 高品質 | `0` |
| `--tnr-strength 值` | 時域降噪強度,-1.0 到 1.0 | 不設定 |
| `--ee-mode 0/1/2` | 邊緣強化:0 關閉、1 快速、2 高品質 | 不設定 |
| `--ee-strength 值` | 邊緣強化強度,-1.0 到 1.0 | 不設定 |

### 4.5 使用範例

```bash
# 建議的預設設定
./run_vo_flight_capture_fix3.sh

# 處理速度跟不上 1 秒間隔時
./run_vo_flight_capture_fix3.sh --sift-features 2000

# 白天壓低增益,減少雜訊與降噪造成的塗抹
./run_vo_flight_capture_fix3.sh --gain 1 2 --isp-gain 1 1

# 固定 2 ms 快門,不存沒被選中的連拍照片
./run_vo_flight_capture_fix3.sh --exposure-ms 2 2 --no-save-burst

# 不透過啟動腳本,直接執行
python3 vo_flight_capture_fix3.py --port /dev/ttyTHS1 --baud 57600 --interval 1
```

---

## 5. 輸出檔案

每次執行會建立 `photo_obtained/<yyyymmdd_hhmmss>/`,內容如下:

| 檔案 | 內容 |
|---|---|
| `VO_*.png` | 每次拍攝被選中的那張照片 |
| `burst/` | 沒被選中的連拍照片(有 `--save-burst` 時) |
| `vo_log.csv` | 每張照片一列:狀態、真實 GPS、VO 位置、位移、匹配數、姿態等 |
| `attitude_log.csv` | 收到的每一筆 ATTITUDE(姿態與角速度) |
| `run_args.txt` | 這次執行的完整設定 |

終端機的輸出另外存在 `logs/vo_flight_capture_fix3_<時間>.log`。

### 5.1 照片檔名

```
VO_<時間>_<真實緯度>_<真實經度>_<真實高度>_<VO緯度>_<VO經度>_<VO高度>_<真實與VO的差距m>_
   <北向位移>_<東向位移>_<高度變化>_<匹配數>_<inlier數>_<累積距離>.png
```

和 fix2 有兩個差別,欄位數不變,`parse_flight_results.py` 可以照常使用:

- `<時間>` 多了 3 位毫秒,格式為 `yyyymmddhhmmssmmm`(17 位數)。
- **兩個高度都改成相對於起飛點的高度(公尺)**,不再是海平面高度。海平面高度帶有 GPS 垂直誤差,同一地點不同趟飛行會差到十幾公尺。

| 欄位 | 內容 |
|---|---|
| 真實高度 | Pixhawk 回報的相對高度(`relative_alt`),和程式判斷起飛、降落用的是同一個數字 |
| VO 高度 | VO 追蹤的高度減去起飛點的地面海拔 |
| 高度變化 | 這一張和前一張之間相對高度的差值,向上為正 |

VO 的高度本來就跟著氣壓計走,所以「真實高度」和「VO 高度」通常是同一個數字。在地面附近這個數字可能是小的負值(例如 `-0.30`),屬於正常漂移。

海平面高度仍保留在 `vo_log.csv` 的 `real_alt_msl` 與 `vo_alt_msl` 欄位;相對高度是 `real_alt_agl` 欄位。

### 5.2 `vo_log.csv` 的 `status` 欄位

| 狀態 | 意義 | 位置是否更新 |
|---|---|---|
| `FIRST` | 第一張,沒有前一張可比對 | — |
| `OK` | 位移可信 | 是 |
| `LOW-CONF` | inlier 比例偏低,位移不太可信 | 是 |
| `REJECT` | fix3 新增。速度、地面比例或地面旋轉不合理,量到的數值仍記錄在該列 | **否** |
| `FAIL` | inlier 不足,或這張照片沒有對應的姿態資料 | 否 |

分析時建議以 `vo_log.csv` 的 `frame` 欄位排序,不要依檔案修改時間。

---

## 6. 常見訊息與處理方式

| 訊息 | 意義 | 處理 |
|---|---|---|
| `All files are synced to disk -- safe to power off.` | 所有檔案已寫入磁碟 | 可以斷電 |
| `Writing the remaining photos to disk -- DO NOT CUT THE POWER YET ...` | 正在寫入還沒存完的照片 | 等上一則訊息出現 |
| `WARNING: ATTITUDE arrives at only x Hz` | Pixhawk 沒有照要求的頻率送 ATTITUDE,傾斜補償會不準 | 檢查該遙測埠的 `SRx_EXTRA1` 參數,或 Pixhawk 是否支援 `SET_MESSAGE_INTERVAL` |
| `WARNING: processing this frame took ... longer than --interval` | 處理一張照片的時間超過拍照間隔,實際間隔被拉長 | 加 `--sift-features 2000`、`--burst 1`,或拉長 `--interval` |
| `WARNING: N photos are waiting to be written` | 磁碟寫入速度跟不上拍照 | 加 `--no-save-burst`,或拉長 `--interval` |
| `WARNING: no ATTITUDE sample covering this frame time` | 這張照片的時間點沒有姿態資料,該段會是 `FAIL` | 偶爾出現可忽略;持續出現請檢查遙測連線 |
| `WARNING: real GPS is stale` | 超過 2 秒沒有收到新的 GPS 位置 | 檢查遙測連線 |
| `WARNING: camera may be STUCK` | 真實 GPS 有移動,VO 累積距離卻幾乎沒變 | 相機可能卡在舊畫面,降落後檢查照片 |
| `Camera self-check FAILED` | 自我檢查時兩張畫面幾乎相同 | 確認有晃動相機;仍失敗則 `sudo systemctl restart nvargus-daemon` 後重試,再不行就重開機 |
| `Got a GPS position, but no ATTITUDE message within 10s` | 收不到 ATTITUDE,程式無法啟動 | 檢查遙測串流設定;Pixhawk 確實無法送 ATTITUDE 時只能改用 fix |

---

## 7. 上傳到 Jetson

在 PC 的 PowerShell 中,於程式資料夾執行:

```powershell
.\upload_to_jetson.ps1
```

腳本已加入 `vo_flight_capture_fix3.py`、`run_vo_flight_capture_fix3.sh` 與本說明檔。Jetson 上已存在的同名檔案會先備份成 `<檔名>.bak_<時間>`。

---

## 8. 飛行前檢查清單

1. 擦拭鏡頭,確認沒有霧氣、指紋或保護膜。
2. 第一次使用 fix3 時,先完成第 2 節的地面測試。
3. 確認沒有其他拍攝程式在執行:`./run_vo_flight_capture_fix3.sh status`。腳本啟動時也會自動檢查。
4. 啟動後依提示晃動相機,確認出現 `Camera self-check PASSED`。
5. 確認 `ATTITUDE rate` 接近 20 Hz,沒有警告。
6. 飛行中建議安排一段懸停、一段直線等速、一段轉彎,方便事後分別評估雜訊、距離比例與航向。
7. 降落後等 `safe to power off` 出現再斷電。
