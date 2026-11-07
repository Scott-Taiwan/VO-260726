# VO-260726 — GPS-less Visual Odometry 專案摘要

無 GPS 環境下，用下視相機拍到的連續照片估算 F450／Jetson Orin Nano 無人機的位移（Visual Odometry），並將估算結果接進 Pixhawk 作為第二組 GPS 來源。跟另外兩個專案 `gpsless_mapping`／`gpsless_superpoint`（用衛星圖磚比對做「絕對定位」）是互補關係：這裡做的是兩次定位之間的「相對位移」估算。這個 repo 是獨立、可自行運作的版本（`gps_sim.py` 已一併複製過來，不依賴其他 repo）。

## 核心演算法 — `visual_odometry.py`

流程（詳見 [flow.md](flow.md) 的完整圖解）：

1. **`list_images()` / `parse_altitude_m()`** — 依拍攝時間排序照片，並從檔名解析每張的高度。
2. **`detect_and_match()`** — 先做中央裁切（濾掉魚眼鏡頭邊緣畸變），再用 SIFT／ORB 偵測特徵並比對。
3. **`estimate_similarity()`** — RANSAC 擬合「旋轉＋縮放＋平移」的相似變換。
4. **`displacement_from_transform()`** — 用高度＋相機視角換算像素位移為公尺。
5. **信心度把關** — inlier 數量與 inlier 比例都太低時，標記為 `LOW-CONFIDENCE`／`FAIL`，不會假裝有把握。

**數學已驗證正確**：用已知的合成位移（20m／-10m／15°）反推，誤差在 3 公分內。

## 新相機（77 度水平視角）專用飛行工具 — `vo_flight_capture.py` 系列

原本的 `visual_odometry.py`／`config.py` 是為舊的 ~166 度魚眼鏡頭設計的（需要中央裁切），後來換了一顆水平視角 77 度的新鏡頭，不再需要裁切，所以另外寫了一支**新的、獨立的飛行用程式**，而不是直接改舊程式（避免影響還在用舊鏡頭假設的 `process_dji_sequence.py`、`recompute_vo_trajectory.py`、`dead_reckoning.py` 自我測試等）：

- **`vo_flight_capture.py`** — 主程式。開機後讀一次 Pixhawk 的真實 GPS＋航向當作固定錨點（之後不再重讀），用同一個相機連線做**起飛前自我檢查**（拍兩張、提示操作者轉動相機，確認真的有抓到新畫面，不是卡住的舊畫面），通過後才等高度超過門檻（預設 20m AGL）開始拍照，每張照片檔名同時記錄「即時真實 GPS」跟「VO 推算位置」方便事後比對。純記錄用途，**不會把 VO 結果寫回 Pixhawk**。
- **`vo_flight_capture_always.py`** — 同上，但不等待起飛高度，GPS／相機自我檢查過了就立刻開始拍（適合手持地面測試），也沒有自動降落停止（只能 Ctrl+C），避免高度在門檻附近晃動時誤判成已降落。
- **`vo_flight_capture_1s.py`** — 同 `vo_flight_capture.py`，只是把預設拍照間隔從 2 秒改成 1 秒（直接重用 `vo_flight_capture.main()`，不是複製貼上）。
- **`run_vo_flight_capture*.sh`** — 對應的背景執行腳本，支援 `status`／`stop`。`stop` 送出 `SIGINT` 後最多等 10 秒確認真的結束，超時會自動升級成 `kill -9` 強制終止（相機底層卡死、對一般訊號沒反應時，單純送訊號是沒用的）。
- **`vo-flight-capture.service`** — 對應的 systemd 服務設定檔，可開機自動啟動（需要 `sudo cp` 到 `/etc/systemd/system/` 並 `daemon-reload` 才會生效）。
- **`preflight_camera_check.py`** — 單獨的起飛前相機健康檢查（拍兩張、請操作者搖晃相機、比對像素差異），用的是跟飛行時完全相同的相機開啟方式。
- **`camera_stability_test.py`** — 簡單的相機穩定度測試，固定間隔連續拍照存成編號檔名，給人事後肉眼比對用。
- **`parse_flight_results.py`** — 把 `photo_obtained/<run_id>/` 底下一批飛行照片的檔名解析成 CSV，並畫出「真實 GPS vs VO 估算」的飄移量／軌跡圖。

## 實測結果與已知限制

### 靜態測試（舊鏡頭時期）
用 `photo_obtained/` 裡 3 張真實照片測試（檔名內建的 GPS 座標僅作驗證用，從未餵進演算法）：

| 路段 | VO 估算 | GPS 真實距離 | 誤差 | 信心 |
|---|---|---|---|---|
| B→A | 7.07 m | 47.24 m | 85% | LOW-CONFIDENCE |
| A→C | 3.96 m | 9.48 m | 58% | LOW-CONFIDENCE |

**根本原因**：這 3 張照片間隔太遠（60m 高度、視野只有約 70×55m），兩張照片重疊區域太小，加上草地與跑道圈狀標記等重複性紋理，讓 RANSAC 鎖定一個自洽但錯誤的變換。**結論：拍照間隔必須夠密（重疊度夠高）VO 才準**，稀疏的「每個航點拍一張」用不了。

### 新鏡頭實際飛行測試
用 `vo_flight_capture.py` 實際飛了兩次（2026-11-01、2026-11-07），都遇到同一個問題：**相機 session 中途卡死，之後每一張拍到的都是同一張凍結畫面**（兩次飛行拍到的卡死畫面內容完全一樣，連畫面細節都對得上），VO 因此全程回報位移趨近於 0，跟真實 GPS 持續有位移的狀況明顯矛盾。另外也觀察到約 20~25% 的照片會變成 0 位元組的空檔案（存檔失敗）。詳見下方「相機穩定度問題」。

## Pixhawk 整合

沿用既有的 `gpsless_mapping/gps_sim.py` `GPS_INPUT`（MAVLink 232，GPS2，`GPS2_TYPE=14`）機制，沒有另外開新通道。**`vo_flight_capture.py` 系列不會呼叫這個機制**——純記錄用途，真實 GPS 持續負責飛行。

- **`dead_reckoning.py`** — `PositionTracker`：持有一個「錨點」（絕對定位：lat/lon/alt + 真實航向），用每一段 VO 結果把它往前推。信心不足或失敗的路段會讓 `hacc_m`（水平精度）成長更多，讓 `GPS_AUTO_SWITCH` 自動偏向較新的絕對定位。旋轉數學已用合成案例驗證（航向 0° 與 90° 兩種情況）。
- **`gps_sim.py`**（微調）— `send_gps_input()` 新增 `hacc` 參數（預設 0.5，向下相容），是信心標記真正接進去的地方。
- **`vo_gps_bridge.py`** — 批次版驅動程式：讀一個資料夾的照片，逐段跑 VO → dead-reckoning → 送 `GPS_INPUT`，含 `--dry-run`。

## 即時拍照迴圈（舊鏡頭版本）— `live_vo_gps.py`

開一次相機、依固定 `--interval` 秒數持續拍照，每張新照片跟前一張跑 VO，結果餵進 `PositionTracker`，兩次拍照之間持續以 `--gps-hz` 頻率重送目前位置給 Pixhawk（避免 GPS2 被當成逾時）。**這支程式原本是給舊鏡頭用的即時導航工具，會真的把 VO 結果送回 Pixhawk**；新鏡頭的飛行改用上面的 `vo_flight_capture.py`（純記錄，不送回），但 `vo_flight_capture.py` 仍直接重用這支程式裡的 `gstreamer_pipeline()`／`capture_gray()`／`connect_pixhawk()`／`read_global_position()`／`read_heading()` 這幾個函式，沒有重寫。

## 相機穩定度問題（2026-11 新發現，目前最大的未解項目）

飛行測試反覆遇到相機卡死，追查後發現**不是單一原因**，而是三個疊在一起的問題：

1. **CSI 排線／插槽接觸不良（硬體層）**：查開機時的 kernel log，發現 `imx219` 感測器的 I2C 探測（`error during i2c read probe (-121)`，即 `EREMOTEIO`）在好幾次開機裡時好時壞——同一個插槽，有時候探測成功、有時候完全失敗。換了另一個 CSI 插槽做對照實驗，初步看起來有改善，但樣本數還不夠多，需要持續觀察。
2. **CMA（相機專用保留記憶體）耗盡（系統資源層）**：同一次開機內，反覆開關相機很多次之後，會開始出現 `NvMapMemAllocInternalTagged ... error 12`（記憶體不足）、`SCF: Error InsufficientMemory` 這類錯誤，**只有整台重開機能清除**，單純重啟 `nvargus-daemon` 沒有用（這是核心層級資源，不受 daemon 重啟影響）。
3. **程式本身的資源洩漏（已修復）**：原本 `vo_flight_capture.py`／`vo_flight_capture_always.py` 的相機自我檢查（`self_check_camera()`）沒有包在涵蓋到結尾的 `try/finally` 裡，只要自我檢查失敗（今天幾乎每次相機異常都會在這裡失敗），`cap.release()` 就不會被呼叫，等於每次失敗都在洩漏相機資源，加速了問題 2 的發生。**已修正**：現在從開相機到程式結束全部包在同一個 `try/finally` 裡，不管是自我檢查失敗（`SystemExit`）還是 `Ctrl+C` 中斷，都保證會呼叫 `cap.release()`。

**因應這些問題新增的防護機制**：
- 起飛前自我檢查（`self_check_camera()`，整合進 `vo_flight_capture.py` 本身，而不是另外開一個 session 測試——因為實測發現「另外測一個 session」通過，不代表接下來真正飛行開的那個 session 也會正常）。
- 飛行中即時偵測：每隔固定張數比較「真實 GPS 位移量」跟「VO 推算位移量」，兩者明顯對不上（真實有在動、VO 卻幾乎不動）就印出警告，不用等降落後才發現。
- `cv2.imwrite()` 存檔失敗現在會被偵測並印出警告（含當下剩餘硬碟空間），而不是靜默失敗。
- `stop` 腳本的訊號處理：`SIGINT` 等 10 秒沒生效就自動升級成 `kill -9`，避免「送了訊號但其實沒停」的情況。

### 意外發現的環境問題（已修復）

`~/.local/lib/python3.10/site-packages` 裡有一個 pip 裝的 `opencv-python`（4.13.0，無 GStreamer），蓋掉了 JetPack 系統內建、有 GStreamer 支援的 OpenCV（`/usr/lib/python3/dist-packages`，4.5.4），導致 `gpsless_mapping/capture_sample.py` 一度無法開啟相機（`ERROR: Could not open CSI camera`）。**沒有整個移除** `~/.local` 的 opencv-python——它是 `ultralytics`（YOLO 偵測腳本）的相依套件，整個解除安裝可能連帶弄壞那些腳本。改成跟 `live_vo_gps.py` 一樣的做法：在 `import cv2` 之前插入 `sys.path.insert(0, '/usr/lib/python3/dist-packages')`，讓匯入時優先吃到系統那份有 GStreamer 的 OpenCV。已用真實相機驗證修復有效。

## 檔案清單

| 檔案 | 說明 |
|---|---|
| `config.py` | 相機／演算法／dead-reckoning 全部可調參數（舊鏡頭） |
| `visual_odometry.py` | 核心 VO 演算法 + CLI（舊鏡頭，含中央裁切） |
| `dead_reckoning.py` | 錨點＋dead-reckoning 的 `PositionTracker` |
| `vo_gps_bridge.py` | 批次版：資料夾照片 → VO → GPS_INPUT（舊鏡頭） |
| `live_vo_gps.py` | 即時版：邊拍邊估算邊送 GPS_INPUT（舊鏡頭，會寫回 Pixhawk） |
| `vo_flight_capture.py` | **新鏡頭（77°）專用，純記錄，不寫回 Pixhawk，起飛前自我檢查＋卡住偵測** |
| `vo_flight_capture_always.py` | 同上，不等起飛高度，適合地面手持測試 |
| `vo_flight_capture_1s.py` | 同 `vo_flight_capture.py`，預設拍照間隔改成 1 秒 |
| `preflight_camera_check.py` | 獨立的起飛前相機健康檢查 |
| `camera_stability_test.py` | 固定間隔連續拍照，供肉眼比對相機穩定度 |
| `parse_flight_results.py` | 把飛行照片檔名解析成 CSV＋飄移／軌跡圖 |
| `run_vo_flight_capture*.sh` | 對應的背景執行腳本（status／stop，含逾時強制終止） |
| `vo-flight-capture.service` | 開機自動啟動用的 systemd 服務設定 |
| `flow.md` | `visual_odometry.py` 流程圖（中文） |
| `method_comparison.md` | SIFT vs ORB、本專案 vs ChatGPT 版 VO 的比較 |
| `vo_flight_capture_walkthrough.html` / `code_walkthrough.html` | 逐函式學習頁面 |
| `photo_obtained/` | 測試照片（含真實 GPS 檔名，僅供驗證） |
| `gps_sim.py` | 複製自 `gpsless_mapping` 專案，並加上 `hacc` 參數供本專案接入信心標記 |

## 下一步待辦

1. **持續排查相機硬體穩定度**——CSI 排線／插槽的對照實驗還需要更多次開機資料才能下定論；如果問題持續，考慮換一條新排線。
2. 真正飛到有實際位移的一趟完整飛行，確認相機在整趟飛行中都穩定，拿到一組乾淨、可信的 VO 準確度數據（目前兩次實飛都卡在相機問題，還沒拿到乾淨的位移比對結果）。
3. 用真實飛行 log 校準 `config.py` 裡的 `HACC_GROWTH_FRACTION_*` / `HACC_PENALTY_*`（目前是合理猜測，非實測校準值）。
4. 確認相機安裝角度，校正 `CAMERA_MOUNT_YAW_OFFSET_DEG`（目前預設 0，未針對實機驗證）。
5. 新鏡頭的 `FX_PIXELS`／`FY_PIXELS` 目前是用水平視角 77° 搭配「正方形像素」假設反推出來的，還沒有真正做過棋盤格校正，準確度有限。
6. 串接絕對定位（SIFT／SuperPoint 圖磚比對）成功時自動呼叫 `tracker.reset_anchor()`，取代目前的手動錨點。
