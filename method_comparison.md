# VO 方法比較

三套跟「GPS-less 定位」有關的程式，任務本質不完全相同，比較前先釐清：

| 專案 | 程式 | 解決的問題 | 特徵方法 |
|---|---|---|---|
| `gpsless_VO_image`（本專案）| `visual_odometry.py` | **相對位移**：連續兩張照片比對，算飛了多遠、轉幾度 | SIFT（預設）或 ORB，可切換 |
| `VO_ChatGPT/vo_project` | `vo.py` | 同上，**相對位移** | 只有 ORB |

`visual_odometry.py` 跟 `vo.py` 是同一種任務的兩種實作，適合直接比較；

---

## 一、`visual_odometry.py` vs `vo.py`（VO_ChatGPT）

### 演算法層面

| 項目 | `visual_odometry.py`（本專案）| `vo.py`（ChatGPT 版）|
|---|---|---|
| 特徵偵測器 | SIFT（預設）或 ORB，`config.py` 可切換 | 只有 ORB，寫死 |
| 特徵比對 | 單向 `knnMatch` + Lowe's ratio test | **雙向**比對（forward + reverse 都要通過 ratio test 且互相一致才算數），比較嚴格 |
| 幾何模型 | `cv2.estimateAffinePartial2D`（相似變換：旋轉+縮放+平移）+ RANSAC | 同一個 OpenCV 函式，同樣的相似變換 + RANSAC |
| 信心把關 | 2 層：inlier 絕對數量（≥12）+ inlier 比例（≥0.30）| **4 層**：inlier 數量（≥20）+ inlier 比例（≥0.25）+ 特徵點分布覆蓋率（凸包面積 ≥ 畫面 1%）+ 仿射縮放合理範圍（0.70~1.40）+ 單幀旋轉上限（35°）|
| 魚眼畸變處理 | 中央裁切（避開 Jetson IMX219 鏡頭邊緣畸變）| 沒有處理 |
| 相機焦距假設 | 用假設的對角視角（84°）反推焦距 px（約 444px，for 640×480）| 預設 `focal = max(width, height)`（例如 640px），可手動輸入校正值覆蓋 |
| 位移換算 | 兩個純量 GSD（`FX_PIXELS`/`FY_PIXELS`）| 完整相機矩陣（fx, fy, cx, cy），並額外除以該幀的仿射縮放係數，修正高度變化造成的影響（本專案目前沒做這步，`altitude_change_m` 只回報不回饋）|

### 系統層面

| | `visual_odometry.py` | `vo.py`（ChatGPT 版）|
|---|---|---|
| 絕對定位錨點 | 需手動提供經緯度＋航向 | 直接讀照片 EXIF GPS，並能用「GPS 起訖方向自動對齊」自動算出正確航向 |
| 尺度校正 | `distance_scale` / `yaw_scale`（本專案後續新增，並用真實 GPS 做最小平方法校正）| 只有「已知實際總距離」線性等比例校正，沒有角度校正 |
| Pixhawk / MAVLink | 有完整整合（`gps_sim.py`、`dead_reckoning.py`、即時餵給飛控當 GPS2）| 沒有，純離線分析工具 |
| 即時拍照 | 有（`live_vo_gps.py`，接 Jetson CSI 相機）| 沒有，只讀現成資料夾 |
| 介面 | CLI script | Streamlit 網頁 GUI（地圖、圖表、逐張照片預覽、CSV 下載）|
| 檔名排序 | mtime 或 DJI 序號正則 | 通用自然排序（檔名數字部分比大小）|
| 混合解析度 | 沒特別處理 | 自動抓數量最多的解析度、排除其他並警告 |
| 打包 | 無 | PyInstaller 打包成 Windows `.exe` |

### 一個具體、可驗證的發現

拿真實 GPS 資料反推最佳焦距，算出來這台相機比較合理的焦距約 **524px**（對角視角約 75°）。本專案原本假設的 444px（84° 視角）比 ChatGPT 版預設的 640px（65° 視角）更接近這個真實反推值——但兩邊都只是假設值，不是真正校正過的鏡頭規格。

---

## 二、SIFT vs ORB 實測（用真實 GPS 驗證，本專案內部比較）

同一批 DJI 密集序列（199 張、2 秒一張），套用 `recompute_vo_trajectory.py --detector sift` 與 `--detector orb`，再用 `fit_correction_scales.py` 對真實 GPS 做最小平方法校正：

| | SIFT | ORB |
|---|---|---|
| 處理時間（198 對）| ~33.5s | ~22.9s（快約 1.5 倍）|
| 未校正距離偏差 | +16.8% | +17.2%（幾乎一樣）|
| 未校正最終位置誤差 | 61.80m | 61.71m（幾乎一樣）|
| 最佳化 `distance_scale` | 0.8483 | 0.8506（幾乎一樣）|
| 最佳化 `yaw_scale` | 0.9358 | 0.9360（幾乎一樣）|
| 最佳化後平均誤差 | 3.95m | **3.20m**（略勝）|
| 最佳化後最終誤差 | 12.14m | **10.82m**（略勝）|

**結論**：在「同一台相機拍的連續密集照片、重疊度高」這種本專案實際使用情境下，SIFT 理論上比較準的優勢沒有體現出來——兩者的系統性偏差幾乎一模一樣，ORB 甚至最終誤差略小，且快 1.5 倍。目前 `config.py` 維持 `DETECTOR = 'sift'` 為預設；若之後需要更高的拍照頻率（縮短 `--interval`），有實測數據支持切換到 ORB 是安全的選項。

---


