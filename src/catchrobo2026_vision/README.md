# カメラ認識表示

Jetson上で `camera/image_raw → red_cylinder_detector → vision_http_bridge` を動かし、
同じLANのブラウザーへJPEG画像と認識座標を配信します。ロボット制御用topic・service・actionへの接続はありません。
親 `production.launch.py` は `camera_enabled:=true` の場合にこの独立launchを起動します。
既定は無効なので、カメラがなくてもUIを起動できます。

```bash
source /opt/ros/jazzy/setup.bash # Jetsonでは使用するROS環境に合わせる
source install/setup.bash
ros2 launch catchrobo2026_vision vision.launch.py
```

本番カメラは `camera_backend:=auto`、`camera_device:=/dev/video0` が既定です。
取り込んだUSB V4L2・Jetson Argus・IMX219 raw backendを維持しています。
rawのセンサー設定は取込元機体向けなので、実機との適合確認が必要です。
既定は640×480、20 FPS、JPEG品質75。調整はcameraのROSパラメーター、detectorの
`jpeg_quality`・bridgeの`stale_seconds` (既定1秒) を利用できます。

## カメラなしで再現する

```bash
ros2 run catchrobo2026_vision make_test_video.py /tmp/catchrobo-vision.avi
ros2 launch catchrobo2026_vision vision.launch.py \
  camera_backend:=video camera_video_path:=/tmp/catchrobo-vision.avi
```

動画は640×480/20 FPSの決定的なMJPEG AVIです。赤い候補が移動し、4–5秒に消失、5–6秒に緑色になります。
`camera_video_loop:=false` で最後に入力が止まり、HTTPのstale表示と再起動後の復帰を確認できます。
video backendは`camera_fps`で再生し、ファイル内のfpsに自動追従しません。
生成ファイルをGitへ追加する必要はありません。

## HTTP API

既定は `http://<Jetsonのホスト名またはIP>:8081`。UIポートとは別です。
Jetsonから端末へこのTCPポートを通し、同じ有線LANで使用してください。
HTTP配信なので、HTTPSのページから使う場合は同一HTTPSのreverse proxy等が必要です。

- `GET /stream.mjpg`: `multipart/x-mixed-replace; boundary=frame`。各partはJPEG、`Content-Length`、`X-Sequence`、`X-Stamp-Ns`付き。
- `GET /status.json`: CORS `*`、no-store。`available`、`stale`、`age_ms`（bridge公開後）、`source_age_ms`（ROS画像stampから）、`sequence`、`stamp: {sec,nanosec}`、`frame_id`、`width`、`height`、`detected`、`center_pixel: {u,v}`、`jpeg_bytes`。
- 未受信は `available=false, stale=true`。入力が1秒途絶、または元画像が1秒以上古い場合は `stale=true, detected=null, center_pixel=null`。
- 座標は画像のpixel単位（u右向き、v下向き）。3D・ロボット座標への変換、制御への使用は行いません。

検出器は既存の`detected`/`center_pixel`/`position`を保持し、追加の
`red_cylinder/result` (`std_msgs/String` JSON) にstamp付きの検出有無とpixel座標をまとめます。
bridgeはこれと`debug_image/compressed` (`sensor_msgs/CompressedImage`) の**同じstamp**だけを結合し、1つのsnapshotとして公開します。
JSON statusと別接続の画像は取得時刻が異なるため、厳密な比較にはsequence/stampを使ってください。
画像内の枠・座標注記は当該画像自身の検出結果です。
`pipeline`には受信画像/結果数、結合数、旧stamp棄却数、直近の検出全体/JPEG符号化時間とJPEG画像到着時の鮮度を診断用に返します。

camera→detectorのraw画像はreliable/depth 1。detectorでJPEG圧縮し、
bridgeには小さいJPEG画像と結果JSONをbest-effort/depth 1で送ります。
大きいraw debug画像をPythonへ配送・再圧縮する処理を避けます。未結合サンプルは各3件、JPEGは最新1枚だけ保持します。
各HTTP接続は送信完了後に最新を選び、遅れたフレームをアプリ内でキューへ積みません。
OSのTCP送信bufferは小さく制限し、書込みが0.8秒進まないクライアントを切断します。
最大同時接続は8です。ブラウザーの再接続で最新から復帰します。
入力停止中のMJPEGは新しいフレームを出さないため、UIはJSON statusで期限を判定してください。
bridge終了時は待機・HTTP接続を閉じてサーバースレッドを回収します。

## 検証

`colcon test --packages-select catchrobo2026_vision red_cylinder_detector` でHTTPと検出器の試験が動きます。
`test/probe_video_pipeline.py <install prefix> <output directory>` は動画を生成し、実際の
camera→detector→HTTP経路を起動してFPS、画像stampから受信までの遅延、JPEG転送量、
低速接続、切断後再接続、動画終端時のstale、入力再開をJSONへ記録します。
カメラ・Jetson性能・LANの実測値はローカル模擬試験と区別してください。

取込元: `../supplementary/gaku2026_ws` / `a011c9382d4953f76189bbd78fb5eef83343b3ac` の
`camera_publisher` と `red_cylinder_detector`。ソースは本リポジトリ内へコピー済みで、外部パスへの実行・ビルド依存はありません。
追加差分はvideo backend・depth 1・stamp付き結果・HTTP bridge・launch・模擬動画・試験です。
