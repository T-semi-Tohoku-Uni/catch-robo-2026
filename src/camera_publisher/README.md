# camera_publisher

赤い円柱候補の検出と座標出力には、[red_cylinder_detector](../red_cylinder_detector/README.md)を使用できます。

ROS 2 Humble向けのC++カメラノードです。
`sensor_msgs/msg/Image`（`bgr8`）を配信します。
`backend:=auto`ではsysfs名から次の取得方法を選択します。

- Jetson IMX219（`vi-output`と`imx219`を含む）: V4L2でRG10を直接取得する`raw`
- その他のJetson CSI（`vi-output`を含む）: `argus`
- USBカメラなど: OpenCVの`v4l2`

取込元環境ではArgusのフレーム取得がタイムアウトしますが、RG10の直接取得は動作します。
`raw`ではJetsonの上位10ビットに配置されたRG10を読み取り、黒レベル補正、
RGGBのデモザイク、簡易ホワイトバランス、ガンマ補正を行います。
露出とセンサーゲインは固定値です。ArgusのISPと同等の画質補正ではありません。
`raw`の利用にはDISPLAYの変更やカメラサービスの再起動は不要です。

## ビルド・起動

```bash
# catch-robo-2026 のルートで実行
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select camera_publisher
source install/setup.bash
ros2 run camera_publisher camera_publisher_node
```

zshでは`setup.bash`の代わりに`setup.zsh`を使用してください。

取込元のJetsonではOpenCVがCUDA 12.2を要求したため、CUDAのバージョン不一致で
ビルドに失敗する場合は、以下のようにインストール済みの12.2を指定します。

```bash
colcon build --symlink-install --packages-select camera_publisher \
  --cmake-args -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-12.2
```

## パラメータ

すべて起動時に指定します。起動後の変更は受け付けません。

| 名前 | 既定値 | 内容 |
| --- | --- | --- |
| `device` | `/dev/video0` | カメラのデバイスパス |
| `backend` | `auto` | `auto` / `raw`（Jetson IMX219）/ `argus` / `v4l2`（USB）/ `video`（試験用） |
| `video_path` | 空文字列 | video backendの入力ファイル |
| `video_loop` | `true` | 動画終端で先頭に戻る |
| `sensor_id` | `0` | Argusのセンサー番号。`device`とは独立して指定 |
| `raw_sensor_mode` | `4` | rawのセンサーモード。下表参照 |
| `exposure_us` | `20000` | rawの露出時間（マイクロ秒） |
| `gain` | `64` | rawのセンサーゲイン。16が1倍、64が4倍 |
| `width` | `640` | 要求する画像幅（ピクセル） |
| `height` | `480` | 要求する画像高さ（ピクセル） |
| `fps` | `30.0` | 要求するカメラFPSと配信周期（0より大きく1000以下） |
| `frame_id` | `camera_optical_frame` | 画像ヘッダーのフレーム名 |

```bash
ros2 run camera_publisher camera_publisher_node --ros-args \
  -p device:=/dev/video0 -p width:=1280 -p height:=720 -p fps:=15.0 \
  -r camera/image_raw:=/my_camera/image_raw
```

解像度やFPSの実際の値はカメラの対応モードに依存し、起動ログに表示されます。
rawとArgusではセンサー画像を指定した幅・高さにリサイズします。
rawはこの環境のJetson IMX219専用です。USBのRG10カメラなどには使用できません。

| `raw_sensor_mode` | センサー解像度 | 最大FPS |
| --- | --- | --- |
| 0 | 3280×2464 | 21 |
| 1 | 3280×1848 | 28 |
| 2 | 1920×1080 | 30 |
| 3 | 1640×1232 | 30 |
| 4 | 1280×720 | 60 |

例えば露出とゲインを指定する場合:

```bash
ros2 run camera_publisher camera_publisher_node --ros-args \
  -p backend:=raw -p exposure_us:=20000 -p gain:=64
```

露出はフレーム周期より短くしてください（30 FPSなら約33333 µs未満）。
rawではセンサーをモードの最大FPS（モード4は配信30 FPS以下なら30 FPS）で
動作させ、配信時に最新のフレームを選びます。例えば15 FPS配信でもセンサーは30 FPSです。
露出時間はこのセンサー側の周期に収める必要があります。
モード4で60 FPSを指定する場合は`-p exposure_us:=10000`などを併せて指定してください。
同じカメラを使う別のノードやGStreamerは停止してから起動してください。

## Argusを明示的に使用する場合

Argusの利用にはGStreamer対応のOpenCV、`nvarguscamerasrc`、`nvvidconv`と
動作中の`nvargus-daemon`が必要です。

SSHのX11転送（`DISPLAY=localhost:...`）を使用している場合は、Argusが
転送先の画面に接続してEGLエラーになるため、ノードだけ`DISPLAY`を外して起動します。
画像ビューアーは別のターミナルで通常どおり起動できます。

```bash
env -u DISPLAY ros2 run camera_publisher camera_publisher_node --ros-args -p backend:=argus
```

`NvBufSurfaceFromFd Failed`やセンサーのタイムアウトが出た場合は、
カメラを使用するプロセスを停止してからサービスを再起動し、再度起動してください。
sudoの認証が必要です。

```bash
sudo systemctl restart nvargus-daemon
env -u DISPLAY ros2 run camera_publisher camera_publisher_node --ros-args -p backend:=argus
```
画像のタイムスタンプはフレーム取得直後のROS時刻です。
カメラのキャリブレーション情報（`camera_info`）やTFは配信しません。

## 配信の確認

既定のトピックは`/camera/image_raw`です。QoSはReliable / Volatile、depth 1です。

```bash
ros2 topic echo /camera/image_raw --qos-reliability reliable --once --field header
ros2 run rqt_image_view rqt_image_view
```

`rqt_image_view`は別途インストールが必要な場合があります。
カメラが開けない場合は、デバイスパス、アクセス権限、他のアプリによる使用を確認してください。

## このワークスペースへの取込み

取込元: `gaku2026_ws` / `a011c9382d4953f76189bbd78fb5eef83343b3ac`。
外部ワークスペースへ依存せず、ソースをコピーして使用します。
追加の `backend:=video` と `video_path:=/path/to/video.avi` はカメラなしの試験用です。
`video_loop` の既定はtrue、`fps` の周期でフレームを配信します。
画像のROS通信はreliable/depth 1で、大きいフレームの欠落と古いフレームの蓄積を抑えます。
動画の生成・表示・試験は `catchrobo2026_vision/README.md` を参照してください。
Jetson実機でのカメラ動作はこの取込み時点では未検証です。
