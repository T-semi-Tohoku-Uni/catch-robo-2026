# red_cylinder_detector

ROS 2 Humble / C++ / OpenCVで、赤い円柱の候補を検出して座標を配信します。
既存の `camera_publisher` の画像を購読するので、カメラを二重に開きません。

HSVの赤色範囲（色相の両端）を抽出し、モルフォロジー処理でノイズを除去します。
輪郭面積、縦横比（高さ/幅）、矩形内の充填率を満たす候補のうち、最大面積の1個を選びます。
**既定では、画像上で縦向きに写る円柱を想定します。** 赤い板や箱なども検出し得るため、
円柱形状そのものを保証する分類器ではありません。照明、背景、傾きに応じて調整してください。

対象の実寸は **直径66 mm・長さ170 mm** を既定値にしています。
長さ/直径は約2.58です。既定の縦横比範囲はこの比率の0.65〜1.35倍（約1.67〜3.48）です。
この幅は見え方や色抽出のばらつきを許容する目安であり、実画像で調整してください。
長さは形状候補の絞り込みに、直径は3次元推定に使用します。

## ビルド・起動

zshの例です。bashでは `setup.bash` に置き換えます。

```zsh
# catch-robo-2026 のルートで実行
source /opt/ros/humble/setup.zsh
colcon build --symlink-install --packages-select camera_publisher red_cylinder_detector
source install/setup.zsh
ros2 run camera_publisher camera_publisher_node
```

別ターミナル:

```zsh
# catch-robo-2026 のルートで実行
source install/setup.zsh
ros2 run red_cylinder_detector red_cylinder_detector_node
```

座標・検出画像の確認（別ターミナル）:

```zsh
source install/setup.zsh
ros2 topic echo /red_cylinder/center_pixel
ros2 topic echo /red_cylinder/detected
# 画像ビューアーがインストール済みの場合:
ros2 run rqt_image_view rqt_image_view /red_cylinder/debug_image
```

## トピック

| トピック | 型 | 内容 |
| --- | --- | --- |
| `camera/image_raw` | `sensor_msgs/msg/Image` | 入力。`bgr8` / `rgb8`。行パディングにも対応。Reliable / Volatile、depth 1 |
| `red_cylinder/center_pixel` | `geometry_msgs/msg/PointStamped` | 検出輪郭の重心。x=u（右向き）、y=v（下向き）、z=0。単位はピクセル、原点は左上 |
| `red_cylinder/detected` | `std_msgs/msg/Bool` | 有効な画像ごとに検出の有無を配信 |
| `red_cylinder/debug_image` | `sensor_msgs/msg/Image` | 検出候補に緑のバウンディングボックス、中心・座標を重ねたBGR画像。有効な入力画像ごとに配信。Reliable / Volatile、depth 1 |
| `red_cylinder/debug_image/compressed` | `sensor_msgs/msg/CompressedImage` | 同じ注釈画像のJPEG。best-effort / depth 1。購読者がいる時に生成 |
| `red_cylinder/result` | `std_msgs/msg/String` | 同じstampの検出有無、座標、画像寸法、処理時間のJSON。best-effort / depth 1 |
| `red_cylinder/position` | `geometry_msgs/msg/PointStamped` | オプション。カメラ光学座標系での推定位置（m） |

座標と画像は元画像のタイムスタンプと `frame_id` を引き継ぎます。
`center_pixel` は **3次元位置ではありません**。TF変換には使用しないでください。
未検出フレームでは座標を配信せず `detected=false` を配信します。
不正な入力は警告して無視します。入力が途切れた場合は何も配信しないため、
制御などで利用する側は `header.stamp` による鮮度確認も行ってください。
座標と検出状態のQoSはReliable / Volatile（depth=10）です。

入力トピックを変更する場合:

```zsh
ros2 run red_cylinder_detector red_cylinder_detector_node --ros-args \
  -r camera/image_raw:=/my_camera/image_raw
```

## パラメータ

すべて起動時のみ設定できます。

| 名前 | 既定値 | 内容 |
| --- | --- | --- |
| `hue_width` | `10` | OpenCV HSV色相0〜179の赤の幅。0〜hue_widthおよび179−hue_width〜179を抽出。0〜89 |
| `saturation_min` | `100` | 彩度の下限（0〜255） |
| `value_min` | `60` | 明度の下限（0〜255） |
| `min_area` | `300.0` | 最小輪郭面積（px²） |
| `min_aspect_ratio` | 約`1.67` | 高さ/幅の下限。未指定時は長さ/直径×0.65 |
| `max_aspect_ratio` | 約`3.48` | 高さ/幅の上限。未指定時は長さ/直径×1.35 |
| `min_fill_ratio` | `0.55` | 輪郭面積/外接矩形面積の下限（0より大きく1以下） |
| `morphology_kernel` | `5` | ノイズ除去のカーネル幅。1〜31の奇数。1なら実質無効 |
| `jpeg_quality` | `75` | JPEG符号化品質（1〜100）。raw debug画像には影響なし |
| `publish_debug_image` | `true` | 検出結果画像の出力 |
| `cylinder_diameter_m` | `0.066` | 円柱の実直径（m）。0で3次元推定を無効化し、既定の縦横比範囲は1.2〜8.0に戻る |
| `cylinder_length_m` | `0.170` | 円柱の実長（m）。正の値。既定の縦横比範囲の算出に使用 |
| `fx`, `fy` | `0.0` | 入力画像に対応する焦点距離（px）。3次元推定時は正の値が必要 |
| `cx`, `cy` | `0.0` | 入力画像に対応する主点（px）。3次元推定時は校正値を設定 |

例:

```zsh
ros2 run red_cylinder_detector red_cylinder_detector_node --ros-args \
  -p min_area:=500.0 -p saturation_min:=80 -p min_aspect_ratio:=1.0
```

## カメラ基準の3次元座標（任意）

RGB画像だけから絶対距離は確定できません。実際の円柱直径と校正済みの内部パラメータを
与えると、見かけの幅からピンホールモデルで近似します。
既定の直径は0.066 mです。`fx=fy=0` のままなら、画像とピクセル座標のみ出力します。
3次元推定を有効にするには `fx` と `fy` の両方を正の校正値に設定してください。
`Z=fx*直径/検出幅`, `X=(u-cx)*Z/fx`, `Y=(v-cy)*Z/fy`。
Xは右、Yは下、Zは前で、入力の `frame_id` はカメラ光学座標系である必要があります。
空の `frame_id` や画面端に接する候補に対して3次元座標は出力しません。

**以下のカメラ内部パラメータは説明用です。実際の校正値に置き換えてください。**

```zsh
ros2 run red_cylinder_detector red_cylinder_detector_node --ros-args \
  -p cylinder_diameter_m:=0.066 -p cylinder_length_m:=0.170 \
  -p fx:=600.0 -p fy:=600.0 -p cx:=320.0 -p cy:=240.0
ros2 topic echo /red_cylinder/position
```

円柱の軸が画像上で縦向き、全体が見えていることを想定した近似です。
傾き、遮蔽、レンズ歪み、誤検出で距離がずれます。内部パラメータは、リサイズや
歪み補正を含む**入力画像の状態と解像度**に合わせてください。
歪み補正やCameraInfoの購読、TF変換は本ノードでは行いません。

## テスト

```zsh
colcon test --packages-select red_cylinder_detector --event-handlers console_direct+
colcon test-result --verbose
```

実行ノードに合成画像を送信し、座標、3次元近似、RGB/BGR、行パディング、
色相の両端、最大候補選択、未検出、ノイズ、画面端、不正入力からの復帰を検証します。
実物を撮影した環境での検出精度は別途確認してください。

## このワークスペースへの取込み

取込元: `gaku2026_ws` / `a011c9382d4953f76189bbd78fb5eef83343b3ac`。
外部パスへの依存はありません。画像通信をreliable/depth 1へ変更し、表示用に
`red_cylinder/result` (`std_msgs/String` JSON) を追加しました。
JSONの通信はbest-effort / depth 1です。小さいJSONは大きい画像と独立に最新を送ります。
JSONは元画像の `sec`・`nanosec`、`detected`、`center_pixel: {u,v}`（未検出はnull）を含みます。
既存のBool・PointStamped出力も維持しています。
HTTP bridgeはJPEG画像と同じstampのJSON結果だけを公開し、画像と異なる時点の座標を混ぜません。
これは表示専用で、自動運転へ接続していません。
