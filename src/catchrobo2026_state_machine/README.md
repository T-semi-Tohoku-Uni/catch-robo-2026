# アーム準備照会と位置目標RPC

`state_machine_node`は`request_initialization`を提供します。MCUを先に投入して通信READY/アプリWAITINGへ置き、駆動電源投入後の明示INITIALIZEでモーター設定を開始します。ROS起動/BOOTや通常の準備照会だけでは開始しません。

PAUSEして既存target RPCを終端した後、空要求の`arm_initialize`を1回送り、返されたBoot IDを固定して`arm_ready`を有限pollします。診断は64byte/version2（従来の全3軸診断48byte＋own Boot ID16byte）です。有効な同一Boot ID、outcome OK、業務READY・必須flags31を確認した場合だけ同じpause tokenでRESUMEします。旧48byte、version不正、zero Boot ID、途中のBoot ID変更では成功しません。

開始要求は1boot1attemptです。MCUが既にRUNNING/READY/FAULTなら再始動せず診断を返します。開始応答のlost/timeout/nonOKを受けて自動再開始しません。残り期限があれば読取専用`arm_ready`を1回だけ取得して全軸診断を追加し、その診断がREADYでもINITは失敗で終えます。取得できなければ元の失敗理由を保持します。FAULTの解除は操作者によるMCU resetとbridgeの明示再activationが必要で、ROS再照会だけでは復旧しません。

開始指示前のSTOP送信は行わず、指示後の原点復帰・モード切替・故障時STOPと既存SYSTEM_RESET/q0 STOPの規約は維持します。UI/ROSのRESETへ自動rehomeは追加しません。

`arm_target_client`は起動時PAUSEDです。明示的なINITIALIZE成功までは位置目標を送信しません。既定の初期化前手順は空です。初期化前にMOVEを追加してもtarget gateが閉じているため動作しません。

ROSの`target_joint_angles` 3要素をlatest1件に集約し、1inflight・設定周期以上の間隔で`arm_target` RPCへ送ります。既定`application.arm_control.target_period_ms: 10`で上限100Hz、入力500Hzの全sampleを送信する意味ではありません。RTTやTag guardにより実効周期は遅くなり得ます。設定は10ms以上で、native制御周期10msは変更しません。

応答result0・float32 echo一致・confirmed-axis mask7で各軸native TXと新statusを確認します。位置到達の保証ではなく、到達はFollowRouteの3軸実測判定で確認します。古いtarget PUBLISHは使用しません。

unknown/reject/timeout/peer変更/不正echo・maskではfaultを保持し、新規目標の送信を止めてinflightとlatestを破棄します。MCUが最後の位置を保持し得るため、これは物理停止の保証ではありません。同じ500Hz入力が続いても再開しません。明示INITIALIZEが成功してgateを解除した後の新着入力だけを採用します。解除時に購読を作り直し、解除前のDDS待ちsampleも再送しません。

`arm_target/gate`はROS-localの`ArmTargetGate`サービスです。PAUSEが返すboot-random由来tokenと、同一ホストの単調時計に基づくabsolute deadlineをRESUMEで照合します。古いtoken、期限切れ、inflight残存時のRESUMEは拒否します。gateのPAUSE成功はRPC送信枠が空になった意味で、機械停止を証明しません。

automatic/main/handoperated launchはprofileの実効peer RPC期限を読みます。targetclientはarm response+50ms、初期化はprofileのinitialization_timeout_ms（既定20秒）+target待ち+開始応答+ready照会+余裕を覆う全phase予算（既定21.3秒）、UI/Joy/sequence/HTTP/browserはさらに外側の期限を用います。pumpもpump_set宛peerの実効RPC期限を用い、overrideがなければglobal期限を維持します。nodeを直起動する場合も同じservice名・期限を指定してください。

- targetclient: `canrpc_service`, `arm_target_endpoint`, `rpc_timeout_sec`, `target_period_ms`, `target_gate_service`
- initialization: `canrpc_service`, `arm_initialize_endpoint`, `arm_ready_endpoint`, `rpc_timeout_sec`, `readiness_poll_period_sec`（既定0.1秒）, `target_gate_service`

wireの正本は[CAN-RPC protocol](../canrpc_bridge/docs/protocol.md)と[configuration](../canrpc_bridge/docs/configuration.md)です。native statusにTag/parameter echoがない部分の因果相関、実CANの遅延、物理到達/停止/原点精度は模擬試験の保証範囲に含めません。

### 時間付き3軸軌道

`generate_trajectory`（`catchrobo2026_msgs/srv/GenerateTrajectory`）で生成した3軸軌道を、`follow_trajectory` Actionへ明示的に渡します。関節名は `theta1, theta2, theta3`、位置rad・時間秒です。公開APIは3値、計算ライブラリへの境界でのみ第4成分を0に補完します。通常のFollowRoute→Joy経路は維持します。Action内の `plan_from_start_goal`、保存済み軌道の暗黙再利用、payload変更はhardwareでは受け付けません。

`trajectory_dynamics_config` / `trajectory_pipeline_config` のYAMLを実際に読み込みます。hardware出力を明示無効にした設定、未計測dynamics、未設定速度・加速度・jerk制限では時間軌道のみ無効になり、通常の目標RPCは起動を維持します。同梱 `gakusai_trajectory.yaml` は制限値が未設定のため実行できません。仮のpreview値で解除しません。FF+PD/torque送信やhardware内部plantはありません。

時間軌道中はJoyからの位置目標を採用せず、既存位置RPCだけで送信します。通常FollowRouteは `arm_target/route_lease` の単一所有権を取得し、100msごとに更新します（lease500ms、取得/更新応答期限200ms、同一ホストmonotonic clock）。失効・古いtoken・RPC結果不明で新規目標送信を停止し、明示INITIALIZEまで保持します。これは機械の物理停止を保証せず、MCUは最後の位置を保持し得ます。

正常完了・取消は送信中RPCの終端と `joy/rebase` の成功を待ちます。Joyは500ms以内の3軸実測からFKし、速度0・新しいneutral入力待ちへ戻し、古いtarget_poseのDDS受信queueを破棄します。PoseStampedはrebase後のstampかつ500ms以内を要求し、zero/未来/不正stampを拒否します。ROS clock逆行や大きなjump後は再rebaseまで送信を止めます。取消時も新しい保持位置RPCは追加しません。時間軌道の成功は最終3位置payloadのAPPLIED/echo/mask7と、そのACK後に受信したfresh実測で3軸許容・XYZ30mm内を確認した場合に限ります。結果のyaw誤差は対象外のNaN、実測不明の位置/追従誤差もNaNとします。

`arm_target/cancel_trajectory`（Trigger）は外部Actionを含む取消、RPC drain、Joy再基準化、Action終端まで有界に待ちます。idleは成功、fault/失効/再基準化未確認はfalseです。UI STOP/RESETもこの確認を待ちます。明示INITIALIZEのGate PAUSEは全所有権を取り消し、worker/RPC drain後にREADY照会とJoy再基準化を経て解除します。古い目標・古い計画結果は再演しません。

生成/実行前validationはworkerで行いRPC/Gate/STOPのtimerを塞ぎません。生成は5秒で応答期限切れ・取消・結果破棄、Actionは `trajectory_timeout_sec`（既定30秒）でvalidation中も失敗します。worker終端まで新規所有権を認めず、世代の違うworkerが遅れてfollowerを開始しても結果回収時に取消します。停止待ちはRPC期限+1秒、外側UI/navはprofileの実効arm期限+1.5秒を使います。


### CAN起動失敗時の診断

bridgeが起動初期化を完了していない場合、診断用 `arm_ready` が64B/version2の実応答を返してもCall outcomeはUNAVAILABLE等の失敗です。stateはpayload長・version・Boot IDを操作成功とは別に検証し、有効な全3軸の状態・最後のcommand・応答状態とbridge側の理由を返します。操作成功はoutcome OKかつ業務READY・必要flagsがそろう場合に限り、非OKではtarget gateを再開しません。

UI/sequenceへは既存512B枠内の短い理由とdecoded診断を返し、raw全体・省略前のbridge理由はROSログへ残します。UIの起動継続や診断取得成功を、駆動電源正常・初期化成功・運動許可と同一視しません。


target adapterは `canrpc_status_topic`（既定 `canrpc/status`）の型付きBridgeStatusを監視します。READY・commands_enabled・socket_open・有効なactivation sessionがそろい、heartbeatが `bridge_status_timeout_sec`（既定2.5秒、1秒より大きい値）以内であることをRESUMEと指令送信の条件とします。初回READYだけでは初期PAUSEDを解除しません。

非READY、未受信/期限切れ、READYのままのsession変更でも、送信中でない場合を含めgateをPAUSED/faultへ閉じ、latest目標・route所有権・trajectoryを取り消し、旧resume tokenと未応答rebaseを無効にします。通信がREADYへ戻っても自動再開せず、明示INITIALIZEのpause→開始指示→同Boot READY照会→Joy再基準化→resume後に受信した新入力だけを使います。過去の目標・遅延rebase応答で復活しません。
