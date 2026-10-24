# CAN-RPC実装仕様

この文書は同梱 `core/include/canrpc_core.h` とROS／STM32アダプターが実装する通信仕様です。外部ファイルを参照せず利用できます。プロジェクトのNode ID・Subject・型・時間は [設定仕様](configuration.md) で定義します。

## フレームとID

CAN FDの29bit Extended ID・データフレームを使用します。標準ID、Classic CAN、RTR、error frameはROS受信対象外です。送信BRSはprofileの `bus.brs` に従います。Node IDは1〜126、0は予約、127はbroadcast宛先です。

| ID bit | CONTROL／REQUEST／RESPONSE | PUBLISH |
| --- | --- | --- |
| 28–26 | Priority 0〜7（0が最優先） | 同左 |
| 25–24 | Class: CONTROL=0、PUBLISH=1、REQUEST=2、RESPONSE=3 | 同左 |
| 23–17 | Destination | Subject上位7bit |
| 16–12 | Opcode／Service 0〜31 | Subject下位5bit |
| 11–5 | Source 1〜126 | 同左 |
| 4–0 | Tag／管理通知Seq | SubjectごとのSeq |

PUBLISHのSubjectは0〜4095。通常Dataには追加ヘッダを置きません。ROSはSubjectとSourceの設定一致で受信routeを選びます。送信Seqはrouteごとにmod32で進め、個別ACKはありません。Seqは計測の鮮度を保証しません。

対応Data型はIEEE754 float32、int32、uint8です。32bit値はlittle-endian、単位と並びはprofileとアプリケーションが定義します。固定要素数から論理長を決め、CAN FDの線上長 `0〜8,12,16,20,24,32,48,64` へ切り上げて末尾を0埋めします。受信は一致する線上長と余りの0を検証します。分割転送、可変長型、64byteを超えるpayloadは実装しません。

## 起動世代通知

CONTROL Opcode `0x10` BOOT_ANNOUNCEをbroadcastへ、通信開始直後・状態変更時・設定周期で送ります。Low5は通知Seqです。Boot IDは通信初期化時にLinux getrandom／MCU hardware RNGから生成する非ゼロ128bit値で、その通信世代内で維持します。

| offset | 長さ | 内容 |
| --- | --- | --- |
| 0 | 16 | Boot ID（不透明なbytes） |
| 16 | 4 | uptime_ms、u32 LE |
| 20 | 1 | Major=1 |
| 21 | 1 | Minor=0 |
| 22 | 1 | STARTING=0、READY=1、DEGRADED=2、RESET_PENDING=3、FAULT=4 |
| 23 | 1 | Capabilities bit0=SYSTEM_RESET、他bit=0 |

CONTROL `0x11` ANNOUNCE_QUERYはunicast、Low5=0、Data長0で再通知を依頼します。ROS bridge自体はCAN上のアプリケーションserviceを提供せず、宛先がROSのREQUESTにはCONTROL `0x02` REJECT_SERVICEを返します。REJECT_SERVICEはData長0、元REQUESTのTagを保持し、要求元Nodeを宛先にします。MCUの宣言済みserviceには次節の普通RPCでアクセスします。長時間Action・任意serviceの自動実行は対象外です。

同世代はuptimeが符号付き32bit差で正に進んだ通知だけで生存時刻・状態を更新します。未知／非READY／設定期限以上の通知途絶はpeer gateを閉じます。新Boot IDでは通知基準を更新し、直近8世代への後退を無視します。BOOT自体でセンサ値の受信時刻を更新しません。

## 普通RPC（REQUEST／RESPONSE）

宣言済みendpointはunicast REQUESTで呼び出し、RESPONSEで最終結果を返します。RESPONSEは要求のSource／Destinationを反転し、ServiceとTagを保持します。payloadの型と論理長はprofileのendpointとアプリケーション契約で定めます。REQUEST／RESPONSEにも同じCAN FD長への0埋め規則を適用します。

Tagは向き付きclient→serverペアで全serviceに共通です。clientはpeerごとに1件だけinflightを許し、guard中のTagを再利用しません。serverはsource＋Tagを全serviceで照合し、同じBoot世代・Service・payloadの重複は保存結果またはRUNNINGを返して再実行しません。相違はCONFLICT。cacheは設定容量（最大8）の固定枠で、受付時からTag guard終了まで枠を再使用しません。容量不足はREJECT_BUSYです。

| CONTROL Opcode | 意味 |
| --- | --- |
| 0x00 | ACCEPTED、受付済み |
| 0x01／0x02／0x03 | REJECT_BUSY／REJECT_SERVICE／REJECT_REQUEST |
| 0x04 | QUERY_STATE、状態／保存結果照会 |
| 0x05 | RUNNING |
| 0x0b | UNKNOWN_OUTCOME |
| 0x0d／0x0e | REJECT_NOT_READY／CONFLICT |

上記CONTROLのData長は0、Low5は元要求Tag、宛先は相手Nodeです。QUERY_STATEに対し完了済みで結果保持期間内なら元Service／TagのRESPONSEを返し、実行中ならRUNNING、履歴失効ならUNKNOWN_OUTCOMEを返します。

clientは受付前だけ、同じIDと全payloadを固定して設定窓内で有限再送します。ACCEPTED／RUNNING後はREQUESTを再送せずQUERY_STATEに切り替えます。server世代変更・通知失効・応答期限超過で未完了RPCを結果不明として終わらせ、自動再実行しません。server側も要求元の既知・fresh READY世代を必要とします。診断用endpointのrequire_ready=falseはserver自身がSTARTING／FAULTでも照会できる例外で、既知fresh Boot IDは必要です。

serverは受付後response_timeout_msを超えた操作を完了成功へ変更せず、clientも期限外の結果を現在の操作へ適用しません。完了結果はresult_retention_ms保持し、保持期限後もTag guard中は重複を新規実行せずUNKNOWN_OUTCOMEを返します。guardは response_timeout＋result_retention＋max_frame_age 以上、retentionは response_timeout＋max_frame_age 以上に設定します。max_frame_ageはqueue／遅延フレームの運用上限の想定であり、全フレームの実際の年齢をwireから測定する機能ではありません。

### 学祭アプリケーションservice

| endpoint | 宛先／Service | request | response |
| --- | --- | --- | --- |
| pump_set | Node 3／0 | u32LE絶対mask、4byte | u32LE result＋u32LE applied mask、8byte |
| arm_ready | Node 2／0 | 0byte | result/flags＋version 1の3軸診断、48byte |
| arm_target | Node 2／1 | float32LE×3、12byte | result＋echo＋axis confirmation mask、20byte |

pump resultは0=APPLIED、1=INVALID、2=NOT_READY、3=FAULT。APPLIEDはGPIO ODRをactive-lowで読み戻したmaskが指定値と一致したことを意味し、物理圧力の成立を保証しません。旧pump PUBLISHでは状態を上書きしません。

arm resultは0=READY、1=NOT_READY、2=FAULT。flagsはbit0=boot FSM完了、bit1=全3軸feedback新鮮、bit2=全3軸native feedbackの運転状態が2、faultが0、bit3=native送信確認済み、bit4=home switch edge到達。READYには0x1fすべてが必要です。物理静止や任意パラメータのreadback保証ではありません。arm_readyは再原点復帰・モータ再初期化を開始せず、現在の状態だけを返します。

## 世代条件付きSYSTEM_RESET

CONTROL `0x12` RESET_REQUESTと `0x13` RESET_STATUSをunicastで使い、Low5は独立したReset Tagです。Mode=0のMCU system resetだけを実装します。送信元Nodeのallowlistはprofileの各nodeの `reset_sources`。空ならreset無効／capability 0です。ROS自体のSYSTEM_RESET serverは非対応で、ROS宛要求には権限なしを返します。

RESET_REQUESTは48byte固定です。

| offset | 長さ | 内容 |
| --- | --- | --- |
| 0 | 16 | Expected target Boot ID |
| 16 | 16 | Requester Boot ID |
| 32 | 8 | Reset nonce、u64 LE |
| 40 | 4 | Not_after_target_uptime_ms、u32 LE |
| 44 | 1 | Mode=0 |
| 45 | 1 | Flags=0 |
| 46 | 2 | Reserved=0 |

対象の現在Boot ID、許可元、要求元の既知世代・READY・生存期限、Mode／予約値、対象uptime軸の受付期限を検証します。新規要求は締切差が正かつ設定上限以内の場合だけ受理します。同一source・requester世代・nonceの完全一致再送は保存状態を返して再実行せず、内容不一致はCONFLICT、別操作の競合はBUSYです。履歴はRAM内の当該操作のみで、reset後は新Boot IDの照合で旧要求を拒否します。

RESET_STATUSも48byteで、0=要求target Boot ID、16=応答側現在Boot ID、32=nonce、40=応答側uptime、44=Status、45=Mode echo、46–47=0です。

| Status | 意味 |
| --- | --- |
| 0x00／0x01 | PREPARING／ACCEPTED（いずれも再起動完了ではない） |
| 0x10／0x11／0x12 | target世代不一致／受付期限不正／権限なし |
| 0x13／0x14／0x15 | Mode未対応／BUSY／要求元世代無効 |
| 0x16／0x17／0x18／0x1f | 安全化失敗／形式不正／CONFLICT／実行失敗 |

MCUアダプターは安全化し、ACCEPTEDの実送信を確認してresetします。安全化・送信確認が実行期限内に成立しなければresetせず失敗を保存します。学祭armのq0 STOP実送信確認などの機器処理はアダプター側に置きます。

ROSの起動clientは全reset対象の現READYとcapabilityを確認してから、各対象へ1操作を送ります。締切は観測uptime＋観測後の経過＋request_window_ms。同じ操作のTag・nonce・全Dataを固定した有限再送だけを行い、ACCEPTEDまたは新Boot IDの観測で再送を止めます。新世代を観測してもRESET_REQUESTを作り直しません。

全対象で異なるBoot IDの新しいREADYを確認して完了します。STATUSが失われても新READYで完了できます。初期化中もROS自身は通信準備状態のREADYを通知しますが、ROSのactive・指令購読・自動制御開始は全対象完了後です。拒否・通信エラー・全体期限超過は起動失敗で、launchの子を終了し非ゼロで終了します。

## 保証境界

全時間設定は32bit循環比較が成立する2^31ms未満です。timeoutは未実行・停止済みを意味しません。Boot IDは公開情報で認証ではありません。通常PUBLISH／REQUEST／RESPONSEにはBoot IDがないため、世代変更通知より前の旧フレームを完全排除できません。RESETの世代条件付きpayloadとは保証が異なります。8世代を超える過去世代や、uptimeの正進だけでは区別できない遅延通知に対する完全な履歴保証もありません。

peer通知断で既存物理出力を停止する処理は追加しません。接続回復時に保存した指令を再生しません。センサ測定の鮮度と、通信通知の生存時刻は別に扱います。

参考: [RobStride 03公式マニュアル](https://robstride.com/assets/product_manual_robStride03-9c54db7a.pdf) のnative運転状態・応答仕様。運転状態feedbackの確認を、設定パラメータのreadback確認と解釈しません。

### Arm readiness diagnostic version 1

`arm_ready` の応答は48byteです。offset 0のresult、4のflagsに続き、8=boot stage（0〜28は実行step、29=READY、30=CONTROL_ACK）、9=failure reason、10=failed axis（255=無し）、11=version（1）です。offset 12/24/36に各軸の12byte recordを置きます。recordはmode u8、fault u8、validity u8、motor ID u8、feedback age u32LE、last command u8、response state u8、reserved u16LE（0）。validity bit0=ever seen、bit1=fresh(ageがfeedback_max_age_ms未満かつonline)、bit2=fault-free(ever seenかつfault0)、bit3=最後のnative command（初期化・通常制御）の応答確認済み、bit7=FAULT_SNAPSHOTです。失敗時にSTOP送信前の全軸情報を固定し、ageは失敗時点の値です。STARTING/READYのlive値と区別してください。

reasonは0 NONE、1 TX_ENQUEUE_TIMEOUT、2 TX_COMPLETE_TIMEOUT、3 RESPONSE_TIMEOUT、4 MOTOR_FAULT、5 MODE_MISMATCH、6 HOME_TIMEOUT、7 FINAL_FEEDBACK_TIMEOUT、8 CONTROL_FAILURE（既存ADRC guardによる停止要求）。response stateは0 NONE、1 QUEUE_WAIT、2 TX_WAIT、3 RESPONSE_WAIT、4 CONFIRMED、5 FAILED。last commandは0 NONE、1 STOP、2 MODE_OPERATION、3 ENABLE、4 MODE_SPEED、5 VELOCITY、6 ZERO、7 MODE_PP、8 PP_VMAX、9 PP_ACCEL、10 CURRENT_LIMIT、11 MODE_CURRENT、12 CURRENT_ZERO、13 REFRESH、14 POSITION_TARGET、15 CURRENT_REFERENCEです。不明version/enumでもROSは生bytesを記録します。

起動reset失敗時は`startup_diagnostic` endpointへ読み取り専用要求を送り、endpoint名・node・transport outcome・raw hexをERRORログへ出します。全診断で共通の`response_timeout_ms`期限を用い、元の起動失敗を成功へ変えません。例としてrawの先頭`020000000000000005040001`はresult=FAULT、flags=0、stage=5、reason=MOTOR_FAULT、failed axis=0、version=1です。その後の各24桁hexがq0/q1/q2 recordなので、上記offsetで読みます。通常INITのTrigger.messageは同じ情報を軸ごとの名前付き文字列で返します。

### Arm target command

Node2 service1は3軸絶対目標float32LE×3(rad)を要求12byteで受けます。応答20byteはresult u32LE（0=APPLIED、1=INVALID、2=NOT_READY、3=FAULT）、要求目標のecho12byte、native応答確認済み軸mask u32LEです。目標値の定期送信もREQUEST/RESPONSEで行い、旧target PUBLISHは受け付けません。native応答確認は物理到達・静止の保証ではなく、到達判断はcurrent feedbackを使う上位制御が担当します。


native受信はtype2 statusだけを扱い、診断faultはその6bit faultです。type21 FAULT_REPORTのfull32bit値は現在取得しません。type21だけが返る場合は応答timeoutとなり、詳細faultは未取得です。type21をtype2 faultへ推測で写像しません。native statusには要求Tagやparameter index echoがないため、同軸・投入後のstatusを確認する範囲であり、遅延旧statusとの厳密な因果相関を保証しません。

初期化中の通常STOPはtype2 statusを待ちます。SYSTEM_RESETおよびFAULT cleanupの専用q0 STOPは、既存ユーザー承認の実TXevent確認による安全化境界を維持し、mode0応答や物理停止を追加の必須条件とはしません。FAULT時のreadonly照会は再home/enableを行わず、現SYSTEM_RESETのREADY受付条件により、FAULT解除には手動電源再投入等で新READY世代へ復帰させる必要があります。
