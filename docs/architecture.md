# システム構成と制御仕様

## 処理の流れ

1. `llm_controller.py`が日本語の指示を1回受け取る。
2. Windows版Ollamaの`POST /api/generate`へ送信する。
3. API応答の`response`文字列をJSONとして解釈する。
4. `validate_plan()`が全動作を検証し、`(動作種別, 数値)`の列に変換する。
5. 計画を表示し、`yes`が入力された場合だけROS 2を初期化する。
6. `RobotController`が各動作を同期的に実行する。センサのコールバックは`rclpy.spin_once()`で処理する。
7. 動作完了または中断後にゼロ速度送信、ノード破棄、ROS終了を行う。

APIエラーや計画検証エラーではROSノードを作成しません。確認入力で実行しない場合も同様です。前進中に障害物検知で`RuntimeError`が発生した場合、その後の計画は実行しません。

## コンポーネント

| ファイル・環境 | 役割 |
| --- | --- |
| `llm_controller.py` | 入出力、LLM要求、JSON解釈、計画検証、実行確認、動作順序の管理 |
| `robot_controller.py` | ノード`robot_controller`、速度送信、距離・角度制御、LiDAR監視 |
| Windows版Ollama | `qwen3:8b`のローカル推論 |
| ROS 2 / ros_gz_bridge | ROSとGazeboの速度・センサ通信 |
| Gazebo / TurtleBot3 Burger | シミュレーション、オドメトリ・LiDAR配信 |

HTTPは`http://127.0.0.1:11434/api/generate`を使います。検査時に確認したWSLの方式は`mirrored`で、`GET /api/tags`への接続も確認しています。ROS探索は実行手順で`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`を指定します。

## Ollama要求と計画検証

要求パラメータは`model="qwen3:8b"`、`stream=false`、`format="json"`、`think=false`、`options.temperature=0`です。HTTPのタイムアウト設定値は180秒です。自動リトライは実装していません。

標準ライブラリの`urllib.request`を使用し、APIキーはありません。HTTPエラー、接続エラー、タイムアウト、JSON不正、`response`キー欠落などは`main()`の`except Exception`で表示し、実行前に終了します。要求・応答の形式は[Ollama API資料](https://docs.ollama.com/api/generate)も参照してください。

| 項目 | 実際の検証仕様 |
| --- | --- |
| 計画の型 | 辞書 |
| `actions` | 1〜5個の辞書の配列 |
| 前進 | キーは`type`と`distance`のみ、距離0.05〜1.0m |
| 旋回 | キーは`type`と`angle`のみ、角度の絶対値1〜180度 |
| 数値 | `int`または`float`、有限値。`bool`や文字列は拒否 |
| その他の動作 | 拒否 |

各動作内の余分なキーは拒否しますが、計画の最上位にある`actions`以外のキーは検証していません。プロンプトでは未対応の指示を空配列で返すよう求めますが、空配列は検証でエラーになります。

## ROSトピックとQoS

| トピック | 方向（制御ノード基準） | メッセージ型 | QoS |
| --- | --- | --- | --- |
| `/cmd_vel` | Publish | `geometry_msgs/msg/TwistStamped` | 深さ10、既定のReliable / Volatile / Keep Last |
| `/odom` | Subscribe | `nav_msgs/msg/Odometry` | 深さ10、既定のReliable / Volatile / Keep Last |
| `/scan` | Subscribe | `sensor_msgs/msg/LaserScan` | `qos_profile_sensor_data`: Best Effort / Volatile / Keep Last、深さ5 |

インストール済みBurger用ブリッジ設定でも上記3トピックの型が一致することを確認しています。稼働中の配信側QoSとの互換性は、Gazebo起動後に`ros2 topic info --verbose`で確認する必要があります。特に`/odom`の購読はReliableなので、配信側がBest Effortのみの場合は受信できません。

速度メッセージの`header.frame_id`は`base_link`です。前進速度は`twist.linear.x`、旋回速度は`twist.angular.z`で送信します。タイムスタンプはノードの時計から取得し、コード自身は`use_sim_time`を設定しません。

## 前進制御

- 位置情報を最大5秒、次に有効な前方LiDAR距離を最大5秒待つ。
- オドメトリの開始位置から現在位置までの直線距離を`math.hypot()`で計算する。経路長の積算ではない。
- 残り距離が0.01m以下になれば完了。
- 速度は`min(0.12, max(0.03, 残り距離 × 0.6))`m/s。
- 制御ループの`spin_once()`はタイムアウト0.05秒、動作のタイムアウトは実時間20秒。
- 前方距離が無効、または0.30m未満の場合は例外で中断。
- 制御ループの`finally`でゼロ速度を10回、0.05秒間隔で送信する。

## 旋回制御

- クォータニオンからyaw角を計算し、初回取得を最大5秒待つ。
- yawの差分を`atan2(sin(差分), cos(差分))`で正規化して積算する。
- 目標との差の絶対値が2度未満で完了。
- 角速度の絶対値は`min(0.8, max(0.15, 残り角度の絶対値[rad] × 1.5))`rad/s。
- 正の指示は左、負の指示は右。実時間20秒でタイムアウト。
- 制御ループの`finally`でゼロ速度を10回、0.05秒間隔で送信する。

旋回中にLiDARコールバックは処理されますが、距離による停止判定は行いません。

## LiDAR処理と停止の範囲

各点の角度を`angle_min + index × angle_increment`で求め、−π〜πに正規化します。正面±15度の中にある有限かつ正の測距値から最小値を選びます。有効値がなければ`front_distance=None`になります。`range_min`・`range_max`による除外は行いません。

受信時刻やデータの古さは管理していません。データが届かなくなった場合、最後の有効な距離や位置・角度が残ります。初回受信のタイムアウトと、有効値なしのスキャンを受信した場合の停止は、配信途絶の検知とは異なります。

`llm_controller.py`の終了処理でもゼロ速度を10回送信しますが、この10回には待機を挟みません。送信例外、ROSコンテキスト終了、ノード生成失敗などを含む終了処理の課題は[コード確認報告](publication_review.md)に記載しています。全方向の衝突回避、停止完了のフィードバック、独立した非常停止は未実装です。
