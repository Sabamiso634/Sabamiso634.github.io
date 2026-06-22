# Supplementary implementation notes

この補足では、ポスター本文では省略した実装上の対応関係を説明する。特に、神経スパイク列がどのように筋アクチュエーションへ変換され、身体状態がどのようにS1系の入力へ戻されるかを中心にまとめる。

## 1. 実行単位とメインループ

統合シミュレーションの入口は`CBCT-NRP/src/main.cu`である。1回の試行では、`T_MAX = 10050 ms`のシミュレーション時間を1 ms単位で進める。各時刻`t`で、以下の順にモジュールを更新する。

1. 小脳モデルを更新する。
2. 皮質-基底核-視床を含むCBTモデルを更新する。
3. CPGモデルを更新する。
4. CPG、M1、小脳、視床の間のスパイク伝搬を更新する。

皮質-基底核-視床（CBT）モデルは1.0 ms刻みで更新する。一方、小脳とCPGは0.1 ms刻みの内部ダイナミクスを持ち、CPGでは1 msごとに10 substepsを計算して、その1 ms内にスパイクが生じたかを記録する。NRP/Gazeboとの入出力は毎ステップではなく、50 msごとのブロックとして扱う。

50 msブロックが終わるたびに、メインスレッドと`TransferFunction`スレッドを同期する。`TransferFunction`スレッドは、この50 msに発生したM1スパイクをGPUからCPU側へコピーし、筋アクチュエーションとS1入力率を更新する。この設計により、神経計算と身体通信を同じシミュレーション時間上で接続している。

## 2. NRP/Gazeboとの通信

身体モデルとの通信にはROSBridge/WebSocketを使う。接続先は環境変数`NRP_WS_URL`で指定でき、未指定時は`ws://127.0.0.1:9090`へ接続する。

身体側からは、以下のトピックを購読する。

```text
/gazebo_muscle_interface/robot/muscle_states
```

このメッセージから、実装では主に以下の値を取り出す。

- `Humerus1`の筋長
- `Humerus2`の筋長
- `Foot1`の`path_points[2].y`

神経側から身体側へは、以下の6つの筋に対してアクチュエーション値を送る。

```text
/gazebo_muscle_interface/robot/Foot1/cmd_activation
/gazebo_muscle_interface/robot/Foot2/cmd_activation
/gazebo_muscle_interface/robot/Radius1/cmd_activation
/gazebo_muscle_interface/robot/Radius2/cmd_activation
/gazebo_muscle_interface/robot/Humerus1/cmd_activation
/gazebo_muscle_interface/robot/Humerus2/cmd_activation
```

実行開始時には6筋すべてに`0.0`を送信し、その後、50 msごとにM1活動から計算した値を送る。

## 3. M1スパイク列から筋アクチュエーションへの変換

運動指令の読み出しには、M1の深層出力集団`M1_L5B_PT`を用いる。実装では、この集団をニューロンIDで前半と後半の2群に分けている。50 msごとに、その時間窓内で各群に生じたスパイク数を集計する。

2群のスパイク数を直接筋に送るのではなく、まず発火差から1つの状態変数を更新する。

```text
diff = (group_A_spikes - group_B_spikes) / 3000
state <- state + (diff - 0.05 * state) * 1.0 ms
```

ここで`0.05 * state`は漏れ項であり、瞬間的な発火差をそのまま出力するのではなく、短い時間スケールで平滑化された運動状態として扱う。更新後の`state`は`-1.0`から`1.0`にクリップする。

`state`が正の場合は、一方の筋群を駆動する。

```text
Foot1   = state
Radius1 = state
Humerus2 = state
Foot2   = 0
Radius2 = 0
Humerus1 = 0
```

`state`が負の場合は、絶対値を用いて反対側の筋群を駆動する。

```text
Foot1   = 0
Radius1 = 0
Humerus2 = 0
Foot2   = abs(state)
Radius2 = abs(state)
Humerus1 = abs(state)
```

したがって、この実装でのM1読み出しは、`M1_L5B_PT`の2群の活動差を、前肢の2相性・拮抗的な筋駆動へ変換する低次元デコーダである。詳細な脊髄回路や個別運動ニューロンを明示的にモデル化するのではなく、M1集団活動から筋骨格モデルへ送るための最小限の読み出しとして実装している。

## 4. 身体状態からS1入力率への変換

身体モデルから得られる筋長は、そのままS1ニューロンへ入力するのではなく、発火率様の値へ変換する。実装では、`Humerus1`と`Humerus2`の筋長から2種類の入力率`fr1`、`fr2`を計算する。

`fr1`と`fr2`は、あらかじめ設定した3点の筋長、すなわち前方位置、中央位置、後方位置に対応する値を用いて、2次補間により計算する。計算後の値は0から100の範囲にクリップする。ここでの値は、S1系へ入る身体由来入力の発火率として扱う。

実際のコードでは、`fr1`は`latest_humerus1_length`、`fr2`は`latest_humerus2_length`から計算している。`fr1/fr2`の配列では、50 msブロック番号を`k`とすると、`host_fr1_fr2[2*k]`に`fr1`、`host_fr1_fr2[2*k+1]`に`fr2`を保存する。

`fr1`の計算式は以下である。

```text
L1 = latest_humerus1_length
fwd1    = 0.01053
center1 = 0.01195
bwd1    = 0.01337

fr1 =
  50  * (L1 - fwd1) * (L1 - bwd1)
        / ((center1 - fwd1) * (center1 - bwd1))
  + 100 * (L1 - fwd1) * (L1 - center1)
        / ((bwd1 - fwd1) * (bwd1 - center1))

fr1 = min(max(fr1, 0), 100)
```

この式は、`L1 = fwd1`で0、`L1 = center1`で50、`L1 = bwd1`で100となる2次補間である。

`fr2`の計算式は以下である。

```text
L2 = latest_humerus2_length
fwd2    = 0.018309
center2 = 0.0102685
bwd2    = 0.009228

fr2 =
  100 * (L2 - center2) * (L2 - bwd2)
        / ((fwd2 - center2) * (fwd2 - bwd2))
  + 50  * (L2 - fwd2) * (L2 - bwd2)
        / ((center2 - fwd2) * (center2 - bwd2))

fr2 = min(max(fr2, 0), 100)
```

この式は、`L2 = fwd2`で100、`L2 = center2`で50、`L2 = bwd2`で0となる2次補間である。つまり、`fr1`と`fr2`は同じ筋長変化を同じ向きに符号化しているのではなく、前肢運動の2方向に対応するように逆向きの発火率変化として定義している。

50 msごとに計算された`fr1/fr2`は、CPU側のpinned memoryに保存した後、CUDAの非同期コピーでGPU側へ転送する。CBTのstepカーネルでは、`TH_S1_EZ_thalamic_nucleus_TC`集団に対して、この値をポアソンスパイク入力の発火率として使う。

実装上は、`TH_S1_EZ_thalamic_nucleus_TC`の前半ニューロンが`fr1`、後半ニューロンが`fr2`を参照する。各ニューロンでは、以下の確率で身体由来スパイク入力が生成される。

```text
P(spike) = 0.001 * f_rate * DT_
```

ここで`DT_ = 1.0 ms`である。身体由来入力はシミュレーション開始直後には使わず、`t >= 100 ms`以降に有効化している。これにより、初期状態の不安定な身体値がただちにS1系へ入ることを避けている。

この実装では、NRP/Gazeboから得た身体状態が直接S1細胞へ注入されるのではなく、まず視床S1リレーに相当する`TH_S1_EZ_thalamic_nucleus_TC`へポアソン入力として与えられ、その後、既存のTH-to-S1接続を通じてS1活動に反映される。

## 5. CPGからM1への入力

CPGは、2ニューロンからなるペアを100組持つモデルとして実装している。各ペアでは、相互抑制、AHP、ノイズを含むLIF様のダイナミクスを0.1 ms刻みで10 step更新し、1 msごとにスパイクの有無を記録する。

CPGからM1への入力では、CPGニューロンを偶数ID群と奇数ID群に分ける。偶数ID群のスパイクは`M1_L5B_PT`の前半へ、奇数ID群のスパイクは`M1_L5B_PT`の後半へ入力される。各M1ニューロンに入るCPG由来電流は、指数減衰するシナプス状態として更新される。

```text
sum <- exp(-DT_ / tau_syn) * sum + weight * CPG_spike_count
I_CPG <- g_bar * sum
```

現在の設定では、`tau_syn = 1.2 ms`、`weight = 1.5`、`g_bar = 1.0`を用いている。M1のstepカーネルでは、前半ニューロンにはCPG偶数群由来の電流を、後半ニューロンにはCPG奇数群由来の電流を加える。これにより、CPGの2相性活動がM1の2群活動として読み出され、最終的に拮抗筋駆動へ変換される。

## 6. M1、小脳、視床の接続

小脳モデルとの接続では、`M1_L5B_PT`のスパイク列を苔状線維入力として小脳側へ渡している。実装では、各M1スパイクを小脳のglomerulus入力へ複製して書き込む。小脳側のDCNスパイクはhost側で計算されるため、各時刻でGPU側へコピーしてから視床入力に変換する。

DCNからM1側視床への入力は、`TH_M1_EZ_thalamic_nucleus_TC`へ加える。単純な全結合入力ではなく、CPGの相に応じて視床ニューロン群の前半・後半を選択する実装になっている。CPGの一方の相が発火しているときは視床前半群、もう一方の相が発火しているときは視床後半群へDCN由来入力を許可する。

この処理は、M1の2群構造と対応した視床入力を作るための実装である。ただし、今回の発表では小脳による適応的ゲイン更新そのものを主結果とはせず、M1-身体-S1のオンライン閉ループ成立を主な結果として扱う。

## 7. CBT内のシナプス計算

CBTモデルでは、領域内・領域間のシナプス入力をCUDAカーネルで計算する。接続はCSR形式で保持し、postニューロンごとにpreニューロンのコンダクタンスを集計する。

接続の種類に応じて、複数のカーネルを使い分けている。

- `calc_Isyn1`: 一部の接続をまとめて処理する基本カーネル。
- `calc_Isyn2`: postニューロンごとの入力を複数スレッドに分割して計算する。
- `calc_Isyn3`: preニューロンのAMPA/NMDAコンダクタンスをshared memoryに置き、warp shuffleで集約する。
- `calc_Isyn2_thrdprN`: 入次数が小さい接続を1ニューロン1スレッドで処理する。

この分岐は、神経モデルの仮定そのものではなく、大規模SNNを実時間に近い速度で回すための実装上の最適化である。

## 8. 細かい実装部位

実装上の処理は、以下のファイルと関数に分かれている。

### 実行入口とNRP通信

- `CBCT-NRP/src/main.cu`
  - `main()`: NRP/ROSBridgeへ接続し、CB、小脳、CPGを初期化してメインループを回す。
  - `muscleStatesCallback()`: NRP/Gazeboから`muscle_states`を受け取り、`Humerus1`、`Humerus2`の筋長と`Foot1`位置を更新する。
  - `compute_fr1()`, `compute_fr2()`: 筋長をS1系への入力率へ変換する。
  - `TransferFunction()`: 50 msごとにM1スパイク列を集計し、筋アクチュエーションのpublishと`fr1/fr2`のGPU転送を行う。

`main()`では、シミュレーション本体を1 msごとに進める。一方、NRP/Gazeboとの通信とM1読み出しは`TransferFunction()`でまとめて処理する。両者は50 msブロックごとに`pthread_barrier`で同期する。

### 時間刻みと主要定数

- `CBCT-NRP/inc/option.h`
  - `T_MAX = 10050`: シミュレーション時間。
  - `DT = 0.1`: 小脳とCPGの内部刻み幅。
  - `DT_ = 1.0`: CBTモデルの刻み幅。
  - `DEV_NUM = 1`: 単一GPU実行。
  - `nthreads = 8`: CBT側のOpenMP/CUDA stream分割に使うスレッド数。

### M1スパイク集計と筋出力

- `CBCT-NRP/src/main.cu`
  - `m1spike_DtoH_Async(...)`: `M1_L5B_PT`のスパイク列をGPUからCPU側へコピーする。
  - `h_rate0`, `h_rate1`: 50 ms窓ごとのM1 2群スパイク数を保存する。
  - `rates[0..5]`: 6筋へpublishするアクチュエーション値を保持する。

筋への対応は以下である。

```text
rates[0] -> Foot1
rates[1] -> Foot2
rates[2] -> Radius1
rates[3] -> Radius2
rates[4] -> Humerus1
rates[5] -> Humerus2
```

`state >= 0`では`Foot1`, `Radius1`, `Humerus2`を駆動し、`state < 0`では`Foot2`, `Radius2`, `Humerus1`を駆動する。

### S1フィードバック用メモリ

- `CBCT-NRP/inc/GPUCBTSimulation.h`
  - `NRPtoS1`: `host_fr1_fr2`と`dev_fr1_fr2`を持つ構造体。
- `CBCT-NRP/src/GPUCBTSimulation.cu`
  - `CBT_Initialize()`: `T_MAX / 50`ブロック分の`fr1/fr2`配列を確保する。
  - `step(...)`: `TH_S1_EZ_thalamic_nucleus_TC`で`fr1/fr2`をポアソン入力として使う。

`host_fr1_fr2`はpinned memoryとして確保している。これは、50 msごとにCPU側で更新された`fr1/fr2`をGPUへ転送するためである。

### name_idによる特殊処理

CBTの通常ニューロンは同じLIF更新式で処理するが、一部の集団には`name_id`を付けて特殊な入力を加えている。

```text
name_id = 1: TH_M1_EZ_thalamic_nucleus_TC
  DCN由来入力を加える。

name_id = 2: M1_L5B_PT
  CPG由来入力を加える。
  スパイク列をM1読み出し用バッファにも保存する。

name_id = 3: TH_S1_EZ_thalamic_nucleus_TC
  NRP/Gazebo由来のfr1/fr2からポアソンスパイク入力を生成する。
```

この設計により、既存のCBT回路を大きく作り替えずに、CPG入力、DCN入力、身体由来S1入力だけを特定集団へ追加している。

### CPG

- `CBCT-NRP/src/CPG_Simulation.cu`
  - `CPG_Initialize()`: CPGの膜電位、シナプス電流、AHP電流、乱数状態を初期化する。
  - `CPG_Loop()`: 1 msごとに0.1 ms刻みで10 substepsを実行する。
  - `ExportSpikeRaster()`, `ExportCPGSpikeCSV()`: CPGスパイク列をCSVへ出力する。
- `CBCT-NRP/inc/CPG_Simulation.h`
  - `SET_ = 100`: 2ニューロンペアの数。
  - `N_num = 2`: 1ペアあたりのニューロン数。
  - `W_cpg = -80.0`: ペア内の相互抑制重み。
  - `TAU_AHP = 350.0`: AHP電流の時定数。

CPGのスパイク行列は`d_spike_matrix`に`time x neuron`形式で保存される。この行列を、M1入力と図生成の両方に使う。

### CPG、M1、小脳、視床間のスパイク伝搬

- `CBCT-NRP/src/Spike_propagation.cu`
  - `K_CPG_to_M1_Isyn`: CPG偶数群/奇数群の活動を、M1前半/後半へ分けて入力する。
  - `K_M1_to_MF`: `M1_L5B_PT`スパイクを小脳の苔状線維入力へ渡す。
  - `K_DCN_to_TH_cpg`: DCNスパイクをCPG相に応じて`TH_M1_EZ_thalamic_nucleus_TC`へ入力する。
  - `Spike_Prop()`: 上記3つの伝搬処理を1 msごとに呼び出す。

この部分が、CPG、M1、小脳、視床を単に並列に動かすだけでなく、同じ時刻のスパイク列として接続している実装部位である。

### CBT内のシナプス計算

- `CBCT-NRP/src/GPUCBTSimulation.cu`
  - `CBT_loop()`: CBT全体のシナプス入力計算とニューロン更新を呼び出す。
  - `calc_Isyn1`, `calc_Isyn2`, `calc_Isyn3`, `calc_Isyn2_thrdprN`: 接続タイプや入次数に応じて使い分けるシナプス電流計算カーネル。
  - `CBT_Finalize()`: スパイク列、M1集計、身体位置ログを出力する。

`calc_Isyn3`では、preニューロンのAMPA/NMDAコンダクタンスをshared memoryに置き、warp shuffleで分割和を集約する。これは神経モデル上の新しい仮定ではなく、大規模SNNの計算を高速化するための実装である。

### 接続生成

- `CBCT-NRP/src/GPUCBTSimulation.cu`
  - `init_connect_host()`, `init_connect_dev()`: 接続行列を生成し、CSR形式でGPU側へ転送する。
- `CBCT-NRP/src/Connection_const.cu`
  - 領域間接続の種類を定義する。
- `CBCT-NRP/src/THtoS1_connection_const.cu`
  - `TH_S1_EZ_thalamic_nucleus_TC`からS1への接続を定義する。
- `CBCT-NRP/src/THtoM1_connection_const.cu`
  - `TH_M1_EZ_thalamic_nucleus_TC`からM1への接続を定義する。

特に`TH_S1_EZ_thalamic_nucleus_TC -> S1_L4_Pyr`と`TH_M1_EZ_thalamic_nucleus_TC -> M1_L5B_PT`では、pre/post集団を前半・後半に分けて対応させる処理が入っている。これは、`fr1/fr2`やCPGの2相性入力を、皮質側の2群構造へ保ったまま伝えるための実装である。

## 9. 出力ログ

実行後、神経活動と身体関連の補助変数をファイルへ出力する。

- `cbt_results/`: M1、S1、BG、TH、PGなど各領域のスパイク時刻。
- `spike_output/spike_result_trial0.dat`: 小脳モデル側のスパイク出力。
- `cpg_spikes_raster.csv`: CPGスパイクラスタ。
- `log/cpg_spike_CSV.csv`: CPGスパイクCSV。
- `log/m1_rates_50ms.csv`: 50 ms窓ごとのM1 2群スパイク集計。
- `log/foot1_path_points.csv`: NRP/Gazeboから受け取った`Foot1`位置の記録。
- `elapsed_time/runXXXX_realtime_summary.csv`: 実行時間とリアルタイム比の概要。
- `elapsed_time/runXXXX_loop_metrics.csv`: 50 msブロックごとの遅延計測。

現時点では、6筋すべてにpublishした`cmd_activation`値の直接ログは保存していない。そのため、Fig. 1の筋アクチュエーションは、保存済みの`m1_rates_50ms.csv`から、上記と同じM1読み出し規則で再構成している。

## 10. 掲載時の注意

この補足で説明しているNRP/Gazebo前肢モデルが、今回の主結果に対応する実行系である。MuJoCo動画は、自由行動型の仮想齧歯類モデルへ拡張するための別プロトタイプとして扱う。

また、リアルタイム性の数値をWebサイトに掲載する場合は、NRP/ROSBridgeを再起動したfreshな条件で再測定し、run ID、GPU、CUDA/コンテナ、NRP起動条件を併記する。
