# tools/grandtour: GrandTour での検証のためのスクリプト

公開データセット GrandTour で検証するために作ったスクリプト（**今は使っていない**。検証は [AWSIM](../../docs/validation_awsim.md) で行うことにし、GrandTour の検証計画 `docs/validation_grandtour.md` は削除した。節の番号はその文書のもので、git の履歴で見られる）。ダウンロード（3.1 節）、事前確認（3.3 節）、ROS 2 bag への変換（3.2 節）、案 A の地図の作成（4.3 節）。案 B のアンカー決定と評価は、この後に作る。

| ファイル | 内容 |
|---|---|
| `download.py` | Hugging Face から、ミッションとトピックを絞って落とし、展開する |
| `inspect_grandtour.py` | 落としたミッションを調べ、レポート（`report.md`）を作る |
| `grandtour_to_bag.py` | ミッションを、推定ノード用の ROS 2 bag（MCAP）にする。真値の CSV とパラメータのファイルも作る |
| `build_map_from_gt.py` | 案 A の点群地図（真値の姿勢で Livox の点群を重ねる）と `maps.yaml` を作る |
| `missions.py` | ミッションの一覧（略称 ⇔ フォルダ名。論文の Table 3） |
| `gt_zarr.py` | Zarr の読み込み、静的 TF、UTM への変換、真値（base の姿勢） |
| `analysis.py` | 事前確認の計算（numpy だけ） |
| `test/` | 単体テストと、合成ミッションでの一連の確認（`run_tests.sh`。CI の `tools_grandtour` ジョブ） |

## 必要なもの

Docker の `dev` ステージのコンテナには入っている。コンテナの外で動かす場合は:

```bash
pip install numpy matplotlib "zarr>=3.0.7,<4" pyproj "huggingface_hub<2"
```

#### memo
devcontainer 内では

```bash
python3 -m pip install --break-system-packages 'huggingface_hub<2' 
```

が必要だった（`huggingface_hub` 2 系では動かないため、`docker/Dockerfile` で `<2` に固定した。Dockerfile を変えた後は、
VS Code の「Dev Containers: Rebuild Container」でコンテナを作り直すと入るので、手で入れる必要はなくなる）

Hugging Face 上の `data/.zgroup` が壊れている（JSON として読めない）ミッションがある。`download.py` が展開のときに
Zarr v2 のグループとして書き直す（`repaired malformed root metadata` と表示される）。

`download.py` だけなら `huggingface_hub` があればよい。データセットは公開されているので、Hugging Face へのログインは要らない（ログインしてあっても問題ない）。

## 1. 軽いトピックを落として、ミッションの組を決める

```bash
export GLL_DATA=/path/to/data     # リポジトリの外
python3 tools/grandtour/download.py --dest $GLL_DATA/grandtour --missions candidates --preset light
python3 tools/grandtour/inspect_grandtour.py --data-dir $GLL_DATA/grandtour --out $GLL_DATA/grandtour/report_light
```

- `candidates` は検証計画の候補の 15 ミッション（ETH・SPX・SBB・ARC・LEICA）。1 ミッションあたり数十 MB。
- 特定のミッションだけなら `--missions SPX-2 SPX-3`（略称かフォルダ名）。何を落とすかは `--dry-run` で確かめられる。
- 一度展開したトピックは、次に実行したときは落とさない。展開した後の `.tar` は消す（残すなら `--keep-tar`）。

`report_light/report.md` と図（`site_*.png`）を Claude に渡す。見るところ:

| 節 | 内容 | 決めること |
|---|---|---|
| 1 | 時間・経路長・GNSS の欠け・最初の静止の時間 | `attitude.static_init_time` |
| 2 | ミッションどうしの経路の重なり（地図 → 照合） | 地図を作るミッションと照合するミッションの組（結果: SPX-2 → SPX-3。検証計画 1.3 節） |
| 3.1 | オドメトリの姿勢の解釈と、速度の座標系 | 変換スクリプトでの姿勢と速度の扱い |
| 3.2 | 静的 TF の解釈（GrandTour のサンプルと同じ解釈で正しいか） | 変換スクリプトでの TF の扱い |
| 3.3 | 真値の精度の目安（リアルタイム解との差、トータルステーションとの差） | 評価の分解能 |

## 2. 選んだミッションの点群と IMU を落とす

```bash
python3 tools/grandtour/download.py --dest $GLL_DATA/grandtour --missions SPX-2 SPX-3 --preset lidar
python3 tools/grandtour/inspect_grandtour.py --data-dir $GLL_DATA/grandtour --missions SPX-2 SPX-3 --out $GLL_DATA/grandtour/report_lidar
```

1 ミッションあたり数百 MB〜1 GB 程度。レポートの 4 節に、点群の列・1 スキャンの点数・base から見た LiDAR の位置と向き・地面から base までの高さ・ロボット自身の点の範囲・IMU の単位が出る。

案 B の地図（DLIO の点群）は `--preset map` で落とす（1 ミッションあたり数百 MB）。

## 3. 案 A の地図を作る

SPX-2 の点群を真値の姿勢で重ね、SPX-3 の経路の近く（水平 30 m、高さ −1.5〜+8 m）だけを残す。

```bash
G=$GLL_DATA/grandtour
python3 tools/grandtour/build_map_from_gt.py $G/2024-11-02-17-18-32 $G/maps/spx2 --near $G/2024-11-02-17-43-10
tiled_pcd_map_tiler -i $G/maps/spx2/map.pcd -o $G/maps/spx2/tiles --tile-size 20 --voxel-size 0.2
```

- `map.pcd` の座標は、SPX-2 の最初の base の位置を原点にした ENU（実距離）。`maps.yaml` のアンカー（UTM）は、真値の当てはめから決まる。
- 真値の水平 σ が 5 cm を超える時刻のスキャンは使わない（`--max-sigma`）。SPX-2 は σ p95 が 10 cm なので、一部のスキャンが除かれる。
- 地図の品質（壁の厚み、段差、人や車の残像）は、`map.pcd` を CloudCompare などで開いて目で見る。

## 4. ROS 2 bag にして、推定ノードで再生する

```bash
python3 tools/grandtour/grandtour_to_bag.py $G/2024-11-02-17-43-10 $G/bags/spx3.mcap --map-config $G/maps/spx2/maps.yaml
ros2 launch gll_ros2 localizer.launch.py params_file:=$G/bags/spx3_params.yaml use_sim_time:=true
ros2 bag play $G/bags/spx3.mcap --clock     # 別の端末で
```

- 出力: `spx3.mcap`（bag）、`spx3_groundtruth.csv`（真値。base_link、UTM）、`spx3_params.yaml`（`config/localizer.yaml` に、UTM の帯・LiDAR の外部パラメータ・地面から base までの高さ・静止初期化の時間・地図を入れたもの。推定ノードの出力の CSV は `spx3_output.csv`）。
- トピックは launch の既定値（`/sensing/imu`、`/sensing/odom`、`/sensing/lidar/points`、`/initialpose`）。GNSS は入れない（第 1 段階）。
- 初期姿勢は、静止初期化が終わった 0.5 s 後に、真値の位置で出す。`--init-error 2,15` で 2 m・15° ずらす（V-G2）。
- 障害: `--odom-scale 1.05 --odom-noise 0.05`（V-G3）、`--drop-lidar 60:10`（60 s から 10 s。V-G4）。
- `lidar.crop_box_*`（ロボット自身の点を除く箱）はまだ既定値。lidar の段階のレポートの 4 節を見て決める。

## テスト

```bash
tools/grandtour/test/run_tests.sh
```

GrandTour と同じ構成（配列名・属性・`.tar` の形）の合成ミッションを 3 つ作り、ダウンロード（`--source-dir` で手元から展開）→ 事前確認 → bag への変換 → 案 A の地図、の結果が合成したときの真値に合うかを確かめる。ROS 2 があれば（CI の ros2 ジョブ）、bag を rosbag2 で読み、地図を `tiled_pcd_map_tiler` でタイル化するところまで確かめる。

## データセットについての注意

- GrandTour のデータは CC BY-SA 4.0（公式ページの表記。Hugging Face のカードは MIT）。出典: "GrandTour: A Legged Robotics Dataset in the Wild for Multi-Modal Perception and State Estimation"（arXiv 2602.18164）。
- 配列名と、静的 TF の解釈は、GrandTour のサンプル（github.com/leggedrobotics/grand_tour_dataset の `examples_hugging_face`。MIT）に合わせている。実データで合っているかは、レポートの 3.1・3.2 節で確かめる。
