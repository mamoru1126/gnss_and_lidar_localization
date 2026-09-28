# tools/grandtour: GrandTour での検証のためのスクリプト

[検証計画: GrandTour](../../docs/validation_grandtour.md) で使うスクリプト。いまあるのは、ダウンロード（3.1 節）と事前確認（3.3 節）の 2 つ。ROS 2 bag への変換・地図の作成・評価は、この後に作る。

| ファイル | 内容 |
|---|---|
| `download.py` | Hugging Face から、ミッションとトピックを絞って落とし、展開する |
| `inspect_grandtour.py` | 落としたミッションを調べ、レポート（`report.md`）を作る |
| `missions.py` | ミッションの一覧（略称 ⇔ フォルダ名。論文の Table 3） |
| `gt_zarr.py` | Zarr の読み込み、静的 TF、UTM への変換 |
| `analysis.py` | 事前確認の計算（numpy だけ） |
| `test/` | 単体テストと、合成ミッションでの一連の確認（`run_tests.sh`。CI の `tools_grandtour` ジョブ） |

## 必要なもの

Docker の `dev` ステージのコンテナには入っている。コンテナの外で動かす場合は:

```bash
pip install numpy matplotlib "zarr>=3.0.7,<4" pyproj huggingface_hub
```

`download.py` だけなら `huggingface_hub` があればよい。データセットは公開されているので、Hugging Face へのログインは要らない（ログインしてあっても問題ない）。

## 1. 軽いトピックを落として、ミッションの組を決める

```bash
export GLL_DATA=/path/to/data     # リポジトリの外
python3 tools/grandtour/download.py --dest $GLL_DATA/grandtour --missions candidates --preset light
python3 tools/grandtour/inspect_grandtour.py --data-dir $GLL_DATA/grandtour --out $GLL_DATA/grandtour/report_light
```

- `candidates` は検証計画の候補の 15 ミッション（ETH・SPX・SBB・ARC・LEICA）。1 ミッションあたり数十 MB。
- 特定のミッションだけなら `--missions ETH-1 ETH-3`（略称かフォルダ名）。何を落とすかは `--dry-run` で確かめられる。
- 一度展開したトピックは、次に実行したときは落とさない。展開した後の `.tar` は消す（残すなら `--keep-tar`）。

`report_light/report.md` と図（`site_*.png`）を Claude に渡す。見るところ:

| 節 | 内容 | 決めること |
|---|---|---|
| 1 | 時間・経路長・GNSS の欠け・最初の静止の時間 | `attitude.static_init_time` |
| 2 | ミッションどうしの経路の重なり（地図 → 照合） | 地図を作るミッションと照合するミッションの組（ETH-1 → ETH-3 か、SBB-1 → SBB-2 か） |
| 3.1 | オドメトリの姿勢の解釈と、速度の座標系 | 変換スクリプトでの姿勢と速度の扱い |
| 3.2 | 静的 TF の解釈（GrandTour のサンプルと同じ解釈で正しいか） | 変換スクリプトでの TF の扱い |
| 3.3 | 真値の精度の目安（リアルタイム解との差、トータルステーションとの差） | 評価の分解能 |

## 2. 選んだミッションの点群と IMU を落とす

```bash
python3 tools/grandtour/download.py --dest $GLL_DATA/grandtour --missions ETH-1 ETH-3 --preset lidar
python3 tools/grandtour/inspect_grandtour.py --data-dir $GLL_DATA/grandtour --missions ETH-1 ETH-3 --out $GLL_DATA/grandtour/report_lidar
```

1 ミッションあたり数百 MB〜1 GB 程度。レポートの 4 節に、点群の列・1 スキャンの点数・base から見た LiDAR の位置と向き・地面から base までの高さ・ロボット自身の点の範囲・IMU の単位が出る。

案 B の地図（DLIO の点群）は `--preset map` で落とす（1 ミッションあたり数百 MB）。

## テスト

```bash
tools/grandtour/test/run_tests.sh
```

GrandTour と同じ構成（配列名・属性・`.tar` の形）の合成ミッションを 3 つ作り、ダウンロード（`--source-dir` で手元から展開）→ 事前確認 → 結果が合成したときの真値に合うか、を確かめる。

## データセットについての注意

- GrandTour のデータは CC BY-SA 4.0（公式ページの表記。Hugging Face のカードは MIT）。出典: "GrandTour: A Legged Robotics Dataset in the Wild for Multi-Modal Perception and State Estimation"（arXiv 2602.18164）。
- 配列名と、静的 TF の解釈は、GrandTour のサンプル（github.com/leggedrobotics/grand_tour_dataset の `examples_hugging_face`。MIT）に合わせている。実データで合っているかは、レポートの 3.1・3.2 節で確かめる。
