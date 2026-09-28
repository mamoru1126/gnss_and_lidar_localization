#!/usr/bin/env python3
"""GrandTour のデータを、ミッションとトピックを絞って Hugging Face から落とし、展開する（README.md 参照）。

  python3 download.py --dest $GLL_DATA/grandtour --missions candidates --preset light
  python3 download.py --dest $GLL_DATA/grandtour --missions ETH-1 ETH-3 --preset lidar
  python3 download.py --dest $GLL_DATA/grandtour --missions ETH-1 --preset map

Hugging Face 上では、トピックごとに Zarr のグループを 1 つの .tar にまとめてある
（<フォルダ名>/data/<トピック>.tar）。これを <dest>/<フォルダ名>/data/<トピック>/ に展開する。
展開したトピックには印（data/.<トピック>.extracted）を付け、次に実行したときは落とさない。

必要なパッケージは huggingface_hub だけ（--source-dir で手元のコピーから展開するときは不要）。
"""
import argparse
import fnmatch
import json
import shutil
import sys
import tarfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import missions  # noqa: E402

REPO_ID = "leggedrobotics/grand_tour_dataset"
BROKEN_ROOT_ZGROUP = b'{\n a   "zarr_format": 3\n'
CORRECT_ROOT_ZGROUP = b'{\n    "zarr_format": 2\n}'

# 軽いトピック: 真値・軌跡・静的 TF（1 ミッション数十 MB）。ミッションの組を決める事前確認に使う
LIGHT = [
    "navsatfix_cpt7_ie_tc",   # 後処理の GNSS 解（緯度経度・共分散）
    "cpt7_ie_tc_odometry",    # 同じ解の姿勢（密結合。最も精度が高い）
    "cpt7_ie_rt_odometry",    # リアルタイムの PPP 解（精度が低い。比較用）
    "gnss_raw_cpt7_ie_tc",    # GNSS の解（ECEF と標準偏差。第 2 段階で GNSS の入力を作る元）
    "prism_position",         # トータルステーションで測ったプリズムの位置
    "anymal_state_odometry",  # 脚のオドメトリ（ODOM に使う）
    "dlio_map_odometry",      # DLIO の軌跡（案 B の地図のアンカーを決める）
    "tf",                     # 静的 TF
]
# 照合に使う点群と IMU
LIDAR = LIGHT + ["livox_points_undistorted", "livox_imu", "adis_imu"]

PRESETS = {"light": LIGHT, "lidar": LIDAR, "map": []}


def patterns_for(folder, topics, with_map):
    pats = [f"{folder}/metadata/*.yaml", f"{folder}/data/.zgroup"]
    pats += [f"{folder}/data/{t}.tar" for t in topics]
    if with_map:
        pats.append(f"{folder}/point_cloud_maps/*.ply")
    return pats


def marker(dest, folder, topic):
    return dest / folder / "data" / f".{topic}.extracted"


def repair_root_zgroup(path):
    """data/.zgroup が壊れているミッションがある（Hugging Face 上のファイルが JSON として読めない。
    例: BROKEN_ROOT_ZGROUP）。JSON として読めないか、Zarr v2 のグループでなければ、v2 のグループとして書き直す。"""
    if not path.is_file():
        return False
    try:
        ok = json.loads(path.read_bytes()).get("zarr_format") == 2
    except (ValueError, AttributeError):
        ok = False
    if not ok:
        path.write_bytes(CORRECT_ROOT_ZGROUP)
    return not ok


def safe_extract(tar_path, out_dir):
    """tar を out_dir に展開する（out_dir の外に書くメンバーは拒否する）。展開した最上位の名前を返す。"""
    out_dir.mkdir(parents=True, exist_ok=True)
    root = out_dir.resolve()
    with tarfile.open(tar_path, "r") as tar:
        members = tar.getmembers()
        for m in members:
            target = (out_dir / m.name).resolve()
            if root != target and root not in target.parents:
                raise RuntimeError(f"{tar_path}: unsafe member path {m.name}")
            if m.issym() or m.islnk():
                raise RuntimeError(f"{tar_path}: links are not allowed ({m.name})")
        if hasattr(tarfile, "data_filter"):
            tar.extractall(out_dir, filter="data")
        else:
            tar.extractall(out_dir)
    return sorted({Path(m.name).parts[0] for m in members if m.name not in ("", ".")})


def fetch(src_root, patterns, local_dir):
    """patterns に合うファイルを取ってきて、その置き場所（フォルダ）を返す。"""
    if src_root is not None:
        return src_root  # 手元のコピー（テスト用）
    try:
        from huggingface_hub import snapshot_download
    except ImportError:
        sys.exit("huggingface_hub がない: pip install huggingface_hub")
    local_dir.mkdir(parents=True, exist_ok=True)
    return Path(snapshot_download(repo_id=REPO_ID, repo_type="dataset",
                                  allow_patterns=patterns, local_dir=str(local_dir)))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dest", required=True, type=Path, help="展開先（例: $GLL_DATA/grandtour）")
    ap.add_argument("--missions", nargs="+", required=True,
                    help="略称（ETH-1）かフォルダ名。'candidates' で検証計画の候補の全部")
    ap.add_argument("--preset", choices=PRESETS, default="light")
    ap.add_argument("--topics", nargs="*", default=[], help="追加で落とすトピック（例: livox_points）")
    ap.add_argument("--keep-tar", action="store_true", help="展開した後も .tar を残す")
    ap.add_argument("--source-dir", type=Path, help="Hugging Face の代わりに、同じ構成の手元のフォルダから展開する（テスト用）")
    ap.add_argument("--dry-run", action="store_true", help="落とすものを表示するだけ")
    args = ap.parse_args()

    folders = missions.resolve(args.missions)
    topics = list(dict.fromkeys(PRESETS[args.preset] + args.topics))
    with_map = args.preset == "map"
    dest = args.dest.expanduser()
    download_dir = dest / ".hf_download"

    patterns, todo = [], []
    for f in folders:
        need = [t for t in topics if not marker(dest, f, t).exists()]
        todo.append((f, need))
        have_map = any((dest / f / "point_cloud_maps").glob("*.ply"))
        patterns += patterns_for(f, need, with_map and not have_map)
        print(f"{missions.code_of(f):8s} {f}: {len(need)} topics to fetch" + (" + map" if with_map else ""))
    if args.dry_run:
        print("\n".join(patterns))
        return

    src = fetch(args.source_dir, patterns, download_dir)

    problems = []
    for f, need in todo:
        mdir = dest / f
        # メタデータ・.zgroup・地図はそのまま置く
        for p in [f"{f}/metadata/*.yaml", f"{f}/data/.zgroup"] + ([f"{f}/point_cloud_maps/*.ply"] if with_map else []):
            base = src / Path(p).parent
            if not base.is_dir():
                continue
            for s in base.iterdir():
                if s.is_file() and fnmatch.fnmatch(s.name, Path(p).name):
                    d = mdir / s.relative_to(src / f)
                    d.parent.mkdir(parents=True, exist_ok=True)
                    if src == download_dir and not args.keep_tar and s.suffix == ".ply":
                        shutil.move(str(s), d)
                    else:
                        shutil.copy2(s, d)
        if repair_root_zgroup(mdir / "data" / ".zgroup"):
            print(f"  repaired malformed root metadata for {missions.code_of(f)}")
        for t in need:
            tar_path = src / f / "data" / f"{t}.tar"
            if not tar_path.exists():
                problems.append(f"{missions.code_of(f)}: {t}.tar が無い（このミッションには無いトピックかもしれない）")
                continue
            tops = safe_extract(tar_path, mdir / "data")
            if t not in tops:
                problems.append(f"{missions.code_of(f)}: {t}.tar の中身が想定と違う（最上位: {tops}）")
                continue
            marker(dest, f, t).touch()
            if src == download_dir and not args.keep_tar:
                tar_path.unlink()
            print(f"  extracted {missions.code_of(f)}/{t}")

    if problems:
        print("\n注意:")
        for p in problems:
            print("  - " + p)
    print(f"\n完了: {dest}")


if __name__ == "__main__":
    main()
