"""GrandTour のミッションの一覧（論文 arXiv 2602.18164 の Table 3 より）。

Hugging Face のフォルダ名はミッションの開始時刻（YYYY-MM-DD-HH-MM-SS）。
"""

# (略称, フォルダ名, 時間 [s], 距離 [m], MS60 の範囲 [%], GNSS, タグ)
MISSIONS = [
    ("ETH-1", "2024-10-01-11-29-55", 333, 204.7, 74.9, "Yes", "Outdoor, Pavement, Cars, People, Rain, Cloudy"),
    ("ETH-2", "2024-10-01-11-47-44", 418, 210.6, 67.8, "No", "Indoor, People, Landmarks, Dynamic Objects"),
    ("ETH-3", "2024-10-01-12-00-49", 464, 214.5, 6.5, "Yes", "Urban, Outdoor, Pavement, Crosswalk, Cars, People, Rain, Cloudy"),
    ("SPX-1", "2024-11-02-17-10-25", 374, 147.1, 75.7, "Yes", "Industrial, Outdoor, Metal, Stairs, People, Dawn"),
    ("SPX-2", "2024-11-02-17-18-32", 383, 184.1, 68.2, "Yes", "Industrial, Outdoor, Metal, Stairs, People, Dawn"),
    ("SPX-3", "2024-11-02-17-43-10", 213, 101.3, 82.3, "Yes", "Industrial, Outdoor, Metal, People, Night"),
    ("ICE-1", "2024-11-02-21-12-51", 251, 56.3, 91.7, "Yes", "Indoor, Ice, People"),
    ("SNOW-1", "2024-11-03-07-52-45", 172, 36.5, 47.4, "Yes", "Search & Rescue, Mountain, Outdoor, Snow, Sunlight"),
    ("SNOW-2", "2024-11-03-07-57-34", 261, 173.9, 99.4, "Yes", "Search & Rescue, Mountain, Outdoor, Snow, Sunlight"),
    ("SNOW-3", "2024-11-03-08-17-23", 236, 145.3, 99.9, "Yes", "Search & Rescue, Mountain, Outdoor, Snow, Sunlight"),
    ("EIG-1", "2024-11-03-13-51-43", 429, 219.7, 70.4, "Yes", "Search & Rescue, Mountain, Outdoor, Gravel/Dirt, Trail, Pavement, Metal, Stairs, People, Tracks, Sunlight"),
    ("EIG-2", "2024-11-03-13-59-54", 348, 198.3, 72.3, "Yes", "Search & Rescue, Mountain, Outdoor, Gravel/Dirt, Trail, Pavement, Metal, Stairs, People, Tracks, Sunlight"),
    ("GRI-1", "2024-11-04-10-57-34", 455, 266.7, 58.1, "Yes", "Urban, Outdoor, Grass, Pavement, Stairs, Cars, People, Sunlight"),
    ("CYN-1", "2024-11-04-12-55-59", 510, 297.7, 24.0, "Yes", "Search & Rescue, Forest, Mountain, Outdoor, Gravel/Dirt, Trail, Pavement, Sunlight"),
    ("CYN-2", "2024-11-04-13-07-13", 314, 164.6, 49.3, "Yes", "Search & Rescue, Forest, Mountain, Outdoor, Gravel/Dirt, Trail, Pavement, Sunlight"),
    ("HIL-1", "2024-11-04-16-05-00", 430, 326.1, 72.2, "Yes", "Forest, Outdoor, Mountain, House"),
    ("RIV-1", "2024-11-04-16-52-38", 537, 204.0, 64.1, "No", "Forest, Water, Outdoor, Gravel/Dirt, Sand, Trail, Cars, Sunlight"),
    ("PIL-1", "2024-11-11-12-07-40", 946, 470.5, 21.0, "Yes", "Mountain, Outdoor, Pavement, Cobblestone, Stairs, People, Sunlight"),
    ("PIL-2", "2024-11-11-12-42-47", 409, 273.6, 98.7, "Yes", "Mountain, Outdoor, Gravel/Dirt, Trail, Stairs, People, Sunlight"),
    ("ROOT-1", "2024-11-11-14-29-44", 340, 177.5, 69.2, "Yes", "Search & Rescue, Forest, Mountain, Outdoor, Gravel/Dirt, Trail, Sunlight"),
    ("HAUS-1", "2024-11-11-16-14-23", 337, 170.5, 66.0, "No", "Indoor, Pavement, Cars, People"),
    ("HÖB-1", "2024-11-14-11-17-02", 517, 289.3, 39.2, "Partially", "Forest, Outdoor, Trail, Cloudy"),
    ("HÖB-2", "2024-11-14-12-01-26", 445, 197.3, 27.9, "Partially", "Forest, Outdoor, Trail, Cloudy"),
    ("HEAP-1", "2024-11-14-13-45-37", 290, 162.0, 86.7, "Yes", "Water, Industrial, Outdoor, Gravel/Dirt, Grass, Trail, Cloudy"),
    ("KÄB-1", "2024-11-14-14-36-02", 465, 205.4, 79.3, "Yes", "Forest, Outdoor, Grass, Trail, Cloudy"),
    ("KÄB-2", "2024-11-14-15-22-43", 383, 185.6, 41.1, "Yes", "Forest, Outdoor, Gravel/Dirt, Trail, Cloudy"),
    ("KÄB-3", "2024-11-14-16-04-09", 475, 364.6, 48.3, "Yes", "Outdoor, Gravel/Dirt, Trail, Pavement, People, Cloudy"),
    ("TRIM-1", "2024-11-15-10-16-35", 465, 267.8, 72.5, "Yes", "Forest, Outdoor, Gravel/Dirt, Trail, People, Cloudy"),
    ("ALB-1", "2024-11-15-11-18-14", 282, 108.0, 41.3, "Yes", "Forest, Outdoor, Gravel/Dirt, Trail, People, Cloudy"),
    ("ALB-2", "2024-11-15-11-37-15", 508, 206.6, 65.4, "Yes", "Forest, Outdoor, Gravel/Dirt, Mud, Trail, Cloudy"),
    ("ALB-3", "2024-11-15-12-06-03", 209, 78.0, 89.0, "Yes", "Forest, Outdoor, Gravel/Dirt, Mud, Trail, Cloudy"),
    ("LMB-1", "2024-11-15-14-14-12", 528, 205.0, 58.1, "Yes", "Forest, Outdoor, Grass, Trail, Cloudy"),
    ("LMB-2", "2024-11-15-14-43-52", 244, 100.9, 18.8, "Yes", "Forest, Outdoor, Gravel/Dirt, Trail, Cloudy"),
    ("LEE-1", "2024-11-15-16-41-14", 529, 331.8, 71.6, "Partially", "Underground, Industrial, Indoor, Outdoor, Pavement, People, Cloudy"),
    ("ARC-1", "2024-11-18-12-05-01", 433, 247.9, 71.3, "Yes", "Search & Rescue, Industrial, Urban, Indoor, Outdoor, Pavement, Cobblestone, Metal, Stairs, Cloudy"),
    ("ARC-2", "2024-11-18-13-22-14", 424, 136.2, 27.3, "Yes", "Search & Rescue, Water, Industrial, Urban, Indoor, Outdoor, Pavement, Stairs, Cloudy"),
    ("ARC-3", "2024-11-18-13-48-19", 713, 332.0, 27.5, "Partially", "Search & Rescue, Industrial, Urban, Indoor, Outdoor, Grass, Pavement, Stairs, Cloudy"),
    ("ARC-4", "2024-11-18-15-46-05", 397, 160.0, 20.3, "Yes", "Search & Rescue, Industrial, Urban, Indoor, Outdoor, Grass, Pavement, Stairs, Cloudy"),
    ("ARC-5", "2024-11-18-16-59-23", 391, 173.3, 42.1, "Yes", "Search & Rescue, Industrial, Urban, Indoor, Outdoor, Grass, Pavement, Cobblestone, Dawn"),
    ("ARC-6", "2024-11-18-17-13-09", 349, 103.5, 62.9, "No", "Search & Rescue, Industrial, Indoor, Smoke, Pavement"),
    ("ARC-7", "2024-11-18-17-31-36", 322, 157.2, 35.8, "No", "Search & Rescue, Industrial, Urban, Indoor, Outdoor, Smoke, Pavement, Metal, Stairs, Night"),
    ("LEICA-1", "2024-11-25-14-57-08", 310, 152.7, 80.2, "Yes", "Urban, Indoor, Outdoor, Pavement, Cars, People, Sunlight"),
    ("LEICA-2", "2024-11-25-16-36-19", 422, 208.2, 46.7, "Yes", "Indoor & Outdoor, Warehouse, People, Industrial"),
    ("SBB-1", "2024-12-03-13-15-38", 502, 311.3, 99.5, "Yes", "Industrial, Outdoor, Gravel/Dirt, Tracks, Sunlight"),
    ("SBB-2", "2024-12-03-13-26-40", 271, 149.2, 73.2, "Yes", "Industrial, Outdoor, Gravel/Dirt, Tracks, Sunlight"),
    ("CON-1", "2024-12-09-09-34-43", 388, 263.3, 76.0, "No", "Water, Construction, Indoor, Outdoor, Gravel/Dirt, Woodplanks, People, Cloudy"),
    ("CON-2", "2024-12-09-09-41-46", 384, 266.4, 61.2, "No", "Water, Construction, Indoor, Outdoor, Gravel/Dirt, People, Cloudy"),
    ("CON-3", "2024-12-09-11-28-28", 796, 188.9, 23.6, "No", "Construction, Indoor, Outdoor, Stairs, People, Cloudy"),
    ("CON-4", "2024-12-09-11-53-11", 743, 468.3, 67.1, "No", "Construction, Outdoor, Mud, Stairs, People, Cloudy"),
]

# 検証計画（docs/validation_grandtour.md 1.3 節）の候補
CANDIDATES = ["ETH-1", "ETH-2", "ETH-3", "SPX-1", "SPX-2", "SPX-3", "SBB-1", "SBB-2",
              "ARC-1", "ARC-2", "ARC-3", "ARC-4", "ARC-5", "LEICA-1", "LEICA-2"]

_BY_CODE = {m[0].upper(): m for m in MISSIONS}
_BY_FOLDER = {m[1]: m for m in MISSIONS}


def resolve(names):
    """略称（ETH-1）かフォルダ名の並びを、フォルダ名の並びにする。'candidates' は候補の全部。"""
    out = []
    for n in names:
        if n.lower() == "candidates":
            out += [_BY_CODE[c][1] for c in CANDIDATES]
        elif n.upper() in _BY_CODE:
            out.append(_BY_CODE[n.upper()][1])
        elif n in _BY_FOLDER or (len(n) == 19 and n[4] == "-"):
            out.append(n)
        else:
            raise ValueError(f"unknown mission: {n}（略称かフォルダ名で指定する）")
    return list(dict.fromkeys(out))  # 順序を保って重複を除く


def code_of(folder):
    """フォルダ名 → 略称（表に無ければフォルダ名のまま）。"""
    m = _BY_FOLDER.get(folder)
    return m[0] if m else folder


def info(folder):
    """フォルダ名 → 表の行（無ければ None）。"""
    return _BY_FOLDER.get(folder)
