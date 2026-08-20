#!/usr/bin/env python3
"""Generate docs/circuit/wiring-map.excalidraw (MCP-verified layout + bindings).

Every arrow carries startBinding/endBinding (elementId + fixedPoint) so the
wires stay glued to their components when dragging in the excalidraw.com
editor. Labeled shapes become bound text elements; cameraUpdate/delete
pseudo-elements are stripped.
"""
import json, random, time, uuid

random.seed(7)
NOW = int(time.time() * 1000)


def base(t, x, y, **kw):
    e = {
        "type": t, "version": 1, "versionNonce": random.randint(0, 10**6),
        "isDeleted": False, "id": "", "fillStyle": "solid",
        "strokeWidth": 2, "strokeStyle": "solid", "roughness": 1,
        "opacity": 100, "angle": 0, "x": x, "y": y,
        "strokeColor": "#1e1e1e", "backgroundColor": "transparent",
        "width": 0, "height": 0, "seed": random.randint(0, 10**6),
        "groupIds": [], "frameId": None, "roundness": {"type": 3},
        "boundElements": [], "updated": NOW, "link": None, "locked": False,
    }
    e.update(kw)
    return e


def est_text(txt, fs):
    w = max(len(l) for l in txt.split("\n")) * fs * 0.6
    h = (txt.count("\n") + 1) * fs * 1.25
    return w, h


# shape: (name, type, x, y, w, h, fill, stroke, label, fontsize)
SHAPES = [
    ("bat", "rectangle", 50, 100, 170, 80, "#ffc9c9", "#ef4444", "2S 电池 7.4V", 16),
    ("buck", "rectangle", 270, 110, 160, 80, "#fff3bf", "#f59e0b", "MP1584EN 固定5V", 16),
    ("c1", "rectangle", 50, 190, 110, 26, "#ffc9c9", "#ef4444", "1000uF/16V", 14),
    ("tb", "rectangle", 540, 90, 230, 130, "#a5d8ff", "#4a9eed",
     "TB6612 驱动\nAIN1:G9 AIN2:G10\nPWMA:G1 STBY:G13", 14),
    ("esp", "rectangle", 540, 280, 230, 150, "#c3fae8", "#22c55e",
     "ESP32-S3 HW678\n5VIN 3V3 GND\nOTG口<-接收器(hub) VBUS=5V焊盘桥\nGPIO9/10/1/13/14 -> 驱动/舵机", 14),
    ("sw", "rectangle", 60, 300, 140, 50, "#ffffff", "#1e1e1e", "开关 3A", 16),
    ("srv", "rectangle", 280, 330, 180, 80, "#ffd8a8", "#f59e0b",
     "SG90 舵机\n+5V / 信号G14", 14),
    ("mot", "rectangle", 600, 480, 170, 70, "#ffd8a8", "#f59e0b", "N30 马达", 16),
    ("gnds", "ellipse", 140, 470, 80, 80, "#d3f9d8", "#22c55e", "共地点", 14),
    ("c2", "rectangle", 260, 200, 110, 26, "#fff3bf", "#f59e0b", "1000uF 近OUT", 14),
    ("c3", "rectangle", 540, 45, 110, 26, "#a5d8ff", "#4a9eed", "1000uF 近VM", 14),
    ("c4", "rectangle", 250, 440, 100, 24, "#ffd8a8", "#f59e0b", "470uF 舵机", 14),
    ("c5", "rectangle", 600, 440, 110, 26, "#ffd8a8", "#f59e0b", "100nF 马达", 14),
]

# arrow: (name, x, y, points, color, width, endhead, start_bind, end_bind, label)
ARROWS = [
    ("p2", 220, 140, [[0, 0], [50, 10]], "#ef4444", 2, "arrow",
     ("bat", [1, 0.5]), ("buck", [0, 0.5]), "BAT+ -> IN+"),
    ("lc1", 105, 190, [[0, 0], [0, -10]], "#ef4444", 1.5, None,
     ("c1", [0.5, 0]), ("bat", [0.5, 1]), None),
    ("p1", 220, 110, [[0, 0], [320, 0]], "#ef4444", 2, "arrow",
     ("bat", [1, 0.25]), ("tb", [0, 0.25]), None),
    ("p3", 430, 150, [[0, 0], [0, 160], [110, 160]], "#ef4444", 2, "arrow",
     ("buck", [1, 0.5]), ("esp", [0, 0.25]), "5V 轨"),
    ("p4", 380, 190, [[0, 0], [0, 140]], "#ef4444", 2, "arrow",
     ("buck", [0.5, 1]), ("srv", [0.5, 0]), "5V -> 舵机+"),
    ("g1", 135, 180, [[0, 0], [0, 120]], "#22c55e", 2, "arrow",
     ("bat", [0.5, 1]), ("sw", [0.5, 0]), "BAT-"),
    ("g2", 200, 325, [[0, 0], [-60, 185]], "#22c55e", 2, "arrow",
     ("sw", [1, 0.5]), ("gnds", [0, 0.5]), "开关后"),
    ("g3", 290, 190, [[0, 0], [-40, 0], [-40, 280], [-70, 280]], "#22c55e", 2, "arrow",
     ("buck", [0.15, 1]), ("gnds", [1, 0.05]), "IN-/OUT-"),
    ("g4", 600, 220, [[0, 0], [0, 290], [-380, 290]], "#22c55e", 2, "arrow",
     ("tb", [0.35, 1]), ("gnds", [1, 0.5]), "GND"),
    ("g5", 370, 410, [[0, 0], [0, 60], [-190, 60]], "#22c55e", 2, "arrow",
     ("srv", [0.5, 1]), ("gnds", [0.5, 0]), "-"),
    ("g6", 560, 430, [[0, 0], [0, 80], [-340, 80]], "#22c55e", 2, "arrow",
     ("esp", [0.15, 1]), ("gnds", [1, 0.5]), "GND"),
    ("s1", 600, 280, [[0, 0], [0, -60]], "#4a9eed", 1.5, "arrow",
     ("esp", [0.35, 0]), ("tb", [0.35, 1]), "G9/10/1/13 -> AIN1/2/PWMA/STBY"),
    ("s2", 700, 280, [[0, 0], [0, -60]], "#4a9eed", 1.5, "arrow",
     ("esp", [0.85, 0]), ("tb", [0.85, 1]), "3V3 -> VCC"),
    ("s3", 560, 430, [[0, 0], [-100, -60]], "#4a9eed", 1.5, "arrow",
     ("esp", [0.05, 1]), ("srv", [1, 0.5]), "G14 -> 信号"),
    ("m1", 770, 130, [[0, 0], [0, 385]], "#f59e0b", 2, "arrow",
     ("tb", [1, 0.35]), ("mot", [1, 0.5]), "AO1/AO2 -> I1/I2"),
    ("lc2", 370, 213, [[0, 0], [60, -63]], "#f59e0b", 1.5, None,
     ("c2", [1, 0.5]), ("buck", [1, 0.5]), None),
    ("lc3", 595, 71, [[0, 0], [0, 19]], "#4a9eed", 1.5, None,
     ("c3", [0.5, 1]), ("tb", [0.35, 0]), None),
    ("lc4", 350, 440, [[0, 0], [20, -30]], "#f59e0b", 1.5, None,
     ("c4", [0.95, 0]), ("srv", [0.5, 1]), None),
    ("lc5", 655, 466, [[0, 0], [30, 14]], "#f59e0b", 1.5, None,
     ("c5", [0.5, 1]), ("mot", [0.5, 0]), None),
]

TEXTS = [
    ("vm", 350, 95, "VM 7.4V 直供", 14, "#ef4444"),
    ("capnote", 30, 560, "全部电容负脚 -> 共地点", 14, "#15803d"),
]


def binding(name, fp):
    return {"elementId": name, "fixedPoint": fp, "gap": 0, "focus": 0}


E = []
# shapes first (background), so bound arrows render above
for name, t, x, y, w, h, fill, stroke, label, fs in SHAPES:
    shape = base(t, x, y, id=name, width=w, height=h,
                 strokeColor=stroke, backgroundColor=fill, strokeWidth=2)
    tw, th = est_text(label, fs)
    tid = f"{name}_lbl"
    shape["boundElements"] = [{"type": "text", "id": tid}]
    tl = base("text", x + (w - tw) / 2, y + (h - th) / 2, id=tid,
              text=label, fontSize=fs, fontFamily=1,
              textAlign="center", verticalAlign="middle",
              containerId=name, width=tw, height=th,
              strokeColor="#1e1e1e", baseline=fs * 1.25,
              lineHeight=1.25, originalText=label)
    E.append(shape)
    E.append(tl)

# arrows (with bindings)
for name, x, y, pts, color, width, endhead, sb, eb, label in ARROWS:
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    minx, miny = min(xs), min(ys)
    w, h = max(xs) - minx, max(ys) - miny
    npts = [[p[0] - minx, p[1] - miny] for p in pts]
    ae = base("arrow", x + minx, y + miny, id=name, width=w, height=h,
              strokeColor=color, strokeWidth=width,
              points=npts, lastCommittedPoint=npts[-1],
              endArrowhead=endhead if endhead else None,
              startArrowhead=None,
              startBinding=binding(*sb), endBinding=binding(*eb))
    if label:
        fs = 14
        tw, th = est_text(label, fs)
        tid = f"{name}_lbl"
        ae["boundElements"] = [{"type": "text", "id": tid}]
        tl = base("text", (x + minx) + (w - tw) / 2, (y + miny) + (h - th) / 2 - 8,
                  id=tid, text=label, fontSize=fs, fontFamily=1,
                  textAlign="center", verticalAlign="middle",
                  containerId=name, width=tw, height=th,
                  strokeColor="#1e1e1e", baseline=fs * 1.25,
                  lineHeight=1.25, originalText=label)
        E.append(ae)
        E.append(tl)
    else:
        E.append(ae)

# standalone texts
for name, x, y, txt, fs, color in TEXTS:
    tw, th = est_text(txt, fs)
    E.append(base("text", x, y, id=name, text=txt, fontSize=fs, fontFamily=1,
                  textAlign="left", verticalAlign="top", strokeColor=color,
                  width=tw, height=th, baseline=fs * 1.25,
                  lineHeight=1.25, originalText=txt))

doc = {
    "type": "excalidraw",
    "version": 2,
    "source": "https://excalidraw.com",
    "elements": E,
    "appState": {"gridSize": None, "viewBackgroundColor": "#fafafa"},
    "files": {},
}

out = "docs/circuit/wiring-map.excalidraw"
with open(out, "w", encoding="utf-8") as f:
    json.dump(doc, f, ensure_ascii=False, indent=1)

# validation
ids = [e["id"] for e in E if e["type"] != "text"]
bound = [e.get("containerId") for e in E if e.get("containerId")]
arrow_b = [e["id"] for e in E if e["type"] == "arrow" and e.get("startBinding")]
kinds = {}
for e in E:
    kinds[e["type"]] = kinds.get(e["type"], 0) + 1
print(f"written: {out}")
print("elements:", kinds)
print("arrows with bindings:", len(arrow_b), "/", sum(1 for e in E if e["type"] == "arrow"))
print("bound texts:", len(bound), "| ids unique:", len(set(ids)) == len(ids))
