#!/usr/bin/env python3
"""Generate docs/circuit/wiring-map.drawio from the MCP-verified layout.

draw.io edges reference source/target cells (graph edges), so wires stay
glued to components when dragging — unlike Excalidraw coordinate lines.
Open in https://app.diagrams.net -> File -> Open From -> Device.
"""
import xml.etree.ElementTree as ET
from xml.dom import minidom

# (name, x, y, w, h, fill, stroke, label)
SHAPES = [
    ("bat", 50, 100, 170, 80, "#ffc9c9", "#ef4444", "2S 电池 7.4V"),
    ("buck", 270, 110, 160, 80, "#fff3bf", "#f59e0b", "MP1584EN 固定5V"),
    ("c1", 50, 190, 110, 26, "#ffc9c9", "#ef4444", "1000uF/16V"),
    ("tb", 540, 90, 230, 130, "#a5d8ff", "#4a9eed",
     "TB6612 驱动<br>AIN1:G9 AIN2:G10<br>PWMA:G1 STBY:G13"),
    ("esp", 540, 280, 230, 150, "#c3fae8", "#22c55e",
     "ESP32-S3 HW678<br>5VIN 3V3 GND<br>OTG口&lt;-接收器(hub) VBUS=5V焊盘桥<br>GPIO9/10/1/13/14 -&gt; 驱动/舵机"),
    ("sw", 60, 300, 140, 50, "#ffffff", "#1e1e1e", "开关 3A"),
    ("srv", 280, 330, 180, 80, "#ffd8a8", "#f59e0b", "SG90 舵机<br>+5V / 信号G14"),
    ("mot", 600, 480, 170, 70, "#ffd8a8", "#f59e0b", "N30 马达"),
    ("gnds", 140, 470, 80, 80, "#d3f9d8", "#22c55e", "共地点"),
    ("c2", 260, 200, 110, 26, "#fff3bf", "#f59e0b", "1000uF 近OUT"),
    ("c3", 540, 45, 110, 26, "#a5d8ff", "#4a9eed", "1000uF 近VM"),
    ("c4", 250, 440, 100, 24, "#ffd8a8", "#f59e0b", "470uF 舵机"),
    ("c5", 600, 440, 110, 26, "#ffd8a8", "#f59e0b", "100nF 马达"),
]

# (name, color, width, source, src_pt, target, tgt_pt, label)
# fixedPoint: [fx, fy] relative to shape (0..1); right=[1,0.5] etc.
ARROWS = [
    ("p2", "#ef4444", 2, "bat", [1, 0.5], "buck", [0, 0.5], "BAT+ -&gt; IN+"),
    ("lc1", "#ef4444", 1.5, "c1", [0.5, 0], "bat", [0.5, 1], None),
    ("p1", "#ef4444", 2, "bat", [1, 0.25], "tb", [0, 0.25], "VM 7.4V 直供"),
    ("p3", "#ef4444", 2, "buck", [1, 0.5], "esp", [0, 0.25], "5V 轨"),
    ("p4", "#ef4444", 2, "buck", [0.5, 1], "srv", [0.5, 0], "5V -&gt; 舵机+"),
    ("g1", "#22c55e", 2, "bat", [0.5, 1], "sw", [0.5, 0], "BAT-"),
    ("g2", "#22c55e", 2, "sw", [1, 0.5], "gnds", [0, 0.5], "开关后"),
    ("g3", "#22c55e", 2, "buck", [0.15, 1], "gnds", [1, 0.05], "IN-/OUT-"),
    ("g4", "#22c55e", 2, "tb", [0.35, 1], "gnds", [1, 0.5], "GND"),
    ("g5", "#22c55e", 2, "srv", [0.5, 1], "gnds", [0.5, 0], "-"),
    ("g6", "#22c55e", 2, "esp", [0.15, 1], "gnds", [1, 0.5], "GND"),
    ("s1", "#4a9eed", 1.5, "esp", [0.35, 0], "tb", [0.35, 1],
     "G9/10/1/13 -&gt; AIN1/2/PWMA/STBY"),
    ("s2", "#4a9eed", 1.5, "esp", [0.85, 0], "tb", [0.85, 1], "3V3 -&gt; VCC"),
    ("s3", "#4a9eed", 1.5, "esp", [0.05, 1], "srv", [1, 0.5], "G14 -&gt; 信号"),
    ("m1", "#f59e0b", 2, "tb", [1, 0.35], "mot", [1, 0.5], "AO1/AO2 -&gt; I1/I2"),
    ("lc2", "#f59e0b", 1.5, "c2", [1, 0.5], "buck", [1, 0.5], None),
    ("lc3", "#4a9eed", 1.5, "c3", [0.5, 1], "tb", [0.35, 0], None),
    ("lc4", "#f59e0b", 1.5, "c4", [0.95, 0], "srv", [0.5, 1], None),
    ("lc5", "#f59e0b", 1.5, "c5", [0.5, 1], "mot", [0.5, 0], None),
]

ET.register_namespace("", "http://www.mxgraph.io/mxGraph")
root = ET.Element("mxfile", host="app.diagrams.net")
diagram = ET.SubElement(root, "diagram", name="jocar-wiring", id="jocar1")
model = ET.SubElement(diagram, "mxGraphModel", dx="1000", dy="700", grid="1",
                      gridSize="10", guides="1", tooltips="1", connect="1",
                      arrows="1", fold="1", page="1", pageScale="1",
                      pageWidth="850", pageHeight="1100", math="0", shadow="0")
cells = ET.SubElement(model, "root")
ET.SubElement(cells, "mxCell", id="0")
ET.SubElement(cells, "mxCell", id="1", parent="0")

for name, x, y, w, h, fill, stroke, label in SHAPES:
    style = (f"rounded=1;whiteSpace=wrap;html=1;fillColor={fill};"
             f"strokeColor={stroke};fontSize=14;"
             f"verticalAlign=middle;align=center;")
    cell = ET.SubElement(cells, "mxCell", id=name, value=label,
                         style=style, vertex="1", parent="1")
    geo = ET.SubElement(cell, "mxGeometry", x=str(x), y=str(y),
                        width=str(w), height=str(h), as_="geometry")

for name, color, width, src, sp, tgt, tp, label in ARROWS:
    style = (f"edgeStyle=orthogonalEdgeStyle;rounded=0;html=1;"
             f"strokeColor={color};strokeWidth={width};endArrow=block;"
             f"exitX={sp[0]};exitY={sp[1]};"
             f"entryX={tp[0]};entryY={tp[1]};exitDx=0;exitDy=0;entryDx=0;entryDy=0;")
    edge = ET.SubElement(cells, "mxCell", id=name, style=style,
                         edge="1", parent="1", source=src, target=tgt)
    geo = ET.SubElement(edge, "mxGeometry", relative="1", as_="geometry")
    if label:
        lbl = ET.SubElement(edge, "mxCell", id=f"{name}_lbl", value=label,
                            style="edgeLabel;html=1;align=center;",
                            vertex="1", connectable="0", parent=name)
        lgeo = ET.SubElement(lbl, "mxGeometry", x="0.5", y="0.5",
                             relative="1", as_="geometry")
        ET.SubElement(lgeo, "mxPoint", x="0", y="-8", as_="offset")

# note cell
note = ET.SubElement(cells, "mxCell", id="capnote",
                     value="全部电容负脚 -&gt; 共地点",
                     style="text;html=1;fontColor=#15803d;fontSize=14;",
                     vertex="1", parent="1")
ET.SubElement(note, "mxGeometry", x="30", y="580", width="220", height="24",
              as_="geometry")

xml_str = ET.tostring(root, encoding="unicode")
pretty = minidom.parseString(xml_str).toprettyxml(indent="  ")

out = "docs/circuit/wiring-map.drawio"
with open(out, "w", encoding="utf-8") as f:
    f.write(pretty)
print(f"written: {out} ({len(SHAPES)} shapes, {len(ARROWS)} edges)")
