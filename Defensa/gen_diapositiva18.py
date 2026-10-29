# -*- coding: utf-8 -*-
"""Diapositiva 18 - Resultado 8: comparativa estado previo vs actual.
Tabla antes/despues en paleta del deck (lila grisado vs morado)."""

from PIL import Image, ImageDraw, ImageFont
import os

OUT = r"C:\Users\lrojas\Desktop\P-LR-HKA\Defensa"

PURPLE  = "#3C0366"
ACCENT  = "#6100AC"
LILAC   = "#EFE8F8"
LILAC2  = "#C9B6E4"
GRAY_F  = "#F0EFF2"
BLACK   = "#111111"
GRAY    = "#666666"

FD = r"C:\Windows\Fonts"
def F(names, size):
    if isinstance(names, str):
        names = [names]
    for n in names:
        try:
            return ImageFont.truetype(os.path.join(FD, n), size)
        except OSError:
            continue
    return ImageFont.load_default()

f_h   = F(["segoeuib.ttf", "arialbd.ttf"], 36)
f_b   = F(["segoeui.ttf",  "arial.ttf"],   28)
f_bb  = F(["segoeuib.ttf", "arialbd.ttf"], 28)

def center_text(d, cx, y, s, font, fill):
    d.text((cx, y), s, font=font, fill=fill, anchor="mm")

ROWS = [
    ("PC conectada al\npuerto serial",
     "monitoreo remoto\ny centralizado"),
    ("un equipo a la vez,\nsin visión global",
     "múltiples equipos\nen simultáneo"),
    ("diagnóstico reactivo,\nposterior al fallo",
     "detección temprana\ncon alertas"),
    ("sin historial\nde auditoría",
     "bitácora de 90 días\npor serial"),
    ("firmware solo\ncon visita en sitio",
     "actualización desde\nla nube (gateway + equipo)"),
]

W, H = 1180, 1460
img = Image.new("RGB", (W, H), "white")
d = ImageDraw.Draw(img)
d.rounded_rectangle([4, 4, W - 4, H - 4], radius=26, fill="white",
                    outline=LILAC2, width=6)

# encabezados de columna
LX1, LX2, RX1, RX2 = 40, 575, 605, 1140
d.rounded_rectangle([LX1, 50, LX2, 130], radius=14, fill="#B9AEC9")
center_text(d, (LX1 + LX2) / 2, 90, "ESTADO PREVIO", f_h, "white")
d.rounded_rectangle([RX1, 50, RX2, 130], radius=14, fill=PURPLE)
center_text(d, (RX1 + RX2) / 2, 90, "ESTADO ACTUAL", f_h, "white")

RH = 235
y = 160
for antes, despues in ROWS:
    # celda previo (apagada)
    d.rounded_rectangle([LX1, y, LX2, y + RH - 25], radius=14,
                        fill=GRAY_F, outline="#C9C4D4", width=4)
    for i, ln in enumerate(antes.split("\n")):
        center_text(d, (LX1 + LX2) / 2,
                    y + (RH - 25) / 2 - 20 + i * 42, ln, f_b, "#555555")
    # celda actual (viva)
    d.rounded_rectangle([RX1, y, RX2, y + RH - 25], radius=14,
                        fill=PURPLE)
    for i, ln in enumerate(despues.split("\n")):
        center_text(d, (RX1 + RX2) / 2,
                    y + (RH - 25) / 2 - 20 + i * 42, ln, f_bb, "white")
    # flecha antes -> despues
    mx = (LX2 + RX1) / 2
    my = y + (RH - 25) / 2
    d.line([(LX2 + 4, my), (RX1 - 4, my)], fill=ACCENT, width=5)
    d.polygon([(RX1 - 4, my), (RX1 - 20, my - 10), (RX1 - 20, my + 10)],
              fill=ACCENT)
    y += RH

img.save(os.path.join(OUT, "diapositiva_18_antes_despues.png"),
         dpi=(200, 200))
print("OK: diapositiva_18_antes_despues.png")
