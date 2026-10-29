# -*- coding: utf-8 -*-
"""Diapositiva 19 - Verificacion de cumplimiento de objetivos.
Matriz objetivo -> estado, paleta monocromo del deck."""

from PIL import Image, ImageDraw, ImageFont
import os

OUT = r"C:\Users\lrojas\Desktop\P-LR-HKA\Defensa"

PURPLE  = "#3C0366"
ACCENT  = "#6100AC"
LILAC   = "#EFE8F8"
LILAC2  = "#C9B6E4"
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

f_h   = F(["segoeuib.ttf", "arialbd.ttf"], 44)
f_b   = F(["segoeui.ttf",  "arial.ttf"],   30)
f_bb  = F(["segoeuib.ttf", "arialbd.ttf"], 30)
f_s   = F(["segoeui.ttf",  "arial.ttf"],   25)
f_si  = F(["segoeuii.ttf", "ariali.ttf", "segoeui.ttf"], 25)
f_badge = F(["segoeuib.ttf", "arialbd.ttf"], 22)

def center_text(d, cx, y, s, font, fill):
    d.text((cx, y), s, font=font, fill=fill, anchor="mm")

def badge(d, cx, y, txt, mode):
    """mode: 'ok' solido | 'partial' outline | 'extra' punteado-simulado."""
    w = d.textlength(txt, font=f_badge) + 40
    if mode == "ok":
        d.rounded_rectangle([cx - w/2, y - 24, cx + w/2, y + 24], radius=12,
                            fill=PURPLE)
        fg = "white"
    elif mode == "extra":
        d.rounded_rectangle([cx - w/2, y - 24, cx + w/2, y + 24], radius=12,
                            fill=ACCENT)
        fg = "white"
    else:
        d.rounded_rectangle([cx - w/2, y - 24, cx + w/2, y + 24], radius=12,
                            outline=ACCENT, width=4)
        fg = ACCENT
    center_text(d, cx, y, txt, f_badge, fg)

ROWS = [
    ("Arquitectura del sistema",            "ok",      "LOGRADO", None),
    ("Firmware del gateway",                "ok",      "LOGRADO", None),
    ("Comunicación MQTT v5.0 / TLS",        "ok",      "LOGRADO",
     "LWT + User Properties + certificados"),
    ("Servidor MQTT",                       "partial", "PARCIAL",
     "broker delegado a HiveMQ Cloud"),
    ("Integración extremo a extremo",       "ok",      "LOGRADO", None),
    ("Dashboard en tiempo real",            "ok",      "LOGRADO", None),
    ("Validación sobre banco HIL",          "ok",      "LOGRADO", None),
    ("Documentación técnica",               "ok",      "LOGRADO", None),
]

W, H = 1150, 1330
img = Image.new("RGB", (W, H), "white")
d = ImageDraw.Draw(img)
d.rounded_rectangle([4, 4, W - 4, H - 4], radius=26, fill="white",
                    outline=LILAC2, width=6)

center_text(d, W / 2, 65, "Matriz de cumplimiento", f_h, PURPLE)
center_text(d, W / 2, 115, "objetivo → estado → entregable", f_si, GRAY)

y = 160
for nombre, modo, tag, nota in ROWS:
    rh = 118 if nota else 92
    fill = LILAC if modo != "extra" else "white"
    d.rounded_rectangle([40, y, W - 40, y + rh - 16], radius=14, fill=fill,
                        outline="#DCD3EA", width=3)
    d.text((70, y + 18), nombre, font=f_bb, fill=BLACK)
    if nota:
        d.text((70, y + 58), nota, font=f_s, fill=GRAY)
    badge(d, W - 200, y + (rh - 16) / 2, tag, modo)
    y += rh

# fila extra: objetivo no previsto
d.rounded_rectangle([40, y, W - 40, y + 118], radius=14, fill="#F6F1FB",
                    outline=ACCENT, width=5)
d.text((70, y + 18), "Módulo OTA de doble objetivo", font=f_bb, fill=PURPLE)
d.text((70, y + 58), "no previsto en el plan — ampliación de alcance",
       font=f_s, fill=ACCENT)
badge(d, W - 200, y + 51, "NO PREVISTO", "extra")

img.save(os.path.join(OUT, "diapositiva_19_matriz.png"), dpi=(200, 200))
print("OK: diapositiva_19_matriz.png")
