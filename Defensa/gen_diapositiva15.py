# -*- coding: utf-8 -*-
"""Diapositiva 15 - Resultado 5: latencia end-to-end.
Barras proporcionales por etapa, monocromo morado (deck)."""

from PIL import Image, ImageDraw, ImageFont
import os

OUT = r"C:\Users\lrojas\Desktop\P-LR-HKA\Defensa"

PURPLE  = "#3C0366"
ACCENT  = "#6100AC"
LILAC   = "#C9B6E4"
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

f_h   = F(["segoeuib.ttf", "arialbd.ttf"], 46)
f_si  = F(["segoeuii.ttf", "ariali.ttf", "segoeui.ttf"], 28)
f_lab = F(["segoeuib.ttf", "arialbd.ttf"], 34)
f_b   = F(["segoeui.ttf",  "arial.ttf"],   32)
f_big = F(["impact.ttf",   "segoeuib.ttf", "arialbd.ttf"], 110)

def center_text(d, cx, y, s, font, fill):
    d.text((cx, y), s, font=font, fill=fill, anchor="mm")

# etapa, ms, destacada?
STAGES = [
    ("Sondeo síncrono S1",            118, False),
    ("Empaquetado JSON",              2,   False),
    ("Handshake TLS 1.2",             45,  False),
    ("Publicación + tránsito broker", 85,  True),
    ("Renderizado DOM",               18,  False),
]
TOTAL = 268

W, H = 1150, 1500
img = Image.new("RGB", (W, H), "white")
d = ImageDraw.Draw(img)

center_text(d, W / 2, 60,  "Latencia de extremo a extremo", f_h, PURPLE)
center_text(d, W / 2, 112, "500 iteraciones continuas · del sondeo al navegador",
            f_si, GRAY)

BAR_X = 60
scale = 780 / TOTAL          # px por ms, relativo al total
BH = 64

y = 210
for name, ms, hot in STAGES:
    d.text((BAR_X, y), name, font=f_lab, fill=BLACK)
    y += 58
    w = max(ms * scale, 56)
    col = ACCENT if hot else PURPLE
    d.rounded_rectangle([BAR_X, y, BAR_X + w, y + BH], radius=10, fill=col)
    d.text((BAR_X + w + 18, y + 12), f"{ms} ms", font=f_b,
           fill=ACCENT if hot else GRAY)
    if hot:
        d.text((BAR_X + 12, y + BH + 6),
               "etapa más variable — 86,8 % de la dispersión viene del enlace ISP",
               font=f_si, fill=ACCENT)
        y += 46
    y += BH + 34

# banda total
d.rounded_rectangle([60, y + 10, W - 60, y + 220], radius=24, fill=PURPLE)
center_text(d, W / 2, y + 80, "268 ms", f_big, "white")
center_text(d, W / 2, y + 148, "± 15,3 ms  ·  5,36 % de la ventana de sondeo (5 s)",
            f_si, "#D9C8EE")
center_text(d, W / 2, y + 188, "tiempo real estricto — sin acumular retrasos",
            f_si, "#D9C8EE")

img.save(os.path.join(OUT, "diapositiva_15_latencia.png"), dpi=(200, 200))
print("OK: diapositiva_15_latencia.png")
