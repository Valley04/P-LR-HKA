# -*- coding: utf-8 -*-
"""Mockup diapositiva 17 - Resultado 7: rendimiento global.
Misma plantilla de dos carriles; aqui 3 columnas = 3 dominios,
arriba el ajuste (morado), abajo la medida (blanco)."""

from PIL import Image, ImageDraw, ImageFont
import os

OUT = r"C:\Users\lrojas\Desktop\P-LR-HKA\Defensa"

PURPLE  = "#3C0366"
ACCENT  = "#6100AC"
WHITE   = "#FFFFFF"
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

f_title = F(["impact.ttf", "arialbd.ttf"], 120)
f_tag   = F(["segoeuib.ttf", "arialbd.ttf"], 30)
f_dom   = F(["segoeuib.ttf", "arialbd.ttf"], 34)
f_cardT = F(["segoeuib.ttf", "arialbd.ttf"], 40)
f_cardS = F(["segoeui.ttf",  "arial.ttf"],   30)
f_si    = F(["segoeuii.ttf", "ariali.ttf", "segoeui.ttf"], 26)

W, H = 1920, 1080
img = Image.new("RGB", (W, H), WHITE)
d = ImageDraw.Draw(img)

def center_text(cx, y, s, font, fill):
    d.text((cx, y), s, font=font, fill=fill, anchor="mm")

# banda morada inferior
BAND_Y = 590
d.rectangle([0, BAND_Y, W, H], fill=PURPLE)

# titulo + linea
d.text((60, 35), "RESULTADO 7: RENDIMIENTO", font=f_title, fill=BLACK)
tw = d.textlength("RESULTADO 7: RENDIMIENTO", font=f_title)
d.line([(tw + 110, 95), (W - 60, 95)], fill=BLACK, width=4)

# etiquetas de carril
d.text((60, 200), "EL AJUSTE", font=f_tag, fill=ACCENT)
d.text((60, 660), "LA MEDIDA", font=f_tag, fill=WHITE)

# 3 columnas: dominio -> ajuste(arriba) -> medida(abajo)
CW, CH = 500, 210
COLS = [355, 960, 1565]
doms  = ["MEMORIA", "ENERGÍA", "BASE DE DATOS"]
top   = [("menuconfig", ["Wi-Fi sin aceleración", "por IRAM"]),
         ("subclocking", ["240 → 80 MHz"]),
         ("SQLite", ["índice por serial", "+ purga > 90 días"])]
bot   = [("IRAM 78,39 % → 52,26 %", ["−26,13 % · 34 256 B libres"]),
         ("potencia dinámica → ⅓", ["TLS en 45 ms (AES por hw)"]),
         ("búsqueda < 12 ms", ["con 10 000 registros"])]

for cx, dom, (tt, tl), (bt, bl) in zip(COLS, doms, top, bot):
    # nombre del dominio sobre el eje
    center_text(cx, 255, dom, f_dom, BLACK)
    # tarjeta superior (morada)
    d.rounded_rectangle([cx - CW/2, 310, cx + CW/2, 310 + CH], radius=16,
                        fill=PURPLE)
    center_text(cx, 310 + 60, tt, f_cardT, WHITE)
    for i, ln in enumerate(tl):
        center_text(cx, 310 + 115 + i * 38, ln, f_cardS, "#D9C8EE")
    # flecha cruzando el eje
    d.line([(cx, 525), (cx, 630)], fill=ACCENT, width=8)
    d.polygon([(cx, 645), (cx - 14, 622), (cx + 14, 622)], fill=ACCENT)
    d.ellipse([cx - 20, BAND_Y - 20, cx + 20, BAND_Y + 20], fill=ACCENT)
    # tarjeta inferior (blanca)
    d.rounded_rectangle([cx - CW/2, 690, cx + CW/2, 690 + CH], radius=16,
                        fill=WHITE)
    center_text(cx, 690 + 60, bt, F(["segoeuib.ttf", "arialbd.ttf"], 36),
                PURPLE)
    for i, ln in enumerate(bl):
        center_text(cx, 690 + 115 + i * 38, ln, f_cardS, GRAY)

img.save(os.path.join(OUT, "diapositiva_17_mockup.png"), dpi=(150, 150))
print("OK: diapositiva_17_mockup.png")
