# -*- coding: utf-8 -*-
"""Mockup de la diapositiva 9 respetando el feel & look del deck:
morado #3C0366, acento #6100AC, blanco, negro; dos carriles = dos decisiones."""

from PIL import Image, ImageDraw, ImageFont
import os

OUT = r"C:\Users\lrojas\Desktop\P-LR-HKA\Defensa"

PURPLE   = "#3C0366"   # morado profundo del deck
ACCENT   = "#6100AC"   # violeta de los nodos
WHITE    = "#FFFFFF"
BLACK    = "#111111"
GRAY     = "#666666"

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
f_cardT = F(["segoeuib.ttf", "arialbd.ttf"], 44)
f_cardS = F(["segoeui.ttf",  "arial.ttf"],   32)
f_badge = F(["segoeuib.ttf", "arialbd.ttf"], 24)

W, H = 1920, 1080
img = Image.new("RGB", (W, H), WHITE)
d = ImageDraw.Draw(img)

def center_text(cx, y, s, font, fill):
    d.text((cx, y), s, font=font, fill=fill, anchor="mm")

# ---- banda morada inferior ----
BAND_Y = 590
d.rectangle([0, BAND_Y, W, H], fill=PURPLE)

# ---- titulo + linea ----
d.text((60, 35), "JUSTIFICACIÓN TECNOLÓGICA", font=f_title, fill=BLACK)
tw = d.textlength("JUSTIFICACIÓN TECNOLÓGICA", font=f_title)
d.line([(tw + 110, 95), (W - 60, 95)], fill=BLACK, width=4)

# ---- nodos sobre el eje ----
node_x = [270, 545, 820, 1095, 1370, 1645]
for nx in node_x:
    d.ellipse([nx - 42, BAND_Y - 42, nx + 42, BAND_Y + 42], fill=ACCENT)

# ---- etiquetas de carril ----
def lane_tag(x, y, text, bg, fg):
    w = d.textlength(text, font=f_tag) + 50
    d.rounded_rectangle([x, y, x + w, y + 56], radius=28, fill=bg)
    d.text((x + 25, y + 13), text, font=f_tag, fill=fg)
    return w

lane_tag(60, 215, "DECISIÓN 1 · FRAMEWORK", ACCENT, WHITE)
lane_tag(60, 622, "DECISIÓN 2 · PROTOCOLO", WHITE, PURPLE)

# ---- tarjetas ----
CW, CH = 390, 210
def card(cx, cy, titulo, lineas, dark_card, badge=None):
    x1, y1 = cx - CW / 2, cy - CH / 2
    fill = PURPLE if dark_card else WHITE
    fg   = WHITE if dark_card else BLACK
    sub  = "#D9C8EE" if dark_card else GRAY
    d.rounded_rectangle([x1, y1, x1 + CW, y1 + CH], radius=14, fill=fill,
                        outline=None if dark_card else PURPLE, width=4)
    center_text(cx, y1 + 52, titulo, f_cardT, fg)
    for i, ln in enumerate(lineas):
        center_text(cx, y1 + 112 + i * 40, ln, f_cardS, sub)
    if badge:
        kind, b_txt, b_col = badge
        glyph_w = 26
        bw = d.textlength(b_txt, font=f_badge) + 30 + glyph_w
        bx = x1 + CW - bw - 14
        d.rounded_rectangle([bx, y1 + CH - 48, bx + bw, y1 + CH - 14],
                            radius=8, fill=b_col)
        gc = WHITE if dark_card else PURPLE
        gx, gy = bx + 18, y1 + CH - 31
        if kind == "x":
            d.line([(gx - 7, gy - 8), (gx + 7, gy + 8)], fill=gc, width=4)
            d.line([(gx - 7, gy + 8), (gx + 7, gy - 8)], fill=gc, width=4)
        else:
            d.line([(gx - 8, gy), (gx - 2, gy + 7)], fill=gc, width=4)
            d.line([(gx - 2, gy + 7), (gx + 9, gy - 7)], fill=gc, width=4)
        d.text((bx + 15 + glyph_w, y1 + CH - 43), b_txt, font=f_badge, fill=gc)

def harrow(x1, x2, y, color):
    d.line([(x1, y), (x2, y)], fill=color, width=7)
    d.polygon([(x2, y), (x2 - 20, y - 12), (x2 - 20, y + 12)], fill=color)

# carril superior (morado): Framework
yt = 400
card(445,  yt, "ARDUINO",    ["bucle mono-hilo"], True,  ("x", "DESCARTADA", "#26023F"))
card(1055, yt, "EL RIESGO",  ["delay() bloqueante", "arriesga la ventana fiscal"], True)
card(1635, yt, "ESP-IDF v5.5.2", ["FreeRTOS determinista"], True, ("v", "ELEGIDA", ACCENT))
harrow(640,  860, yt, ACCENT)
harrow(1250, 1440, yt, ACCENT)

# carril inferior (blanco): Protocolo
yb = 815
card(268,  yb, "HTTP REST",  ["cabecera pesada"], False, ("x", "DESCARTADA", "#D9C8EE"))
card(785,  yb, "EL PROBLEMA", ["mensaje cada 5 s", "sobrecarga enorme"], False)
card(1360, yb, "MQTT v5.0",  ["pub-sub ligero"], False, ("v", "ELEGIDA", ACCENT))
harrow(463, 590, yb, WHITE)
harrow(980, 1165, yb, WHITE)

img.save(os.path.join(OUT, "diapositiva_09_mockup.png"), dpi=(150, 150))
print("OK: diapositiva_09_mockup.png")
