# -*- coding: utf-8 -*-
"""Genera la tabla comparativa para la diapositiva 9:
Justificacion tecnologica (Arduino vs ESP-IDF · HTTP vs MQTT)."""

from PIL import Image, ImageDraw, ImageFont
import os

OUT = r"C:\Users\lrojas\Desktop\P-LR-HKA\Defensa"

PURPLE   = "#4E2A84"
PURPLE_D = "#3A1F63"
LILAC    = "#EFE8F8"
RED      = "#B03030"
RED_F    = "#FBEAEA"
GRAY     = "#555555"
GREEN    = "#2E7D4F"
GREEN_F  = "#E4F2E9"
AMBER    = "#C96A00"

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

f_col  = F(["segoeuib.ttf", "arialbd.ttf"], 34)
f_big  = F(["segoeuib.ttf", "arialbd.ttf"], 46)
f_b    = F(["segoeui.ttf",  "arial.ttf"],   33)
f_s    = F(["segoeui.ttf",  "arial.ttf"],   29)
f_si   = F(["segoeuii.ttf", "ariali.ttf", "segoeui.ttf"], 28)
f_mono = F(["consolab.ttf", "courbd.ttf"],  32)

def center_text(d, cx, y, s, font, fill):
    d.text((cx, y), s, font=font, fill=fill, anchor="mm")

W, H = 2050, 1180
img = Image.new("RGB", (W, H), "white")
d = ImageDraw.Draw(img)
d.rounded_rectangle([4, 4, W - 4, H - 4], radius=30, fill="white",
                    outline=LILAC, width=6)

# ---- encabezado de columnas ----
cx_dec, cx_out, cx_in = 240, 840, 1640
hy = 90
center_text(d, cx_out, hy, "OPCIÓN DESCARTADA", f_col, RED)
center_text(d, cx_in,  hy, "OPCIÓN ELEGIDA",   f_col, GREEN)
d.line([(70, 150), (W - 70, 150)], fill=LILAC, width=5)

rows = [
    # (decision, descartada, razon1, razon2, elegida, aporte1, aporte2)
    ("Framework",
     "Arduino",
     "bucle mono-hilo",
     "delay() bloqueante pone en riesgo\nla ventana fiscal del equipo",
     "ESP-IDF v5.5.2",
     "memoria explícita en C",
     "tareas deterministas con FreeRTOS"),
    ("Protocolo",
     "HTTP REST",
     "cabecera pesada",
     "sobrecarga enorme para un mensaje\nque cambia cada 5 s",
     "MQTT v5.0",
     "metadatos en la cabecera",
     "modelo publicación–suscripción ligero"),
]

y = 210
rh = 420          # alto de fila
for i, (dec, out_, r1, r2, in_, a1, a2) in enumerate(rows):
    yc = y + rh // 2
    # etiqueta de la decision
    d.rounded_rectangle([90, y + 120, 390, y + 300], radius=18,
                        fill=LILAC, outline=PURPLE, width=5)
    center_text(d, 240, yc, dec.upper(), f_col, PURPLE_D)
    # tarjeta descartada
    d.rounded_rectangle([450, y + 40, 1230, y + rh - 40], radius=20,
                        fill=RED_F, outline=RED, width=5)
    center_text(d, cx_out, y + 110, out_, f_big, RED)
    center_text(d, cx_out, y + 185, r1, f_b, "#5a1f1f")
    for j, line in enumerate(r2.split("\n")):
        center_text(d, cx_out, y + 245 + j * 42, line, f_si, "#7a4040")
    # tachadura simbolica (X)
    d.line([(470, y + 60), (510, y + 100)], fill=RED, width=7)
    d.line([(510, y + 60), (470, y + 100)], fill=RED, width=7)
    # flecha de decision
    d.line([(1250, yc), (1330, yc)], fill=AMBER, width=9)
    d.polygon([(1330, yc), (1304, yc - 16), (1304, yc + 16)], fill=AMBER)
    # tarjeta elegida
    d.rounded_rectangle([1350, y + 40, 1980, y + rh - 40], radius=20,
                        fill=GREEN_F, outline=GREEN, width=5)
    center_text(d, cx_in, y + 110, in_, f_big, GREEN)
    center_text(d, cx_in, y + 195, a1, f_b, "#2c513c")
    center_text(d, cx_in, y + 250, a2, f_b, "#2c513c")
    # check simbolico
    d.line([(1375, y + 80), (1392, y + 97)], fill=GREEN, width=7)
    d.line([(1392, y + 97), (1422, y + 60)], fill=GREEN, width=7)
    # separador
    if i == 0:
        d.line([(70, y + rh + 20), (W - 70, y + rh + 20)], fill=LILAC, width=5)
    y += rh + 40

# nota al pie: la medicion del ahorro se muestra en resultados
center_text(d, W / 2, H - 70,
            "el ahorro cuantitativo HTTP→MQTT se mide en el bloque de resultados",
            f_si, GRAY)

img.save(os.path.join(OUT, "diapositiva_09_tabla.png"), dpi=(200, 200))
print("OK: diapositiva_09_tabla.png")
