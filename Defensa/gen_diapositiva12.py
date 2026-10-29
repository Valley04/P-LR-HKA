# -*- coding: utf-8 -*-
"""Diagrama vertical de la defensa escalonada en 3 niveles - diapositiva 12."""

from PIL import Image, ImageDraw, ImageFont
import os

OUT = r"C:\Users\lrojas\Desktop\P-LR-HKA\Defensa"

PURPLE   = "#4E2A84"
PURPLE_D = "#3A1F63"
LILAC    = "#EFE8F8"
AMBER    = "#C96A00"
AMBER_F  = "#FDEBD0"
GRAY     = "#555555"
GREEN    = "#2E7D4F"
GREEN_F  = "#E4F2E9"
BLUE     = "#1F5FA8"
BLUE_F   = "#E8F0FA"

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

f_h    = F(["segoeuib.ttf", "arialbd.ttf"], 46)
f_h2   = F(["segoeuib.ttf", "arialbd.ttf"], 36)
f_b    = F(["segoeui.ttf",  "arial.ttf"],   30)
f_s    = F(["segoeui.ttf",  "arial.ttf"],   27)
f_si   = F(["segoeuii.ttf", "ariali.ttf", "segoeui.ttf"], 26)
f_num  = F(["segoeuib.ttf", "arialbd.ttf"], 56)
f_badge= F(["segoeuib.ttf", "arialbd.ttf"], 24)

def center_text(d, cx, y, s, font, fill):
    d.text((cx, y), s, font=font, fill=fill, anchor="mm")

W, H = 1050, 1560
img = Image.new("RGB", (W, H), "white")
d = ImageDraw.Draw(img)
d.rounded_rectangle([4, 4, W - 4, H - 4], radius=26, fill="white",
                    outline=LILAC, width=6)

center_text(d, W / 2, 60, "Defensa escalonada", f_h, PURPLE_D)
center_text(d, W / 2, 112, "si un nivel no basta, escala al siguiente",
            f_si, GRAY)

levels = [
    ("1", "Defensa temporal", BLUE, BLUE_F,
     ["ventanas estrictas por operación:",
      "300 ms ACK reescritura",
      "2000 ms respuestas de estado",
      "3000 ms reporte S1",
      "máx. 3 reintentos"]),
    ("2", "Sanitización + integridad", AMBER, AMBER_F,
     ["limpieza determinista de",
      "búferes por transacción",
      "validación LRC:",
      "trama alterada → descarte + NAK"]),
    ("3", "Reconstrucción estructural", PURPLE, LILAC,
     ["si el error persiste:",
      "desmonta y reancla el",
      "controlador SPI",
      "sin reiniciar el SoC"]),
]

x1, x2 = 90, W - 90
y = 170
for num, titulo, col, colf, lineas in levels:
    bh = 110 + len(lineas) * 42
    d.rounded_rectangle([x1, y, x2, y + bh], radius=20, fill=colf,
                        outline=col, width=6)
    # numero
    d.ellipse([x1 + 20, y + 18, x1 + 96, y + 94], fill=col)
    center_text(d, x1 + 58, y + 56, num, f_num, "white")
    # titulo
    d.text((x1 + 120, y + 30), titulo, font=f_h2, fill=col)
    ty = y + 100
    for ln in lineas:
        d.text((x1 + 120, ty), ln, font=f_s, fill="#333333")
        ty += 42
    # flecha de escalado (excepto en el ultimo)
    if num != "3":
        ay = y + bh
        d.line([(W / 2, ay + 8), (W / 2, ay + 52)], fill=col, width=7)
        d.polygon([(W / 2, ay + 52), (W / 2 - 14, ay + 30),
                   (W / 2 + 14, ay + 30)], fill=col)
        d.text((W / 2 + 24, ay + 14), "persiste el fallo", font=f_si,
               fill=GRAY, anchor="lm")
    y += bh + 62

# resultado medido
d.rounded_rectangle([x1, y, x2, y + 150], radius=20, fill=GREEN_F,
                    outline=GREEN, width=6)
center_text(d, W / 2, y + 40, "Resultado medido", f_h2, GREEN)
center_text(d, W / 2, y + 85, "recuperación automática del bus", f_s, "#2c513c")
center_text(d, W / 2, y + 122, "0 tramas inválidas alcanzaron la BD", f_b, GREEN)

img.save(os.path.join(OUT, "diapositiva_12_niveles.png"), dpi=(200, 200))
print("OK: diapositiva_12_niveles.png")
