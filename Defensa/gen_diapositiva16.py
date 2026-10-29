# -*- coding: utf-8 -*-
"""Diapositiva 16 - Resultado 6: OTA de doble objetivo.
Panel izq: gateway con particion dual. Panel der: reescritura del
periferico por fragmentos de 1024 B. Paleta monocromo del deck."""

from PIL import Image, ImageDraw, ImageFont
import math, os

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

f_h   = F(["segoeuib.ttf", "arialbd.ttf"], 48)
f_h2  = F(["segoeuib.ttf", "arialbd.ttf"], 36)
f_b   = F(["segoeui.ttf",  "arial.ttf"],   32)
f_s   = F(["segoeui.ttf",  "arial.ttf"],   28)
f_si  = F(["segoeuii.ttf", "ariali.ttf", "segoeui.ttf"], 27)
f_mono= F(["consolab.ttf", "courbd.ttf"],  30)

def center_text(d, cx, y, s, font, fill):
    d.text((cx, y), s, font=font, fill=fill, anchor="mm")

def arrow(d, x1, y1, x2, y2, color=PURPLE, width=6, head=16):
    d.line([(x1, y1), (x2, y2)], fill=color, width=width)
    ang = math.atan2(y2 - y1, x2 - x1)
    d.polygon([(x2, y2),
               (x2 - head * math.cos(ang - 0.40), y2 - head * math.sin(ang - 0.40)),
               (x2 - head * math.cos(ang + 0.40), y2 - head * math.sin(ang + 0.40))],
              fill=color)

def panel(title, sub):
    W, H = 1500, 1350
    img = Image.new("RGB", (W, H), "white")
    d = ImageDraw.Draw(img)
    d.rounded_rectangle([4, 4, W - 4, H - 4], radius=30, fill="white",
                        outline=LILAC2, width=6)
    center_text(d, W / 2, 70,  title, f_h, PURPLE)
    center_text(d, W / 2, 122, sub, f_si, GRAY)
    return img, d

def ok_badge(d, cx, y):
    d.rounded_rectangle([cx - 260, y - 32, cx + 260, y + 32], radius=16,
                        fill=PURPLE)
    center_text(d, cx, y, "✓ validado extremo a extremo".replace("✓ ", ""),
                f_s, "white")
    # check dibujado
    gx = cx - 225; gy = y
    d.line([(gx - 10, gy), (gx - 3, gy + 8)], fill="white", width=4)
    d.line([(gx - 3, gy + 8), (gx + 12, gy - 9)], fill="white", width=4)

# ============================================================
# PANEL IZQUIERDO — Objetivo 1: gateway, particion dual
# ============================================================
img, d = panel("OBJETIVO 1 — Gateway ESP32",
               "partición OTA dual · rollback por bootloader")

# paso 1: orden + bandera
d.rounded_rectangle([150, 180, 1350, 300], radius=18, fill=LILAC,
                    outline=PURPLE, width=5)
center_text(d, 750, 222, "orden OTA por MQTT  →  bandera activada", f_b, "#222222")
center_text(d, 750, 268, "el sondeo se detiene durante la actualización",
            f_si, GRAY)

arrow(d, 750, 308, 750, 380)

# particiones
d.rounded_rectangle([120, 390, 1380, 780], radius=22, fill="#FAF8FD",
                    outline=PURPLE, width=5)
center_text(d, 750, 435, "flash — esquema OTA dual", f_h2, PURPLE)

# particion A (activa)
d.rounded_rectangle([170, 480, 660, 740], radius=18, fill=PURPLE)
center_text(d, 415, 530, "PARTICIÓN A", f_h2, "white")
center_text(d, 415, 585, "firmware activo", f_b, "#E6DAF5")
center_text(d, 415, 630, "en ejecución", f_b, "#E6DAF5")
center_text(d, 415, 690, "intacta — punto de retorno", f_si, "#C9B6E4")

# particion B (inactiva)
d.rounded_rectangle([840, 480, 1330, 740], radius=18, fill="white",
                    outline=ACCENT, width=5)
d.rounded_rectangle([848, 488, 1322, 732], radius=14, outline=ACCENT,
                    width=3)
center_text(d, 1085, 530, "PARTICIÓN B", f_h2, ACCENT)
center_text(d, 1085, 585, "OTA inactiva", f_b, "#333333")
center_text(d, 1085, 630, "aquí se escribe", f_b, "#333333")
center_text(d, 1085, 690, "la nueva versión", f_b, "#333333")
arrow(d, 670, 610, 830, 610, ACCENT)
d.text((750, 570), "escribe", font=f_si, fill=ACCENT, anchor="mm")

# paso 3: bootloader
arrow(d, 750, 790, 750, 870)
d.rounded_rectangle([150, 880, 1350, 1050], radius=18, fill=LILAC,
                    outline=PURPLE, width=5)
center_text(d, 750, 930, "si ocurre un corte de energía →", f_b, "#222222")
center_text(d, 750, 978, "el bootloader arranca desde la partición intacta",
            f_b, PURPLE)
center_text(d, 750, 1020, "el dispositivo nunca queda en ladrillo", f_si, GRAY)

ok_badge(d, 750, 1150)

img.save(os.path.join(OUT, "diapositiva_16_ota_gateway.png"), dpi=(200, 200))
print("OK: diapositiva_16_ota_gateway.png")

# ============================================================
# PANEL DERECHO — Objetivo 2: periferico, fragmentos 1024 B
# ============================================================
img, d = panel("OBJETIVO 2 — Periférico fiscal",
               "sin pila de red — el gateway actúa como host de programación")

CX = 750  # eje central del pipeline vertical

# paso 1: binario
d.rounded_rectangle([330, 180, 1170, 300], radius=18, fill=LILAC,
                    outline=PURPLE, width=5)
center_text(d, CX, 220, "binario nuevo (nube)", f_h2, PURPLE)
center_text(d, CX, 268, "supera el heap libre del ESP32", f_s, "#555555")

arrow(d, CX, 308, CX, 380)
d.text((CX + 20, 330), "descarga por fragmentos — no se almacena completo",
       font=f_si, fill=GRAY, anchor="lm")

# paso 2: bufer
d.rounded_rectangle([330, 390, 1170, 510], radius=18, fill=LILAC,
                    outline=PURPLE, width=5)
center_text(d, CX, 432, "búfer de trabajo fijo — 1024 B", f_h2, PURPLE)
center_text(d, CX, 478, "cada fragmento viaja con cabecera + redundancia",
            f_s, "#555555")

arrow(d, CX, 518, CX, 590)
d.text((CX + 20, 545), "programa sobre el mismo bus síncrono del sondeo",
       font=f_si, fill=GRAY, anchor="lm")

# paso 3: periferico
d.rounded_rectangle([330, 600, 1170, 720], radius=18, fill=PURPLE)
center_text(d, CX, 650, "PERIFÉRICO", f_h2, "white")
center_text(d, CX, 695, "firmware reescrito in situ", f_s, "#E6DAF5")

# reglas como chips
rules = ["≤ 3 reintentos + ACK / fragmento",
         "cierre solo si bytes == longitud declarada",
         "corte de red → reanuda con HTTP Range"]
y = 770
for r in rules:
    w = d.textlength(r, font=f_s) + 50
    d.rounded_rectangle([CX - w / 2, y, CX + w / 2, y + 54], radius=27,
                        fill="white", outline=ACCENT, width=4)
    center_text(d, CX, y + 27, r, f_s, "#333333")
    y += 68

# resumen del cierre
d.rounded_rectangle([150, y + 12, 1350, y + 150], radius=18, fill="white",
                    outline=ACCENT, width=5)
center_text(d, CX, y + 55, "una transferencia parcial nunca se compromete",
            f_b, "#333333")
center_text(d, CX, y + 108, "y nunca reinicia desde cero ante cortes",
            f_si, GRAY)

ok_badge(d, CX, y + 215)

img.save(os.path.join(OUT, "diapositiva_16_ota_periferico.png"),
         dpi=(200, 200))
print("OK: diapositiva_16_ota_periferico.png")
