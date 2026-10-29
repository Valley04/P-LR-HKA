# -*- coding: utf-8 -*-
"""Diagrama del banco HIL para la diapositiva 10: Metodologia de validacion."""

from PIL import Image, ImageDraw, ImageFont
import math, os

OUT = r"C:\Users\lrojas\Desktop\P-LR-HKA\Defensa"

PURPLE   = "#4E2A84"
PURPLE_D = "#3A1F63"
LILAC    = "#EFE8F8"
AMBER    = "#C96A00"
AMBER_F  = "#FDEBD0"
BLUE     = "#1F5FA8"
BLUE_F   = "#E8F0FA"
GRAY     = "#555555"
GREEN    = "#2E7D4F"
GREEN_F  = "#E4F2E9"

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

f_h    = F(["segoeuib.ttf", "arialbd.ttf"], 52)
f_h2   = F(["segoeuib.ttf", "arialbd.ttf"], 40)
f_big  = F(["segoeuib.ttf", "arialbd.ttf"], 46)
f_b    = F(["segoeui.ttf",  "arial.ttf"],   32)
f_s    = F(["segoeui.ttf",  "arial.ttf"],   28)
f_si   = F(["segoeuii.ttf", "ariali.ttf", "segoeui.ttf"], 28)
f_mono = F(["consolab.ttf", "courbd.ttf"],  30)

def center_text(d, cx, y, s, font, fill):
    d.text((cx, y), s, font=font, fill=fill, anchor="mm")

def double_arrow(d, x1, y1, x2, y2, color, gap=16, width=6, head=16):
    """Dos flechas paralelas en sentidos opuestos."""
    dx, dy = x2 - x1, y2 - y1
    L = math.hypot(dx, dy)
    ux, uy = dx / L, dy / L          # unitario
    px, py = -uy, ux                 # perpendicular
    for off, fwd in [(+gap / 2, True), (-gap / 2, False)]:
        ax = x1 + px * off; ay = y1 + py * off
        bx = x2 + px * off; by = y2 + py * off
        if not fwd:
            ax, ay, bx, by = bx, by, ax, ay
        d.line([(ax, ay), (bx, by)], fill=color, width=width)
        ang = math.atan2(by - ay, bx - ax)
        d.polygon([(bx, by),
                   (bx - head * math.cos(ang - 0.42), by - head * math.sin(ang - 0.42)),
                   (bx - head * math.cos(ang + 0.42), by - head * math.sin(ang + 0.42))],
                  fill=color)

W, H = 1500, 1220
img = Image.new("RGB", (W, H), "white")
d = ImageDraw.Draw(img)
d.rounded_rectangle([4, 4, W - 4, H - 4], radius=30, fill="white",
                    outline=LILAC, width=6)

center_text(d, W / 2, 70,  "Banco HIL", f_h, PURPLE_D)
center_text(d, W / 2, 120, "Hardware-in-the-Loop — dos niveles de emulación",
            f_si, GRAY)

# ---- ESP32 bajo prueba (izquierda) ----
d.rounded_rectangle([70, 430, 430, 800], radius=20, fill=LILAC,
                    outline=PURPLE, width=6)
center_text(d, 250, 510, "ESP32", f_big, PURPLE_D)
center_text(d, 250, 570, "gateway", f_b, "#333333")
center_text(d, 250, 640, "sistema", f_s, GRAY)
center_text(d, 250, 680, "bajo prueba", f_s, GRAY)
center_text(d, 250, 745, "firmware real", f_si, GRAY)

# ---- Nivel 1: emulador software ----
d.rounded_rectangle([560, 170, 1430, 480], radius=20, fill=BLUE_F,
                    outline=BLUE, width=6)
center_text(d, 995, 225, "NIVEL 1 · Emulador en software", f_h2, BLUE)
d.text((995, 285), "impresora.py", font=f_mono, fill="#1a3a5c", anchor="mm")
for i, ln in enumerate(["pyserial · mismos parámetros eléctricos del enlace",
                        "máquina de estados · calcula LRC",
                        "responde S1 · SV2 · ACK · NAK"]):
    center_text(d, 995, 340 + i * 44, ln, f_s, "#26435f")

# ---- Nivel 2: emulador hardware ----
d.rounded_rectangle([560, 540, 1430, 850], radius=20, fill=BLUE_F,
                    outline=BLUE, width=6)
center_text(d, 995, 595, "NIVEL 2 · Emulador en hardware", f_h2, BLUE)
d.text((995, 655), "ESP32 esclavo", font=f_mono, fill="#1a3a5c", anchor="mm")
for i, ln in enumerate(["mismo bus físico síncrono",
                        "emula íntegramente al periférico",
                        "replica el protocolo de reescritura OTA"]):
    center_text(d, 995, 710 + i * 44, ln, f_s, "#26435f")

# ---- flechas bidireccionales ----
double_arrow(d, 430, 480, 555, 330, PURPLE)
double_arrow(d, 430, 700, 555, 700, PURPLE)
d.text((492, 395), "bus\nsíncrono", font=f_s, fill=GRAY, anchor="mm")

# ---- inyeccion de fallos ----
d.rounded_rectangle([70, 940, 900, 1160], radius=20, fill=AMBER_F,
                    outline=AMBER, width=6)
center_text(d, 485, 990, "Inyección de fallos determinista", f_h2, AMBER)
chips = [["paquetes fuera de secuencia", "checksum alterado"],
         ["corte de enlace"]]
for row, (labels, ey) in enumerate(zip(chips, [1040, 1100])):
    ws = [d.textlength(t, font=f_s) + 30 for t in labels]
    tot = sum(ws) + 20 * (len(labels) - 1)
    ex = 485 - tot / 2
    for t, w in zip(labels, ws):
        d.rounded_rectangle([ex, ey, ex + w, ey + 46], radius=12,
                            fill="white", outline=AMBER, width=4)
        d.text((ex + 15, ey + 7), t, font=f_s, fill="#5a3a10")
        ex += w + 20

# ---- HKA80 real ----
d.rounded_rectangle([940, 940, 1430, 1160], radius=20, fill=GREEN_F,
                    outline=GREEN, width=6)
center_text(d, 1185, 1000, "Impresora HKA80 real", f_h2, GREEN)
center_text(d, 1185, 1055, "equipo disponible la última", f_s, "#2c513c")
center_text(d, 1185, 1095, "semana — validación final", f_s, "#2c513c")

img.save(os.path.join(OUT, "diapositiva_10_hil.png"), dpi=(200, 200))
print("OK: diapositiva_10_hil.png")
