# -*- coding: utf-8 -*-
"""Mejora y anota la foto real del banco HIL (dos ESP32) para la diapositiva 10."""

from PIL import Image, ImageDraw, ImageFont, ImageEnhance, ImageFilter
import os

SRC = r"C:\Users\lrojas\Desktop\P-LR-HKA\Defensa\banco_hil_foto.jpg"
OUT = r"C:\Users\lrojas\Desktop\P-LR-HKA\Defensa"

PURPLE   = "#4E2A84"
PURPLE_D = "#3A1F63"

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

f_lab = F(["segoeuib.ttf", "arialbd.ttf"], 30)
f_sub = F(["segoeui.ttf",  "arial.ttf"],   24)

im = Image.open(SRC).convert("RGB")

# ---- mejora ligera: brillo, contraste, nitidez ----
im = ImageEnhance.Brightness(im).enhance(1.08)
im = ImageEnhance.Contrast(im).enhance(1.12)
im = ImageEnhance.Color(im).enhance(1.05)
im = im.filter(ImageFilter.UnsharpMask(radius=2, percent=120, threshold=2))

# recorte suave: centrar en las placas (quitar mesa vacia de abajo/derecha)
im = im.crop((0, 0, 972, 1000))

im.save(os.path.join(OUT, "banco_hil_foto_mejorada.jpg"), quality=92)

# ---- version anotada ----
d = ImageDraw.Draw(im)

def label(box_xy, lines, anchor_xy, color=PURPLE):
    """Caja blanca con borde morado + linea guia hacia anchor_xy."""
    x, y = box_xy
    pad = 16
    w = max(d.textlength(t, font=f) for t, f in lines) + 2 * pad
    h = len(lines) * 38 + 2 * pad - 8
    # linea guia
    ax, ay = anchor_xy
    cx, cy = x + w / 2, y + h / 2
    d.line([(cx, cy), (ax, ay)], fill=color, width=5)
    d.ellipse([ax - 10, ay - 10, ax + 10, ay + 10], outline=color, width=5)
    # caja
    d.rounded_rectangle([x, y, x + w, y + h], radius=12, fill="white",
                        outline=color, width=5)
    ty = y + pad - 4
    for t, f in lines:
        d.text((x + pad, ty), t, font=f, fill=PURPLE_D)
        ty += 38

# placa superior = esclavo (emulador del periferico)
label((560, 60), [("ESP32 esclavo", f_lab),
                  ("emula al periférico", f_sub)],
      anchor_xy=(430, 220))
# placa inferior = gateway bajo prueba
label((560, 830), [("ESP32 gateway", f_lab),
                   ("sistema bajo prueba", f_sub)],
      anchor_xy=(470, 780))
# cables centrales = bus sincrono
label((30, 460), [("bus síncrono", f_lab),
                  ("S1 · SV2 · ACK · NAK", f_sub)],
      anchor_xy=(380, 560))

im.save(os.path.join(OUT, "banco_hil_foto_anotada.png"))
print("OK: banco_hil_foto_mejorada.jpg + banco_hil_foto_anotada.png")
