# -*- coding: utf-8 -*-
"""Genera el diagrama de apoyo para la diapositiva 7:
Nivel Gateway: ESP32, ESP-IDF y kernel concurrente (FreeRTOS)."""

from PIL import Image, ImageDraw, ImageFont
import math, os

W, H = 2000, 1400
img = Image.new("RGB", (W, H), "white")
d = ImageDraw.Draw(img)

# Paleta (morado de la plantilla + acentos)
PURPLE   = "#4E2A84"
PURPLE_D = "#3A1F63"
LILAC    = "#EFE8F8"
LILAC_BG = "#F8F5FC"
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

f_title  = F(["segoeuib.ttf", "arialbd.ttf"], 44)
f_core   = F(["segoeuib.ttf", "arialbd.ttf"], 42)
f_body   = F(["segoeui.ttf",  "arial.ttf"],   34)
f_small  = F(["segoeui.ttf",  "arial.ttf"],   29)
f_smallI = F(["segoeuii.ttf", "ariali.ttf", "segoeui.ttf"], 29)
f_mono   = F(["consolab.ttf", "courbd.ttf"],  38)
f_ext    = F(["segoeuib.ttf", "arialbd.ttf"], 36)

def center_text(cx, y, s, font, fill):
    d.text((cx, y), s, font=font, fill=fill, anchor="mm")

def arrow(x1, y1, x2, y2, color, width=7, dashed=False, head=18):
    """Linea con punta de flecha en (x2, y2); soporta segmentos en L via polyline."""
    if dashed:
        # segmentos
        length = math.hypot(x2 - x1, y2 - y1)
        if length == 0:
            return
        ux, uy = (x2 - x1) / length, (y2 - y1) / length
        pos, dash, gap = 0, 18, 12
        while pos < length - head:
            seg = min(dash, length - head - pos)
            d.line([(x1 + ux * pos, y1 + uy * pos),
                    (x1 + ux * (pos + seg), y1 + uy * (pos + seg))],
                   fill=color, width=width)
            pos += dash + gap
    else:
        d.line([(x1, y1), (x2 - (x2 - x1) * 0, y2)], fill=color, width=width)
    # punta
    ang = math.atan2(y2 - y1, x2 - x1)
    p1 = (x2, y2)
    p2 = (x2 - head * math.cos(ang - 0.42), y2 - head * math.sin(ang - 0.42))
    p3 = (x2 - head * math.cos(ang + 0.42), y2 - head * math.sin(ang + 0.42))
    d.polygon([p1, p2, p3], fill=color)

def dashed_poly(points, color, width=6, dash=18, gap=12):
    """Polilinea punteada; el ultimo tramo lleva punta de flecha."""
    for (x1, y1), (x2, y2) in zip(points, points[1:]):
        length = math.hypot(x2 - x1, y2 - y1)
        if length == 0:
            continue
        ux, uy = (x2 - x1) / length, (y2 - y1) / length
        pos = 0.0
        while pos < length:
            seg = min(dash, length - pos)
            d.line([(x1 + ux * pos, y1 + uy * pos),
                    (x1 + ux * (pos + seg), y1 + uy * (pos + seg))],
                   fill=color, width=width)
            pos += dash + gap
    x2, y2 = points[-1]
    x1, y1 = points[-2]
    ang = math.atan2(y2 - y1, x2 - x1)
    d.polygon([(x2, y2),
               (x2 - 18 * math.cos(ang - 0.42), y2 - 18 * math.sin(ang - 0.42)),
               (x2 - 18 * math.cos(ang + 0.42), y2 - 18 * math.sin(ang + 0.42))],
              fill=color)

def padlock(cx, cy, scale, color):
    """Candado simple: arco + cuerpo."""
    r = 14 * scale
    d.arc([cx - r, cy - 2 * r - 6 * scale, cx + r, cy + 2 * scale],
          start=180, end=360, fill=color, width=int(5 * scale))
    bw, bh = 34 * scale, 26 * scale
    d.rounded_rectangle([cx - bw / 2, cy - bh / 2, cx + bw / 2, cy + bh / 2],
                        radius=5 * scale, fill=color)
    d.ellipse([cx - 4 * scale, cy - 5 * scale, cx + 4 * scale, cy + 3 * scale],
              fill="white")

# ================= CONTENEDOR ESP32 =================
d.rounded_rectangle([430, 90, 1570, 1150], radius=26,
                    fill=LILAC_BG, outline=PURPLE, width=7)
center_text(1000, 160, "ESP32 DevKitC V4   ·   ESP-IDF  +  kernel FreeRTOS",
            f_title, PURPLE_D)

# ================= NUCLEOS =================
def core_box(x1, y1, x2, y2, titulo, l1, l2):
    d.rounded_rectangle([x1, y1, x2, y2], radius=20,
                        fill=LILAC, outline=PURPLE, width=6)
    cx = (x1 + x2) / 2
    center_text(cx, y1 + 55,  titulo, f_core, PURPLE_D)
    center_text(cx, y1 + 120, l1, f_body, "#222222")
    center_text(cx, y1 + 175, l2, f_smallI, GRAY)

core_box(500, 240, 970, 470, "NÚCLEO 0",
         "Tarea de adquisición fiscal", "sondeo S1 / SV2 cada 5 s")
core_box(1030, 240, 1500, 470, "NÚCLEO 1",
         "Tarea de red", "Wi-Fi · MQTT · TLS")

# ================= MUTEX =================
def mutex_box(x1, y1, x2, y2, nombre, subt):
    d.rounded_rectangle([x1, y1, x2, y2], radius=20,
                        fill=AMBER_F, outline=AMBER, width=6)
    padlock(x1 + 60, (y1 + y2) / 2 - 8, 1.6, AMBER)
    cx = (x1 + x2) / 2 + 30
    center_text(cx, y1 + 62, nombre, f_mono, AMBER)
    center_text(cx, y1 + 122, subt, f_small, "#5a3a10")

mutex_box(500,  760, 970,  930, "bus_local_mutex",  "acceso exclusivo al bus")
mutex_box(1030, 760, 1500, 930, "mqtt_data_mutex", "lectura coherente")

# ================= RECURSOS COMPARTIDOS =================
def res_box(x1, y1, x2, y2, t, subt):
    d.rounded_rectangle([x1, y1, x2, y2], radius=18,
                        fill=GREEN_F, outline=GREEN, width=6)
    cx = (x1 + x2) / 2
    center_text(cx, y1 + 45, t, f_body, GREEN)
    center_text(cx, y1 + 92, subt, f_small, "#2c513c")

res_box(500,  1010, 970,  1125, "Bus local",  "sondeo periódico · OTA")
res_box(1030, 1010, 1500, 1125, "Estructura de telemetría", "S1 · SV2 · RSSI · estado")

# ================= EXTERNOS =================
# Impresora (izquierda, a la altura del bus)
d.rounded_rectangle([50, 985, 390, 1150], radius=20,
                    fill=BLUE_F, outline=BLUE, width=6)
center_text(220, 1030, "Impresora fiscal", f_ext, BLUE)
center_text(220, 1080, "HKA80", f_ext, BLUE)
center_text(220, 1122, "bus síncrono 100 kHz", f_small, "#26435f")

# Broker (derecha, a la altura del nucleo 1)
d.rounded_rectangle([1620, 250, 1950, 460], radius=20,
                    fill=BLUE_F, outline=BLUE, width=6)
center_text(1785, 310, "Broker MQTT", f_ext, BLUE)
center_text(1785, 360, "HiveMQ", f_body, "#26435f")
center_text(1785, 410, "TLS · AES", f_small, "#26435f")

# ================= FLECHAS =================
# Nucleo0 -> bus_local_mutex (solido)
arrow(640, 470, 640, 756, PURPLE)
# Nucleo1 -> mqtt_data_mutex (solido)
arrow(1370, 470, 1370, 756, PURPLE)

# Nucleo0 -> mqtt_data_mutex (punteado: escribe telemetria)
dashed_poly([(900, 470), (900, 560), (1150, 560), (1150, 756)], PURPLE)
d.text((1025, 532), "escribe S1 / SV2", font=f_small, fill=GRAY, anchor="mm")

# Nucleo1 -> bus_local_mutex (punteado: OTA)
dashed_poly([(1100, 470), (1100, 650), (830, 650), (830, 756)], PURPLE)
d.text((965, 622), "OTA", font=f_small, fill=GRAY, anchor="mm")

# mutex -> recurso (solido)
arrow(735, 930, 735, 1006, GREEN)
arrow(1265, 930, 1265, 1006, GREEN)

# bus -> impresora (bidireccional simple)
arrow(500, 1067, 396, 1067, BLUE)
arrow(396, 1097, 500, 1097, BLUE)

# nucleo1 -> broker
arrow(1500, 355, 1614, 355, BLUE)
d.text((1565, 322), "TLS", font=f_small, fill=GRAY, anchor="mm")

# ================= LEYENDA =================
lx, ly = 1560, 1010
f_leg = F(["segoeui.ttf", "arial.ttf"], 26)
d.line([(lx, ly), (lx + 70, ly)], fill=PURPLE, width=7)
d.polygon([(lx + 70, ly), (lx + 52, ly - 9), (lx + 52, ly + 9)], fill=PURPLE)
d.text((lx + 90, ly), "rol primario", font=f_leg, fill=GRAY, anchor="lm")
# punteada
for i in range(3):
    d.line([(lx + i * 26, ly + 60), (lx + i * 26 + 16, ly + 60)], fill=PURPLE, width=7)
d.polygon([(lx + 80, ly + 60), (lx + 62, ly + 51), (lx + 62, ly + 69)], fill=PURPLE)
d.text((lx + 98, ly + 60), "acceso vía mutex", font=f_leg, fill=GRAY, anchor="lm")

out = r"C:\Users\lrojas\Desktop\P-LR-HKA\Defensa\diapositiva_07_gateway.png"
img.save(out, dpi=(200, 200))
print("OK:", out)
