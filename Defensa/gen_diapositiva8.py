# -*- coding: utf-8 -*-
"""Genera los dos paneles de apoyo para la diapositiva 8:
Nivel Cloud: MQTT v5.0 / TLS y Django + HTMX.
Panel A: esquema de topicos MQTT + seguridad.
Panel B: arquitectura del servidor (worker MQTT + Django + navegador HTMX)."""

from PIL import Image, ImageDraw, ImageFont
import math, os

OUT = r"C:\Users\lrojas\Desktop\P-LR-HKA\Defensa"

# Paleta consistente con diapositiva 7
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

f_h    = F(["segoeuib.ttf", "arialbd.ttf"], 56)
f_h2   = F(["segoeuib.ttf", "arialbd.ttf"], 42)
f_b    = F(["segoeui.ttf",  "arial.ttf"],   34)
f_s    = F(["segoeui.ttf",  "arial.ttf"],   28)
f_si   = F(["segoeuii.ttf", "ariali.ttf", "segoeui.ttf"], 28)
f_mono = F(["consola.ttf",  "cour.ttf"],    33)
f_monob= F(["consolab.ttf", "courbd.ttf"],  34)
f_tag  = F(["segoeuib.ttf", "arialbd.ttf"], 24)

def center_text(d, cx, y, s, font, fill):
    d.text((cx, y), s, font=font, fill=fill, anchor="mm")

def arrow(d, x1, y1, x2, y2, color, width=7, head=18):
    d.line([(x1, y1), (x2, y2)], fill=color, width=width)
    ang = math.atan2(y2 - y1, x2 - x1)
    d.polygon([(x2, y2),
               (x2 - head * math.cos(ang - 0.42), y2 - head * math.sin(ang - 0.42)),
               (x2 - head * math.cos(ang + 0.42), y2 - head * math.sin(ang + 0.42))],
              fill=color)

def dashed_poly(d, points, color, width=6, dash=16, gap=11):
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
    x2, y2 = points[-1]; x1, y1 = points[-2]
    ang = math.atan2(y2 - y1, x2 - x1)
    d.polygon([(x2, y2),
               (x2 - 16 * math.cos(ang - 0.40), y2 - 16 * math.sin(ang - 0.40)),
               (x2 - 16 * math.cos(ang + 0.40), y2 - 16 * math.sin(ang + 0.40))],
              fill=color)

def card(w, h):
    img = Image.new("RGB", (w, h), "white")
    d = ImageDraw.Draw(img)
    d.rounded_rectangle([4, 4, w - 4, h - 4], radius=30,
                        fill="white", outline=LILAC, width=6)
    return img, d

# ============================================================
# PANEL A — Esquema de topicos MQTT v5.0 / TLS
# ============================================================
W1, H1 = 1500, 1290
im, d = card(W1, H1)

center_text(d, W1 / 2, 75,  "MQTT v5.0  sobre  TLS 1.2", f_h, PURPLE_D)
center_text(d, W1 / 2, 132, "broker HiveMQ Cloud  ·  puerto 8883  ·  certificados CA",
            f_si, GRAY)

# ESP32 -> broker
d.rounded_rectangle([70, 200, 420, 330], radius=18, fill=LILAC, outline=PURPLE, width=6)
center_text(d, 245, 245, "ESP32", f_h2, PURPLE_D)
center_text(d, 295, 295, "gateway", f_s, GRAY)

d.rounded_rectangle([1080, 200, 1430, 330], radius=18, fill=BLUE_F, outline=BLUE, width=6)
center_text(d, 1255, 245, "HiveMQ", f_h2, BLUE)
center_text(d, 1255, 295, "broker", f_s, GRAY)

arrow(d, 430, 265, 1070, 265, PURPLE)
d.rounded_rectangle([640, 215, 860, 315], radius=14, fill=AMBER_F, outline=AMBER, width=5)
center_text(d, 750, 265, "TLS 🔒" if False else "TLS", f_h2, AMBER)

# Arbol de topicos
d.rounded_rectangle([70, 400, 1430, 900], radius=22, fill="#FAF8FD",
                    outline=PURPLE, width=5)
d.text((110, 430), "v1/fiscal/", font=f_mono, fill=GRAY)
d.text((330, 430), "<serial>/", font=f_monob, fill=PURPLE)

rows = [
    ("json_completo",    "PUB",  PURPLE, "telemetría S1 / SV2 cada 5 s"),
    ("ota",              "SUB",  BLUE,   "comandos de actualización remota"),
    ("alertas_conexion", "LWT",  AMBER,  "última voluntad · retained · QoS 1"),
]
y = 540
for name, tag, tc, desc in rows:
    # conector del arbol
    d.line([(120, 480), (120, y + 22)], fill=PURPLE, width=5)
    d.line([(120, y + 22), (170, y + 22)], fill=PURPLE, width=5)
    d.text((185, y), name, font=f_mono, fill="#222222")
    # etiqueta
    d.rounded_rectangle([660, y - 8, 660 + 30 + d.textlength(tag, font=f_tag), y + 40],
                        radius=10, fill=tc)
    d.text((675, y + 7), tag, font=f_tag, fill="white")
    d.text((840, y), desc, font=f_s, fill=GRAY)
    y += 120
d.line([(120, 480), (120, 480)], fill=PURPLE)

# Chips de capacidades v5.0
def chip(x1, y1, x2, y2, titulo, sub):
    d.rounded_rectangle([x1, y1, x2, y2], radius=18, fill=GREEN_F,
                        outline=GREEN, width=5)
    cx = (x1 + x2) / 2
    center_text(d, cx, y1 + 48, titulo, f_b, GREEN)
    center_text(d, cx, y1 + 100, sub, f_s, "#2c513c")

chip(70,  950, 720,  1120, "User Properties",
     "serial · firmware · RSSI en cabecera")
chip(780, 950, 1430, 1120, "Last Will and Testament",
     "el broker avisa si el equipo cae")

im.save(os.path.join(OUT, "diapositiva_08_mqtt.png"), dpi=(200, 200))
print("OK: diapositiva_08_mqtt.png")

# ============================================================
# PANEL B — Servidor: worker MQTT + Django + navegador HTMX
# ============================================================
W2, H2 = 1500, 1290
im, d = card(W2, H2)

center_text(d, W2 / 2, 75, "Servidor: Django + HTMX", f_h, PURPLE_D)
center_text(d, W2 / 2, 132, "worker MQTT en segundo plano  ·  tiempo real al navegador",
            f_si, GRAY)

# Broker
d.rounded_rectangle([60, 200, 360, 360], radius=18, fill=BLUE_F, outline=BLUE, width=6)
center_text(d, 210, 255, "HiveMQ", f_h2, BLUE)
center_text(d, 210, 305, "broker", f_s, GRAY)

# Worker MQTT
d.rounded_rectangle([430, 180, 1040, 470], radius=20, fill=LILAC, outline=PURPLE, width=6)
center_text(d, 735, 235, "Worker MQTT", f_h2, PURPLE_D)
center_text(d, 735, 285, "descompone cada trama en eventos atómicos", f_si, GRAY)
# chips de eventos (2 filas, centradas en el ancho del worker 430..1040)
rows_chips = [
    ["cambio de estado", "corte Z", "factura"],
    ["reconexión", "actualización OTA"],
]
chip_y = [330, 400]
for labels, ey in zip(rows_chips, chip_y):
    widths = [d.textlength(t, font=f_s) + 34 for t in labels]
    total = sum(widths) + 24 * (len(labels) - 1)
    ex = 735 - total / 2
    for label, w in zip(labels, widths):
        d.rounded_rectangle([ex, ey, ex + w, ey + 48], radius=14,
                            fill="white", outline=PURPLE, width=4)
        d.text((ex + 17, ey + 8), label, font=f_s, fill=PURPLE_D)
        ex += w + 24

# Django / BD
d.rounded_rectangle([1100, 180, 1440, 470], radius=20, fill=GREEN_F, outline=GREEN, width=6)
center_text(d, 1270, 250, "Django", f_h2, GREEN)
center_text(d, 1270, 310, "persiste eventos", f_s, "#2c513c")
center_text(d, 1270, 355, "bitácora · serial", f_s, "#2c513c")
center_text(d, 1270, 415, "sirve vistas HTMX", f_si, "#2c513c")

arrow(d, 370, 280, 420, 280, PURPLE)
arrow(d, 1050, 320, 1092, 320, PURPLE)

# Navegador
d.rounded_rectangle([430, 720, 1440, 1010], radius=20, fill=BLUE_F, outline=BLUE, width=6)
center_text(d, 935, 775, "Navegador — dashboard", f_h2, BLUE)
d.rounded_rectangle([520, 830, 920, 950], radius=14, fill="white", outline=BLUE, width=4)
center_text(d, 720, 868, "cliente MQTT JS", f_b, "#26435f")
center_text(d, 720, 915, "WebSocket 8884", f_s, GRAY)
d.rounded_rectangle([970, 830, 1370, 950], radius=14, fill="white", outline=BLUE, width=4)
center_text(d, 1170, 868, "HTMX", f_b, "#26435f")
center_text(d, 1170, 915, "experiencia de página única", f_s, GRAY)

# Django -> navegador (HTTP/HTMX)
arrow(d, 1270, 480, 1270, 710, GREEN)
d.text((1300, 580), "HTTP + HTMX", font=f_s, fill=GRAY, anchor="lm")

# Broker -> navegador (punteado: WS 8884 tiempo real)
dashed_poly(d, [(210, 360), (210, 865), (420, 865)], BLUE)
d.text((235, 620), "MQTT / WS 8884", font=f_s, fill=GRAY, anchor="lm")
d.text((235, 660), "tiempo real", font=f_si, fill=GRAY, anchor="lm")

# Leyenda minima
f_leg = F(["segoeui.ttf", "arial.ttf"], 25)
d.line([(60, 1090), (130, 1090)], fill=PURPLE, width=7)
d.polygon([(130, 1090), (112, 1081), (112, 1099)], fill=PURPLE)
d.text((150, 1090), "ingesta / persistencia", font=f_leg, fill=GRAY, anchor="lm")
for i in range(3):
    d.line([(60 + i * 26, 1150), (60 + i * 26 + 16, 1150)], fill=BLUE, width=7)
d.polygon([(86, 1150), (68, 1141), (68, 1159)], fill=BLUE)
d.text((150, 1150), "canal directo al navegador", font=f_leg, fill=GRAY, anchor="lm")

im.save(os.path.join(OUT, "diapositiva_08_django.png"), dpi=(200, 200))
print("OK: diapositiva_08_django.png")
