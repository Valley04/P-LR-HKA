# -*- coding: utf-8 -*-
"""Panel 'consola de telemetria' para la diapositiva 11 (Resultado 1).
Muestra el JSON real del topico v1/fiscal/<serial>/json_completo con
campos clave destacados + la nota de la correccion off-by-one del SV2."""

from PIL import Image, ImageDraw, ImageFont
import os

OUT = r"C:\Users\lrojas\Desktop\P-LR-HKA\Defensa"

BG       = "#1E1230"   # terminal oscuro, tinte morado
BG_HDR   = "#2D1B47"
KEY      = "#C9A7F5"
VAL      = "#F4EFFB"
PUNCT    = "#9C85C4"
COMMENT  = "#7FBF9E"
HL       = "#33205A"   # realce de linea
PURPLE_D = "#3A1F63"
GRAY     = "#555555"
AMBER    = "#C96A00"
AMBER_F  = "#FDEBD0"

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

f_code = F(["consola.ttf",  "cour.ttf"],   34)
f_codeb= F(["consolab.ttf", "courbd.ttf"], 34)
f_comm = F(["consola.ttf",  "cour.ttf"],   27)
f_t    = F(["segoeuib.ttf", "arialbd.ttf"], 30)
f_s    = F(["segoeui.ttf",  "arial.ttf"],   30)
f_si   = F(["segoeuii.ttf", "ariali.ttf", "segoeui.ttf"], 28)
f_b    = F(["segoeuib.ttf", "arialbd.ttf"], 34)

W, H = 1400, 1540
img = Image.new("RGB", (W, H), "white")
d = ImageDraw.Draw(img)
d.rounded_rectangle([4, 4, W - 4, H - 4], radius=30, fill="white",
                    outline="#EFE8F8", width=6)

# ---- ventana de terminal ----
TX1, TY1, TX2, TY2 = 40, 40, W - 40, 1250
d.rounded_rectangle([TX1, TY1, TX2, TY2], radius=22, fill=BG)
d.rounded_rectangle([TX1, TY1, TX2, 130], radius=22, fill=BG_HDR)
d.rectangle([TX1, 105, TX2, 130], fill=BG_HDR)   # cuadrar union
for i in range(3):
    d.ellipse([TX1 + 28 + i * 44, 70, TX1 + 56 + i * 44, 98], fill="#6100AC")
d.text((TX1 + 175, 65), "telemetría en vivo — broker HiveMQ",
       font=f_t, fill=VAL)

def line(y, segments, hl=False, comment=None):
    """segments: lista de (texto, color). comment se dibuja a x fija."""
    if hl:
        d.rectangle([TX1 + 14, y - 8, TX2 - 14, y + 40], fill=HL)
    x = TX1 + 50
    for txt, col, bold in segments:
        f = f_codeb if bold else f_code
        d.text((x, y), txt, font=f, fill=col)
        x += d.textlength(txt, font=f)
    if comment:
        d.text((TX1 + 780, y + 5), comment, font=f_comm, fill=COMMENT)

# prompt / topico
line(160, [("pub → ", PUNCT, False),
           ("v1/fiscal/Z1F9999988/json_completo", KEY, True),
           ("   QoS 0", PUNCT, False)])

rows = [
    ("{", None, None),
    ("timestamp", ': 244210,', None),
    ("sts1", ': "0x60",', "← modo fiscal, en espera"),
    ("sts2", ': "0x40",', "← sin errores"),
    ("estado", ': "Modo Fiscal y en Espera",', None),
    ("error", ': "Ningún error",', None),
    ("rif", ': "J3121711970",', "← RIF contribuyente"),
    ("registro", ': "Z1F9999988",', "← serial de la máquina"),
    ("ventas", ': "…0000",', None),
    ("cierres_z", ': "0000",', None),
    ("hora", ': "123000",  "fecha": "300126"', None),
    ("}", None, None),
]
y = 230
for key, rest, note in rows:
    hl = note is not None and ("0x60" in rest or "0x40" in rest)
    if key in ("{", "}"):
        line(y, [(key, PUNCT, False)])
    else:
        line(y, [('"' + key + '"', KEY, True), (rest, VAL, False)],
             hl=hl, comment=note)
    y += 84

# ---- banda SV2 / correccion ----
d.rounded_rectangle([40, 1290, W - 40, 1490], radius=20, fill=AMBER_F,
                    outline=AMBER, width=5)
d.text((80, 1318), "SV2 · versión del periférico", font=f_b, fill=AMBER)
d.text((80, 1375), '"020507GD00"', font=f_codeb, fill=PURPLE_D)
d.text((80, 1438), "llega íntegra — off-by-one corregido:",
       font=f_si, fill="#5a3a10")
d.text((600, 1438), "delimitador dinámico + '\\0' explícito",
       font=F(["consolai.ttf", "consola.ttf"], 28), fill="#5a3a10")
# check dibujado a la derecha
cx, cy = W - 200, 1390
d.line([(cx - 45, cy), (cx - 15, cy + 35)], fill=AMBER, width=16)
d.line([(cx - 15, cy + 35), (cx + 55, cy - 40)], fill=AMBER, width=16)

img.save(os.path.join(OUT, "diapositiva_11_telemetria.png"), dpi=(200, 200))
print("OK: diapositiva_11_telemetria.png")
