# Generador de Firmware Simulado (Mock)
tamaño_bytes = 2500
nombre_archivo = "firmware_prueba.bin"

with open(nombre_archivo, "wb") as f:
    # Generamos un patrón repetitivo del 0 al 255 (0x00 a 0xFF)
    for i in range(tamaño_bytes):
        f.write(bytes([i % 256]))

print(f"✅ Archivo '{nombre_archivo}' generado con éxito ({tamaño_bytes} bytes).")