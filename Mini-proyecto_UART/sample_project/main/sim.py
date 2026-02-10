import serial
import time
import sys

# --- CONFIGURACIÓN ---
PUERTO = "COM7"  # <--- ¡VERIFICA TU PUERTO!
BAUDIOS = 9600

try:
    ser = serial.Serial(
        port=PUERTO,
        baudrate=BAUDIOS,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,  # Coincide con el ESP32
        stopbits=serial.STOPBITS_ONE,
        timeout=0.1
    )
    print(f"✅ PC ESCUCHANDO EN {PUERTO}...")
    
except Exception as e:
    print(f"❌ Error abriendo puerto: {e}")
    sys.exit()

while True:
    try:
        # 1. Leer buffer
        if ser.in_waiting > 0:
            datos = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
            
            if "PING" in datos:
                print(f"📥 ESP32 dijo: {datos}")
                
                # 2. Responder
                respuesta = b"PONG"
                ser.write(respuesta)
                print(f"➡ PC Responde: PONG")
                print("-" * 20)
            else:
                # Si llega basura, la mostramos igual para debug
                print(f"⚠️ Datos extraños: {datos}")

        time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n👋 Cerrando mini proyecto.")
        ser.close()
        break