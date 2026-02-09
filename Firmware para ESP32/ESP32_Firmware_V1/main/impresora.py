import serial
import time

ser = serial.Serial(
    port="COM6",
    baudrate=9600,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE, # <--- ¡IMPORTANTE! COINCIDE CON ESP32
    stopbits=serial.STOPBITS_ONE,
    timeout=0.1
)

print("💻 PC Enviando 'HOLA'...")

try:
    while True:
        # 1. Enviar
        ser.write(b"HOLA")
        
        # 2. Leer
        if ser.in_waiting:
            raw = ser.read(ser.in_waiting)
            print(f"📥 PC Recibió: {raw}")
            
        time.sleep(1)
except KeyboardInterrupt:
    ser.close()
