import serial
import time
import sys

# --- CONFIGURACIÓN ---
PUERTO_PC = "COM4"  # <--- ¡VERIFICAR!
BAUDIOS = 9600

# Control bytes
STX = 0x02
ETX = 0x03
ENQ = 0x05
ACK = 0x06
NAK = 0x15

# Estado fiscal simulado
STS1 = 0x60
STS2 = 0x40

def calcular_lrc(data: bytes) -> int:
    lrc = 0
    for b in data:
        lrc ^= b
    return lrc

def enviar_status(ser):
    cuerpo = bytes([STS1, STS2, ETX])
    lrc = calcular_lrc(cuerpo)
    frame = bytes([STX]) + cuerpo + bytes([lrc])
    
    ser.write(frame)
    ser.flush()
    
    ser.read(len(frame)) 
    
    print(f"➡ STATUS enviado (LRC: {lrc:02X})")

def enviar_respuesta_s1(ser):
    # Serial simulado: Z1F9999988
    campos = [
        b"S100",                # 0: ATM (Cajero)
        b"00000000019849029",   # 1: Ventas
        b"00000000",            # 2: Última factura
        b"00000",               # 3: Emisión
        b"00000000",            # 4: Último ND
        b"00000",               # 5: Monto ND
        b"00000000",            # 6: Último NC
        b"00000",               # 7: Monto NC
        b"00000000",            # 8: Último NF
        b"00000",               # 9: Monto NF
        b"0020",                # 10: Z_cnt
        b"0017",                # 11: F_cnt
        b"J-312171197",         # 12: RIF
        b"Z1F9999988",          # 13: Serial
        b"123000",              # 14: Hora
        b"300126"               # 15: Fecha
    ]

    payload = b"".join([campo + b"\x0A" for campo in campos])

    cuerpo = payload + bytes([ETX])
    lrc = calcular_lrc(cuerpo)
    frame = bytes([STX]) + cuerpo + bytes([lrc])
    
    ser.write(frame)
    ser.flush()

    ser.read(len(frame))

    print(f"➡ Respuesta S1 enviada ({len(frame)} bytes)")
    print(f"   Hex: {frame.hex(' ').upper()}")

def enviar_respuesta_sv2(ser):

    campos = [
        b"Z7C"
        b"VE"
        b"020507GD00"
    ]

    payload = b"SV2\x0A" + b"\x0A".join(campos)

    cuerpo = payload + bytes([ETX])
    lrc = calcular_lrc(cuerpo)
    frame = bytes([STX]) + cuerpo + bytes([lrc])
    
    ser.write(frame)
    ser.flush()
    ser.read(len(frame))

    print(f"➡ Respuesta SV2 enviada ({len(frame)} bytes)")

def procesar_trama_completa(ser, trama):
    print(f"✅ Trama RX: {trama.hex(' ').upper()}")
    
    # Validar LRC
    # Trama = STX [DATA... ETX] LRC
    data_etx = trama[1:-1] 
    lrc_rx = trama[-1]
    lrc_calc = calcular_lrc(data_etx)
    
    if lrc_rx == lrc_calc:
        # 1. Enviar ACK
        ser.write(bytes([ACK]))
        ser.flush()
        print(f"➡ ACK enviado (0x06)")
        
        # 2. Analizar comando
        comando = data_etx[:-1] # Quitar ETX
        try:
            cmd_str = comando.decode('utf-8', errors='ignore')
        except:
            cmd_str = "?"
            
        print(f"📥 Comando: {cmd_str} ({comando.hex()})")
        
        if cmd_str == "S1":
            print("   ↳ Solicitud de Reporte S1 detectada")
            time.sleep(0.2) # Simular tiempo de procesamiento de la impresora real
            enviar_respuesta_s1(ser)
        elif cmd_str == "SV2":
            print("   ↳ Solicitud de Reporte SV2 detectada")
            time.sleep(0.2)
            enviar_respuesta_sv2(ser)
        else:
            print(f"   ↳ Comando '{cmd_str}' no implementado en simulador")
            
    else:
        print(f"❌ Error de LRC (RX: {lrc_rx:02X} != CALC: {lrc_calc:02X})")
        ser.write(bytes([NAK]))
        ser.flush()
        print("➡ NAK enviado")

def simulador_fiscal():
    try:
        ser = serial.Serial(
            port=PUERTO_PC,
            baudrate=BAUDIOS,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_EVEN,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1 # Timeout corto para no bloquear el loop
        )
    except serial.SerialException as e:
        print(f"❌ No se pudo abrir el puerto {PUERTO_PC}: {e}")
        return

    buffer = bytearray()
    estado = "ESPERANDO_STX" 
    
    print(f"\n🖨️  SIMULADOR FISCAL HKA ACTIVO en {PUERTO_PC}")
    print("   (Presiona Ctrl+C para salir)\n")
    
    ser.reset_input_buffer()

    try:
        while True:
            # Lectura no bloqueante gracias al timeout
            data = ser.read(1) 
            
            if data:
                byte = data[0]
                
                # ENQ: Prioridad absoluta (Polling)
                if byte == ENQ:
                    # Solo logueamos ENQ si NO estamos a mitad de una trama para no ensuciar la pantalla
                    if estado == "ESPERANDO_STX":
                        sys.stdout.write(".") # Indicador visual de "latido"
                        sys.stdout.flush()
                    else:
                        print("\n⚠️ ENQ recibido durante trama")
                    
                    enviar_status(ser)
                    # Resetear buffer si llega un ENQ (protocolo HKA suele resetearse)
                    buffer.clear()
                    estado = "ESPERANDO_STX"
                    continue

                # Máquina de estados
                if estado == "ESPERANDO_STX":
                    if byte == STX:
                        print("\n⬅ STX recibido, capturando trama...")
                        buffer.clear()
                        buffer.append(byte)
                        estado = "RECIBIENDO_DATOS"
                        
                elif estado == "RECIBIENDO_DATOS":
                    buffer.append(byte)
                    
                    # Verificación de fin de trama
                    # STX + CMD + ETX + LRC (Mínimo 4 bytes)
                    if len(buffer) >= 4:
                        if buffer[-2] == ETX:
                            procesar_trama_completa(ser, bytes(buffer))
                            buffer.clear()
                            estado = "ESPERANDO_STX"
                        elif len(buffer) > 256: # Safety break
                            print("❌ Trama demasiado larga, reset.")
                            buffer.clear()
                            estado = "ESPERANDO_STX"

    except KeyboardInterrupt:
        print("\n⏹️ Simulador cerrado.")
    finally:
        if ser.is_open:
            ser.close()

if __name__ == "__main__":
    simulador_fiscal()
