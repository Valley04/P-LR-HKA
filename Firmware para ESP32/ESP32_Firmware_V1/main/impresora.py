import serial
import time

# Control bytes
STX = 0x02
ETX = 0x03
ENQ = 0x05
ACK = 0x06
NAK = 0x15

# Estado fiscal fijo
STS1 = 0x60
STS2 = 0x40

def calcular_lrc(data: bytes) -> int:
    lrc = 0
    for b in data:
        lrc ^= b
    return lrc

def enviar_status(ser):
    frame = bytes([
        STX,
        STS1,
        STS2,
        ETX,
        STS1 ^ STS2 ^ ETX
    ])
    ser.write(frame)
    ser.flush()
    print("➡ STATUS enviado:", frame.hex(" ").upper())

def enviar_respuesta_s1(ser):
    payload = (
        b"001000000000000000000000000000000000000000000000000000000000000000000000000000000J3121711970Z1F9999988123000300126"
    )
    cuerpo = payload + bytes([ETX])
    lrc = calcular_lrc(cuerpo)
    frame = bytes([STX]) + cuerpo + bytes([lrc])
    ser.write(frame)
    ser.flush()
    print("➡ Respuesta S1 enviada")
    print(f"   Longitud: {len(frame)} bytes")

def procesar_trama_completa(ser, trama):
    """Procesa una trama completa ya recibida"""
    print(f"✅ Trama completa recibida: {trama.hex(' ').upper()}")
    print(f"   Longitud: {len(trama)} bytes")
    
    # Validar LRC
    data_etx = trama[1:-1]  # STX hasta LRC (excluido)
    lrc_rx = trama[-1]
    lrc_calc = calcular_lrc(data_etx)
    
    print(f"   LRC RX: 0x{lrc_rx:02X}, LRC CALC: 0x{lrc_calc:02X}")
    
    if lrc_rx == lrc_calc:
        print("✅ LRC válido")
        
        # Enviar ACK inmediatamente
        print("➡ Enviando ACK...")
        ser.write(bytes([ACK]))
        ser.flush()
        print("➡ ACK enviado (0x06)")
        
        # Procesar comando
        comando = data_etx[:-1]  # Excluye ETX
        print(f"📥 Comando recibido: {comando}")
        
        if comando == b"S1":
            print("✅ Comando 'S1' reconocido")
            # Pequeña pausa antes de enviar respuesta
            time.sleep(0.1)
            enviar_respuesta_s1(ser)
        else:
            print(f"⚠️ Comando '{comando}' no reconocido, ignorado")
            
    else:
        print(f"❌ LRC inválido")
        print("➡ Enviando NAK...")
        ser.write(bytes([NAK]))
        ser.flush()
        print("➡ NAK enviado (0x15)")

def simulador_fiscal():
    ser = serial.Serial(
        port="COM6",
        baudrate=9600,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_EVEN,
        stopbits=serial.STOPBITS_ONE,
        timeout=1
    )

    buffer = bytearray()
    estado = "ESPERANDO_STX"  # Estados: ESPERANDO_STX, RECIBIENDO_DATOS
    
    print("🖨️ Simulador Fiscal HKA activo...")
    print(f"Conectado a {ser.port} a {ser.baudrate} baudios")
    
    # Limpiar buffers
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    try:
        while True:
            if ser.in_waiting:
                byte = ser.read(1)[0]
                print(f"⬅ Byte recibido: {byte:02X}")

                # Manejo de ENQ (siempre tiene prioridad)
                if byte == ENQ:
                    print("⬅ ENQ recibido → Enviando STATUS")
                    enviar_status(ser)
                    buffer.clear()
                    estado = "ESPERANDO_STX"
                    continue
                
                # Si recibimos STX, iniciar nueva trama
                if byte == STX:
                    buffer.clear()
                    buffer.append(byte)
                    estado = "RECIBIENDO_DATOS"
                    print("⬅ STX detectado, iniciando nueva trama")
                    continue
                
                # Si estamos recibiendo datos
                if estado == "RECIBIENDO_DATOS":
                    buffer.append(byte)
                    
                    # Verificar si tenemos trama completa
                    # Necesitamos: STX + datos + ETX + LRC (mínimo 5 bytes)
                    if len(buffer) >= 5:
                        # Buscar ETX en los últimos bytes
                        if buffer[-2] == ETX:
                            # Tenemos STX...ETX + LRC
                            # La trama está completa
                            procesar_trama_completa(ser, bytes(buffer))
                            buffer.clear()
                            estado = "ESPERANDO_STX"
                        elif len(buffer) > 100:  # Límite de seguridad
                            print("⚠️ Trama demasiado larga, descartando...")
                            buffer.clear()
                            estado = "ESPERANDO_STX"
                
                # Ignorar ACK/NAK entrantes
                elif byte in (ACK, NAK):
                    print(f"⬅ {'ACK' if byte == ACK else 'NAK'} recibido")
                    # No hacer nada, solo log

            time.sleep(0.001)  # Reducir sleep para mejor respuesta

    except KeyboardInterrupt:
        print("\n⏹️ Simulador detenido")
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        ser.close()

if __name__ == "__main__":
    simulador_fiscal()
