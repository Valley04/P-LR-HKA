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

def construir_trama(data: bytes) -> bytes:
    lrc = 0
    for b in data:
        lrc ^= b
    lrc ^= ETX
    return bytes([STX]) + data + bytes([ETX, lrc])

def enviar_respuesta_s1(ser):
    payload = (
        b"001000000000000000000000000000000000000000000000000000000000000000000000000000000J3121711970Z1F9999988123000300126"
    )

    cuerpo = payload + bytes([ETX])
    lrc = calcular_lrc(cuerpo)
    frame = bytes([STX]) + cuerpo + bytes([lrc])

    ser.write(frame)
    ser.flush()

    print("➡ Respuesta S1 enviada:", frame.hex(" ").upper())
    print("➡ Longitud trama S1:", len(frame))

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

                # Manejo de ENQ
                if byte == ENQ:
                    print("⬅ ENQ recibido → Enviando STATUS")
                    enviar_status(ser)
                    continue

                # Ignorar ACK/NAK entrantes
                if byte in (ACK, NAK):
                    print(f"⬅ {'ACK' if byte == ACK else 'NAK'} recibido (ignorado)")
                    continue

                # Inicio de trama
                if byte == STX:
                    buffer.clear()
                    buffer.append(byte)
                    print("⬅ STX detectado, iniciando nueva trama")
                    continue

                if buffer:
                    buffer.append(byte)
                    
                    # Si tenemos ETX en posición -2, necesitamos LRC
                    if len(buffer) >= 4 and buffer[-2] == ETX:
                        # Ya tenemos ETX, esperar LRC si no está
                        if len(buffer) == 4:  # STX + DATA + ETX
                            # Necesitamos leer LRC
                            if ser.in_waiting >= 1:
                                lrc_byte = ser.read(1)[0]
                                buffer.append(lrc_byte)
                                
                                trama = bytes(buffer)
                                print(f"✅ Trama completa recibida: {trama.hex(' ').upper()}")
                                print(f"   Longitud: {len(trama)} bytes")
                                
                                # Validar LRC
                                data_etx = trama[1:-1]  # STX hasta LRC (excluido)
                                lrc_rx = trama[-1]
                                lrc_calc = calcular_lrc(data_etx)
                                
                                if lrc_rx == lrc_calc:
                                    print(f"✅ LRC válido: 0x{lrc_rx:02X}")
                                    print("➡ Enviando ACK inmediatamente...")
                                    ser.write(bytes([ACK]))
                                    ser.flush()
                                    print("➡ ACK enviado (0x06)")
                                    
                                    # Procesar comando
                                    comando = data_etx[:-1]  # Excluye ETX
                                    print(f"📥 Comando recibido: {comando}")
                                    
                                    if comando == b"S1":
                                        print("✅ Comando 'S1' reconocido")
                                        print("➡ Enviando respuesta S1...")
                                        enviar_respuesta_s1(ser)
                                    else:
                                        print(f"⚠️ Comando '{comando}' no reconocido")
                                    
                                else:
                                    print(f"❌ LRC inválido: RX=0x{lrc_rx:02X}, CALC=0x{lrc_calc:02X}")
                                    print("➡ Enviando NAK...")
                                    ser.write(bytes([NAK]))
                                    ser.flush()
                                    print("➡ NAK enviado (0x15)")
                                
                                buffer.clear()
                            else:
                                # Esperar LRC
                                continue
                            
            time.sleep(0.01)

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
