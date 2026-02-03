import serial
import time
import sys

def simulador_depuracion():
    """Simulador con depuración detallada"""
    try:
        ser = serial.Serial(
            port='COM6',
            baudrate=9600,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_EVEN,
            stopbits=serial.STOPBITS_ONE,
            timeout=1
        )
        
        print("=" * 60)
        print("SIMULADOR DE IMPRESORA FISCAL - MODO DEPURACIÓN")
        print("=" * 60)
        print(f"Puerto: {ser.port}")
        print(f"Baudrate: {ser.baudrate}")
        print(f"Bytesize: {ser.bytesize}")
        print(f"Parity: {ser.parity}")
        print(f"Stopbits: {ser.stopbits}")
        print("=" * 60)
        print("Esperando comandos...")
        print("Presiona Ctrl+C para salir")
        print("=" * 60)
        
        contador = 0
        
        while True:
            # Leer datos byte por byte
            if ser.in_waiting > 0:
                byte = ser.read(1)
                contador += 1
                
                print(f"\n[{contador}] Byte recibido:")
                print(f"  Hexadecimal: 0x{byte.hex().upper()}")
                print(f"  Decimal: {int.from_bytes(byte, 'big')}")
                print(f"  Binario: {bin(int.from_bytes(byte, 'big'))[2:].zfill(8)}")
                
                # Intentar interpretar como ASCII
                try:
                    ascii_char = byte.decode('ascii')
                    print(f"  ASCII: '{ascii_char}'")
                except:
                    print(f"  ASCII: No es ASCII válido")
                
                # Mostrar tipo de comando
                if byte == b'\x05':
                    print(f"  TIPO: ENQ (Consulta de estado)")
                elif byte == b'\x4C':
                    print(f"  TIPO: STATUS S1")
                elif byte == b'\x02':
                    print(f"  TIPO: STX (Inicio de trama)")
                elif byte == b'\x03':
                    print(f"  TIPO: ETX (Fin de trama)")
                elif byte == b'\x15':
                    print(f"  TIPO: NAK (No reconocimiento)")
                elif byte == b'\x06':
                    print(f"  TIPO: ACK (Reconocimiento)")
                else:
                    print(f"  TIPO: Desconocido")
                
                # Responder según el byte recibido
                if byte == b'\x05':  # ENQ
                    print("  -> Enviando respuesta ENQ: 0x02 0x60 0x40 0x03")
                    respuesta = b'\x02\x61\x40\x03'
                    ser.write(respuesta)
                    ser.flush()
                    
                elif byte == b'\x4C':  # STATUS S1
                    print("  -> Enviando respuesta STATUS S1")
                    # Construir datos de prueba
                    datos = "001000000000000000000000000000000000000000000000000000000000000000000000000000000J3121711970Z1F9999988123000300126"
                    
                    # Enviar STX + datos + ETX
                    respuesta = b'\x02' + datos.encode('ascii') + b'\x03'
                    print(f"  -> Longitud respuesta: {len(respuesta)} bytes")
                    print(f"  -> STX: 0x02, ETX: 0x03")
                    print(f"  -> Datos ASCII: {len(datos)} caracteres")
                    
                    ser.write(respuesta)
                    ser.flush()
                    
                else:
                    print(f"  -> Byte desconocido, NO respondiendo")
                    # NO enviar NAK para evitar el bucle infinito
                    # ser.write(b'\x15')
                
                print("-" * 40)
            
            time.sleep(0.01)
            
    except KeyboardInterrupt:
        print("\n\nSimulador detenido por usuario")
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Puerto serial cerrado")

if __name__ == "__main__":
    simulador_depuracion()