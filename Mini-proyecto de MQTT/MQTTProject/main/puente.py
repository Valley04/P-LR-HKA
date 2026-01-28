import paho.mqtt.client as mqtt
import pandas as pd
from datetime import datetime
import ssl

datos_acumulados = []

def on_connect(client, userdata, flags, reason_code, properties):
    reason_codes = {
        0: "Success - Conexión exitosa",
        128: "Unspecified error - Error no especificado",
        134: "Bad User Name or Password - Usuario o contraseña incorrectos",
        135: "Not authorized - No autorizado",
    }

    rc_value = reason_code.value
    mensaje = reason_codes.get(rc_value, f"Código desconocido: {rc_value}")
    print(f"Conectado con reason code: {rc_value} - {mensaje}")

    if properties:
        print("Propiedades MQTT v5 recibidas:")
        if hasattr(properties, 'SessionExpiryInterval'):
            print(f"  SessionExpiryInterval: {properties.SessionExpiryInterval}s")

    if rc_value == 0:
        client.subscribe("pruebahka/lorena555")
        print("Suscrito a 'pruebahka/lorena555'")
    else:
        print(f"No suscrito por error: {mensaje}")


def on_message(client, userdata, msg):
    try:
        valor = msg.payload.decode("utf-8")

        propiedades_texto = []

        if hasattr(msg, 'properties'):
            props = msg.properties
            propiedades_texto.append(f"QoS: {msg.qos}")
            
            # Propiedades comunes en v5
            if hasattr(props, 'MessageExpiryInterval'):
                expiracion = datetime.now().timestamp() + props.MessageExpiryInterval
                expiracion_fmt = datetime.fromtimestamp(expiracion).strftime('%H:%M:%S')
                propiedades_texto.append(f"Expira: {expiracion_fmt}")
            
            if hasattr(props, 'ContentType'):
                propiedades_texto.append(f"Tipo: {props.ContentType}")
            
            # User Properties (metadatos personalizados)
            if hasattr(props, 'UserProperty'):
                for key, value in props.UserProperty:
                    propiedades_texto.append(f"{key}: {value}")

        dato_completo = {
            "timestamp": datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            "topic": msg.topic,
            "valor": valor,
            "propiedades": "; ".join(propiedades_texto) if propiedades_texto else "Ninguna"
        }

        datos_acumulados.append(dato_completo)

        if len(datos_acumulados) >= 10:
            df = pd.DataFrame(datos_acumulados)
            df.to_csv("datos_mqtt_v5.csv", index=False)
            print(f"Datos guardados en datos_mqtt.csv - {len(datos_acumulados)} registros.")
            datos_acumulados.clear()

        # Mostrar información
        print(f"\n📨 [{datetime.now().strftime('%H:%M:%S')}] Topic: {msg.topic}")
        print(f"   Payload: {valor[:100]}{'...' if len(valor) > 100 else ''}")
    except Exception as e:
        print(f"Error al procesar el mensaje: {e}")

try:
    print("Iniciando cliente MQTT v5...")
    mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, 
                        protocol=mqtt.MQTTv5, 
                        client_id=f"python_logger_{datetime.now().strftime('%H%M%S')}"
            )
    
    # Configuracion de TTLS
    mqttc.tls_set(tls_version=ssl.PROTOCOL_TLS)
    mqttc.tls_insecure_set(True)

    # Usuario para HiveMQ Cloud
    mqttc.username_pw_set("scrippython", "Python123")

    # Iniciar MQTT v5
    mqttc.on_connect = on_connect
    mqttc.on_message = on_message

    connect_propierties = mqtt.Properties(mqtt.PacketTypes.CONNECT)

    print("Conectando al broker HiveMQ Cloud...")

    mqttc.connect(
        "832b8689599f4045be005c116bc416f0.s1.eu.hivemq.cloud",
        8883,
        60,
        properties=connect_propierties
    )

    print("Conexión iniciada, esperando mensajes...")
    mqttc.loop_forever()

except Exception as e:
    print(f"Error en el cliente MQTT: {e}")
    import traceback
    traceback.print_exc()