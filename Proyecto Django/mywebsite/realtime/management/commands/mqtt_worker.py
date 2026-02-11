from django.core.management.base import BaseCommand
import paho.mqtt.client as mqtt
import ssl

def al_recibir_mensaje(client, userdata, msg):
    print(f"Propiedades del tema: {msg.topic} | Datos recibidos: {msg.payload.decode()}")

def al_conectar(client, userdata, flags, rc, properties=None):
    if rc == 0:
        print("✅ ¡Conectado con éxito al Broker de HiveMQ!")
    else:
        print(f"❌ Error de conexión. Código: {rc}")

class Command(BaseCommand):
    help = 'Lanza el cliente MQTT v5'

    def handle(self, *args, **options):

        cliente = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, protocol=mqtt.MQTTv5)

        cliente.username_pw_set("djangochannels", "Channels12345")

        cliente.tls_set(cert_reqs=ssl.CERT_REQUIRED)

        cliente.on_message = al_recibir_mensaje
        
        self.stdout.write("Conectando al broker...")

        cliente.connect("832b8689599f4045be005c116bc416f0.s1.eu.hivemq.cloud", 8883)

        cliente.subscribe("v1/fiscal/#")

        cliente.on_connect = al_conectar

        cliente.loop_forever()