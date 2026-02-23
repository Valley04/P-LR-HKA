import json
import ssl
from datetime import timedelta
from django.utils import timezone
import paho.mqtt.client as mqtt
from django.core.management.base import BaseCommand
from impresoras.models import Dispositivo, LogDispositivo

class Command(BaseCommand):
    help = 'Inicia el worker que escucha mensajes MQTT y los guarda en la BD'

    def handle(self, *args, **options):
        self.stdout.write(self.style.SUCCESS('🚀 Iniciando Worker MQTT...'))

        # Configuración del Cliente
        client = mqtt.Client()
        client.on_connect = self.on_connect
        client.on_message = self.on_message
        client.tls_set(cert_reqs=ssl.CERT_REQUIRED, tls_version=ssl.PROTOCOL_TLSv1_2)
        client.username_pw_set("workerpython", "Worker12")

        # Conexión (Ajusta con tus datos del Broker)
        try:
            client.connect("832b8689599f4045be005c116bc416f0.s1.eu.hivemq.cloud", 8883, 60)
            client.loop_forever()
        except KeyboardInterrupt:
            self.stdout.write(self.style.WARNING('\nWorker detenido manualmente.'))
        except Exception as e:
            self.stdout.write(self.style.ERROR(f"Error de conexión: {e}"))

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.stdout.write(self.style.SUCCESS('✅ Conectado al Broker MQTT'))
            client.subscribe("v1/fiscal/+/json_completo")
        else:
            self.stdout.write(self.style.ERROR(f"Error al conectar, código: {rc}"))

    def on_message(self, client, userdata, msg):
        try:
            # Extraer serial y parsear mensaje
            topic_parts = msg.topic.split('/')
            serial_msg = topic_parts[2]
            payload = json.loads(msg.payload.decode('utf-8'))

            nuevo_sts1 = int(payload.get('sts1', 0))
            nuevo_sts2 = int(payload.get('sts2', 0))

            # Buscar equipo y guardar log
            equipo = Dispositivo.objects.filter(serial=serial_msg).first()

            if equipo:

                evento_tipo = None
                equipo.ultima_conexion = timezone.now()
                equipo.save(update_fields=['ultima_conexion'])

                if nuevo_sts2 != equipo.ultimo_sts2:
                    evento_tipo = "Cambio de Error"
                elif nuevo_sts1 != equipo.ultimo_sts1:
                    evento_tipo = "Cambio de Estado"

                if evento_tipo:

                    LogDispositivo.objects.create(
                            dispositivo=equipo,
                            evento=evento_tipo,
                            detalles=json.dumps(payload)
                        )

                    equipo.ultimo_sts1 = nuevo_sts1
                    equipo.ultimo_sts2 = nuevo_sts2
                    equipo.save()

                    fecha_limite = timezone.now() - timedelta(days=90)
                    logs_viejos = LogDispositivo.objects.filter(fecha__lt=fecha_limite)
                    cantidad_borrada = logs_viejos.count()

                    if cantidad_borrada > 0:
                        logs_viejos.delete()
                        self.stdout.write(self.style.WARNING(f"🧹 Eliminados {cantidad_borrada} logs antiguos."))

                    self.stdout.write(f"📝 [NUEVO LOG] Serial: {serial_msg}")
            else:
                self.stdout.write(self.style.WARNING(f"⚠️ Equipo con serial '{serial_msg}' no encontrado en la base de datos."))

        except Exception as e:
            self.stdout.write(self.style.ERROR(f"❌ Error procesando mensaje: {e}"))