# listen_mqtt.c
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

            # Buscar equipo
            equipo = Dispositivo.objects.filter(serial=serial_msg).first()

            if equipo:

                viejo_fw_ismart = equipo.fw_ismart_instalado
                viejo_fw_printer = equipo.fw_printer_instalado

                # Actualizamos los datos vitales del equipo
                equipo.ultima_conexion = timezone.now()

                nuevo_fw_ismart = payload.get('fw_ismart')
                nuevo_fw_printer = payload.get('fw_printer')

                if payload.get('fw_ismart'):
                    equipo.fw_ismart_instalado = str(nuevo_fw_ismart)
                if payload.get('fw_printer'):
                    equipo.fw_printer_instalado = str(nuevo_fw_printer)

                equipo.save()                

                # Obtenemos el diccionario con el último historial guardado
                datos_viejos = equipo.ultimo_log_datos

                # Comparamos los valores (usamos str() para asegurar que "60" coincida con "60")
                nuevo_sts1 = str(payload.get('sts1', ''))
                viejo_sts1 = str(datos_viejos.get('sts1', ''))

                nuevo_sts2 = str(payload.get('sts2', ''))
                viejo_sts2 = str(datos_viejos.get('sts2', ''))

                nuevo_vts = str(payload.get('vts', ''))
                viejo_vts = str(datos_viejos.get('vts', ''))

                evento_tipo = None

                if nuevo_fw_ismart and str(nuevo_fw_ismart) != viejo_fw_ismart and viejo_fw_ismart != "Desconocida":
                    evento_tipo = "Actualización Firmware iSmart"
                elif nuevo_fw_printer and str(nuevo_fw_printer) != viejo_fw_printer and viejo_fw_printer != "Desconocida":
                    evento_tipo = "Actualización Firmware Impresora"
                elif nuevo_sts2 != viejo_sts2 and viejo_sts2 != '':
                    evento_tipo = "Cambio de Error"
                elif nuevo_sts1 != viejo_sts1 and viejo_sts1 != '':
                    evento_tipo = "Cambio de Estado"
                elif nuevo_vts != viejo_vts:
                    evento_tipo = "Actualización de Contadores"
                elif not datos_viejos:
                    evento_tipo = "Primer Registro Inicial" # Si es un equipo nuevo

                # 5. Si hubo algún cambio, guardamos el JSON en el historial
                if evento_tipo:

                    LogDispositivo.objects.create(
                        dispositivo=equipo,
                        evento=evento_tipo,
                        detalles=json.dumps(payload)
                    )

                    self.stdout.write(self.style.SUCCESS(f"📝 [NUEVO LOG] {evento_tipo} en {serial_msg}"))

                # 6. Limpieza de logs antiguos (se mantiene igual)
                fecha_limite = timezone.now() - timedelta(days=90)
                logs_viejos = LogDispositivo.objects.filter(fecha__lt=fecha_limite)
                cantidad_borrada = logs_viejos.count()

                if cantidad_borrada > 0:
                    logs_viejos.delete()
                    self.stdout.write(self.style.WARNING(f"🧹 Eliminados {cantidad_borrada} logs antiguos."))

            else:
                self.stdout.write(self.style.WARNING(f"⚠️ Equipo con serial '{serial_msg}' no encontrado."))

        except Exception as e:
            self.stdout.write(self.style.ERROR(f"❌ Error procesando mensaje: {e}"))