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
        client = mqtt.Client(protocol=mqtt.MQTTv5)
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

    def on_connect(self, client, userdata, flags, rc, properties=None):
        if rc == 0:
            self.stdout.write(self.style.SUCCESS('✅ Conectado al Broker MQTT'))
            client.subscribe("v1/fiscal/+/json_completo")
            client.subscribe("v1/fiscal/+/alertas_conexion")
        else:
            self.stdout.write(self.style.ERROR(f"Error al conectar, código: {rc}"))

    def on_message(self, client, userdata, msg):
        try:
            # Extraer serial y parsear mensaje
            topic_parts = msg.topic.split('/')
            serial_msg = topic_parts[2]
            tipo_mensaje = topic_parts[3]
            raw_payload = msg.payload.decode('utf-8', errors='ignore')
            raw_payload = raw_payload.replace('\n', '').replace('\r', '').replace('\t', '')
            payload = json.loads(raw_payload, strict=False)

            # Buscar equipo
            equipo = Dispositivo.objects.filter(serial=serial_msg).first()

            if equipo:

                if tipo_mensaje == "alertas_conexion":
                    if payload.get("st") == "offline":
                        evento_tipo = "Desconexión Abrupta (LWT)"
                        
                        # Rescatamos la última "foto" conocida de los datos del equipo
                        datos_viejos = equipo.ultimo_log_datos if equipo.ultimo_log_datos else {}
                        payload_guardar = datos_viejos.copy()
                        
                        payload_guardar["st"] = "offline"
                        payload_guardar["alerta_lwt"] = payload.get("alerta", "conexion_perdida_abruptamente")
                        payload_guardar["fecha_desconexion"] = timezone.localtime().strftime("%d/%m/%Y %I:%M:%S %p")
                        payload_guardar["fw_ismart"] = equipo.fw_ismart_instalado or "Desconocida"
                        payload_guardar["fw_printer"] = equipo.fw_printer_instalado or "Desconocida"
                        
                        LogDispositivo.objects.create(
                            dispositivo=equipo,
                            evento=evento_tipo,
                            detalles=json.dumps(payload_guardar)
                        )
                        self.stdout.write(self.style.ERROR(f"🔴 [ALERTA] {serial_msg} se ha desconectado (LWT)"))
                    return # Terminamos aquí, no hay contadores que actualizar

                elif tipo_mensaje == "json_completo":

                    viejo_fw_ismart = str(equipo.fw_ismart_instalado)
                    viejo_fw_printer = str(equipo.fw_printer_instalado)

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
                    datos_viejos = equipo.ultimo_log_datos if equipo.ultimo_log_datos else {}

                    viejo_st = datos_viejos.get('st', 'online')

                    nuevo_sts1 = str(payload.get('sts1', ''))
                    viejo_sts1 = str(datos_viejos.get('sts1', ''))

                    nuevo_sts2 = str(payload.get('sts2', ''))
                    viejo_sts2 = str(datos_viejos.get('sts2', ''))

                    eventos_detectados = []

                    # 1. Evaluaciones Independientes (Puros 'if')
                    if viejo_st == "offline":
                        eventos_detectados.append("Reconexión Exitosa")
                    
                    if nuevo_fw_ismart and str(nuevo_fw_ismart) != viejo_fw_ismart and viejo_fw_ismart not in ["Desconocida", "None"]:
                        eventos_detectados.append("Actualización Firmware iSmart")
                    
                    if nuevo_fw_printer and str(nuevo_fw_printer) != viejo_fw_printer and viejo_fw_printer not in ["Desconocida", "None"]:
                        eventos_detectados.append("Actualización Firmware Impresora")
                    
                    if nuevo_sts2 and nuevo_sts2 != viejo_sts2:
                        eventos_detectados.append("Cambio de Error")
                    
                    if nuevo_sts1 and nuevo_sts1 != viejo_sts1:
                        eventos_detectados.append("Cambio de Estado")
                    
                    if not datos_viejos:
                        eventos_detectados.append("Primer Registro Inicial")

                    # 5. Guardamos un Log INDIVIDUAL por cada cambio detectado en la misma trama
                    for evento_tipo in eventos_detectados:
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