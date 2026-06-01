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
        self.stdout.write(self.style.SUCCESS('[OK] Iniciando Worker MQTT...'))

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
            self.stdout.write(self.style.SUCCESS('Conectado al Broker MQTT'))
            client.subscribe("v1/fiscal/+/json_completo")
            client.subscribe("v1/fiscal/+/alertas_conexion")
        else:
            self.stdout.write(self.style.ERROR(f"Error al conectar, código: {rc}"))

    def on_message(self, client, userdata, msg):
        try:
            topic_parts = msg.topic.split('/')
            serial_msg = topic_parts[2]
            tipo_mensaje = topic_parts[3]
            raw_payload = msg.payload.decode('utf-8', errors='ignore').replace('\n', '').replace('\r', '').replace('\t', '')
            payload = json.loads(raw_payload, strict=False)

            # Buscar equipo
            equipo = Dispositivo.objects.filter(serial=serial_msg).first()

            if  not equipo:
                self.stdout.write(self.style.WARNING(f"Equipo '{serial_msg}' no encontrado."))
                return
            
            if tipo_mensaje == "alertas_conexion":
                if payload.get("st") == "offline":
                    equipo.ota_en_curso = False
                    equipo.save()
                    
                    # Rescatamos los datos del equipo
                    datos_viejos = equipo.ultimo_log_datos if isinstance(equipo.ultimo_log_datos, dict) else {}
                    payload_guardar = datos_viejos.copy()
                    
                    payload_guardar.update({
                        "st": "offline",
                        "alerta_lwt": payload.get("alerta", "conexion_perdida_abruptamente"),
                        "fecha_desconexion": timezone.localtime().strftime("%d/%m/%Y %I:%M:%S %p"),
                        "fw_ismart": equipo.fw_ismart_instalado or "Desconocida",
                        "fw_printer": equipo.fw_printer_instalado or "Desconocida"
                    })
                    
                    LogDispositivo.objects.create(
                        dispositivo=equipo,
                        evento="Desconexión Abrupta (LWT)",
                        detalles=json.dumps(payload_guardar)
                    )
                    self.stdout.write(self.style.ERROR(f"[ALERTA] {serial_msg} se ha desconectado (LWT)"))
                return # Terminamos aquí, no hay contadores que actualizar

            elif tipo_mensaje == "json_completo":

                if isinstance(equipo.ultimo_log_datos, str):
                    try:
                        datos_viejos = json.loads(equipo.ultimo_log_datos)
                    except:
                        datos_viejos = {}
                else:
                    datos_viejos = equipo.ultimo_log_datos or {}

                viejo_fw_ismart = str(equipo.fw_ismart_instalado or "Desconocida")
                viejo_fw_printer = str(equipo.fw_printer_instalado or "Desconocida")

                viejo_st = str(datos_viejos.get('st', 'online')).strip()
                viejo_sts1 = str(datos_viejos.get('sts1', '')).strip()
                viejo_sts2 = str(datos_viejos.get('sts2', '')).strip()
                viejo_z_cnt = str(datos_viejos.get('z_cnt', '')).strip()
                viejo_f_cnt = str(datos_viejos.get('f_cnt', '')).strip()

                nuevo_sts1 = str(payload.get('sts1', '')).strip()
                nuevo_sts2 = str(payload.get('sts2', '')).strip()
                nuevo_z_cnt = str(payload.get('z_cnt', '')).strip()
                nuevo_f_cnt = str(payload.get('f_cnt', '')).strip()
                nuevo_fw_ismart = str(payload.get('fw_ismart', viejo_fw_ismart))
                nuevo_fw_printer = str(payload.get('fw_printer', viejo_fw_printer))

                eventos_detectados = []

                if viejo_st == "offline" and payload.get('st') != "offline":
                    eventos_detectados.append("Reconexión Exitosa")
                
                if nuevo_fw_ismart != viejo_fw_ismart and viejo_fw_ismart not in ["Desconocida", "None"]:
                    eventos_detectados.append("Actualización Firmware iSmart")
                
                if nuevo_fw_printer != viejo_fw_printer and viejo_fw_printer not in ["Desconocida", "None"]:
                    eventos_detectados.append("Actualización Firmware Impresora")
                
                if nuevo_sts2 and nuevo_sts2 != viejo_sts2:
                    eventos_detectados.append(f"Cambio de Error (STS2: {viejo_sts2} -> {nuevo_sts2})")
                
                if nuevo_sts1 and nuevo_sts1 != viejo_sts1:
                    eventos_detectados.append(f"Cambio de Estado (STS1: {viejo_sts1} -> {nuevo_sts1})")
                
                if nuevo_z_cnt and nuevo_z_cnt != viejo_z_cnt:
                    eventos_detectados.append(f"Corte Z Generado (Total: {nuevo_z_cnt})")
                    
                if nuevo_f_cnt and nuevo_f_cnt != viejo_f_cnt:
                    eventos_detectados.append(f"Factura Emitida (Total: {nuevo_f_cnt})")

                if not datos_viejos:
                    eventos_detectados.append("Primer Registro Inicial")

                equipo.ultima_conexion = timezone.now()
                if payload.get('fw_ismart'):
                    equipo.fw_ismart_instalado = str(payload['fw_ismart'])
                if payload.get('fw_printer'):
                    equipo.fw_printer_instalado = str(payload['fw_printer'])

                equipo.save()

                # Guardamos un Log INDIVIDUAL por cada cambio detectado en la misma trama
                for evento_tipo in eventos_detectados:
                    LogDispositivo.objects.create(
                        dispositivo=equipo,
                        evento=evento_tipo,
                        detalles=json.dumps(payload)
                    )
                    self.stdout.write(self.style.SUCCESS(f"[NUEVO LOG] {evento_tipo} en {serial_msg}"))

                # Limpieza de logs antiguos (se mantiene igual)
                fecha_limite = timezone.now() - timedelta(days=90)
                logs_viejos = LogDispositivo.objects.filter(fecha__lt=fecha_limite)
                cantidad_borrada = logs_viejos.count()

                if cantidad_borrada > 0:
                    logs_viejos.delete()
                    self.stdout.write(self.style.WARNING(f"Eliminados {cantidad_borrada} logs antiguos."))

        except Exception as e:
            self.stdout.write(self.style.ERROR(f"Error procesando mensaje: {e}"))