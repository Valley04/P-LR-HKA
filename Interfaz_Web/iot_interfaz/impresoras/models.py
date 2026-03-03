import json

from django.db import models
from django.core.validators import RegexValidator

class Grupo(models.Model):
    nombre = models.CharField(max_length=100)
    descripcion = models.TextField(blank=True, null=True)

    def __str__(self):
        return self.nombre
    
class ModeloEquipo(models.Model):
    nombre = models.CharField(max_length=100, unique=True)
    marca = models.CharField(max_length=100, blank=True, null=True)

    def save(self, *args, **kwargs):

        if self.nombre:
            self.nombre = self.nombre.upper()

        if self.marca:
            self.marca = self.marca.upper()

        super().save(*args, **kwargs)

    def __str__(self):

        if self.marca:
            return f"{self.marca} {self.nombre}"
        return self.nombre
    
class ProyectoFirmware(models.Model):
    nombre = models.CharField(max_length=100, unique=True, help_text="Ej: iSmart ESP32 o Impresora Fiscal")
    icono = models.ImageField(upload_to='iconos_fw/', null=True, blank=True, help_text="Sube un icono opcional")
    descripcion = models.TextField(blank=True)
    fecha_creacion = models.DateTimeField(auto_now_add=True)

    def __str__(self):
        return self.nombre
    
class VersionFirmware(models.Model):

    OPCIONES_MODULO = [
        ('ismart', 'Módulo iSmart'),
        ('impresora', 'Impresora Fiscal')
    ]

    proyecto = models.ForeignKey(ProyectoFirmware, on_delete=models.CASCADE, related_name='versiones', null=True)
    tipo_modulo = models.CharField(max_length=20, choices=OPCIONES_MODULO, default='ismart', help_text="¿Para qué hardware es este archivo?")

    validador_version = RegexValidator(
        regex=r'^[A-Za-z0-9]+$', 
        message="La versión solo puede contener letras y números sin espacios (ej. 010000 o 020507GD00).",
        code="version_invalida"
    )

    version = models.CharField(max_length=20, help_text="Ej: v010000")
    archivo_bin = models.FileField(upload_to='firmwares/', help_text="Sube tu archivo .bin compilado en ESP-IDF")
    fecha_subida = models.DateTimeField(auto_now_add=True)
    notas_version = models.TextField(blank=True, help_text="Arreglado el problema con...")
    es_obligatoria = models.BooleanField(default=False, help_text="Fuerza a los equipos a actualizarse")

    def save(self, *args, **kwargs):
        if self.version:
            self.version = self.version.strip().upper()
        super().save(*args, **kwargs)

    class Meta:
        constraints = [
            models.UniqueConstraint(fields=['proyecto', 'version'], name='unique_version_per_project')
        ]

    def __str__(self):
        return f"{self.proyecto.nombre if self.proyecto else 'Sin Proyecto'} - {self.version}"
    
class Dispositivo(models.Model):

    validar_serial = RegexValidator(
        regex=r'^[A-Z0-9]{10}$',
        message="El serial solo puede contener letras mayúsculas y números, sin espacios (ej. Z1F9999988).",
        code="serial_invalido"
    )

    serial = models.CharField(max_length=50, unique=True, validators=[validar_serial], help_text="Ingrese el serial exacto del equipo.")
    modelo = models.ForeignKey(ModeloEquipo, on_delete=models.CASCADE, related_name='dispositivos', null=True, blank=True)
    grupo = models.ForeignKey(Grupo, on_delete=models.SET_NULL,null=True, blank=True)
    fecha_registro = models.DateTimeField(auto_now_add=True)
    ultimo_sts1 = models.IntegerField(null=True, blank=True)
    ultimo_sts2 = models.IntegerField(null=True, blank=True)
    ultima_conexion = models.DateTimeField(null=True, blank=True)
    fw_ismart_instalado = models.CharField(max_length=20, default="Desconocida", blank=True)
    fw_printer_instalado = models.CharField(max_length=20, default="Desconocida", blank=True)
    firmware_actual = models.ForeignKey(VersionFirmware, on_delete=models.SET_NULL, null=True, blank=True, related_name='equipos_instalados')

    def formatear_version_hka(self, v):
        if v and len(v) >= 6 and v[:6].isdigit():
            version_base = f"{v[0:2]}.{v[2:4]}.{v[4:6]}"
            extra = v[6:]
            if extra:
                return f"{version_base} {extra}"
            return version_base
        return v
    
    @property
    def alertas_ota(self):
        from .models import VersionFirmware
        alertas = []
        
        # 1. Leemos los datos directamente del JSON crudo
        datos = self.ultimo_log_datos
        fw_ismart_json = datos.get('fw_ismart', 'Desconocida')
        fw_printer_json = datos.get('fw_printer', 'Desconocida')

        # 2. Evaluamos el iSmart (Buscamos el último .bin etiquetado como 'ismart')
        ultima_v_ismart = VersionFirmware.objects.filter(tipo_modulo='ismart').order_by('-id').first()
        
        if ultima_v_ismart and fw_ismart_json != ultima_v_ismart.version and fw_ismart_json != "Desconocida":
            es_obligatoria = getattr(ultima_v_ismart, 'es_obligatoria', False)
            alertas.append({
                'modulo': 'iSmart',
                'version_nueva': ultima_v_ismart.version,
                'tipo': 'obligatoria' if es_obligatoria else 'opcional'
            })

        # 3. Evaluamos la Impresora (Buscamos el último .bin etiquetado como 'impresora')
        ultima_v_printer = VersionFirmware.objects.filter(tipo_modulo='impresora').order_by('-id').first()
        
        if ultima_v_printer and fw_printer_json != ultima_v_printer.version and fw_printer_json != "Desconocida":
            es_obligatoria = getattr(ultima_v_printer, 'es_obligatoria', False)
            alertas.append({
                'modulo': 'Impresora Fiscal',
                'version_nueva': ultima_v_printer.version,
                'tipo': 'obligatoria' if es_obligatoria else 'opcional'
            })
            
        return alertas
    
    @property
    def ultimo_log_datos(self):

        ultimo_registro = self.logs.first()

        if ultimo_registro and ultimo_registro.detalles:
            try:
                return json.loads(ultimo_registro.detalles)
            except json.JSONDecodeError:
                return {}
        return {}

    @property
    def ismart_display(self):
        return self.formatear_version_hka(self.fw_ismart_instalado)

    @property
    def printer_display(self):
        return self.formatear_version_hka(self.fw_printer_instalado)

    def save(self, *args, **kwargs):
        if self.serial:
            self.serial = self.serial.strip().upper()
        super().save(*args, **kwargs)
    
    def __str__(self):
        return self.serial
    
class LogDispositivo(models.Model):
    dispositivo = models.ForeignKey(Dispositivo, on_delete=models.CASCADE, related_name='logs')

    evento = models.CharField(max_length=100)

    detalles = models.TextField()

    fecha = models.DateTimeField(auto_now_add=True)

    class Meta:
        ordering = ['-fecha']
    
    def __str__(self):
        return f"{self.dispositivo.serial} - {self.evento} ({self.fecha})"
