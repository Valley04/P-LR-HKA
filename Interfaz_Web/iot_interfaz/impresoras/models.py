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

    proyecto = models.ForeignKey(ProyectoFirmware, on_delete=models.CASCADE, related_name='versiones', null=True)

    validador_version = RegexValidator(
        regex=r'^[vV]?\d+(\.\d+)+$', 
        message="La versión debe tener un formato válido (ej. 1.0, v1.0.5). No se permiten letras sueltas ni espacios.",
        code="version_invalida"
    )

    version = models.CharField(max_length=20, validators=[validador_version], help_text="Ej: v1.0.0")
    archivo_bin = models.FileField(upload_to='firmwares/', help_text="Sube tu archivo .bin compilado en ESP-IDF")
    fecha_subida = models.DateTimeField(auto_now_add=True)
    notas_version = models.TextField(blank=True, help_text="Arreglado el problema con...")
    es_obligatoria = models.BooleanField(default=False, help_text="Fuerza a los equipos a actualizarse")

    def save(self, *args, **kwargs):
        if self.version:
            version_limpia = self.version.strip()
            self.version = version_limpia.lstrip('vV')  # Elimina cualquier 'v' o 'V' al inicio
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
