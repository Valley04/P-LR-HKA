from django.db import models

class Grupo(models.Model):
    nombre = models.CharField(max_length=100)
    descripcion = models.TextField(blank=True, null=True)

    def __str__(self):
        return self.nombre
    
class ModeloEquipo(models.Model):
    nombre = models.CharField(max_length=100, unique=True)
    marca = models.CharField(max_length=100, blank=True, null=True)

    def __str__(self):
        return self.nombre
    
class VersionFirmware(models.Model):
    version = models.CharField(max_length=20, unique=True, help_text="Ej. v1.0.0")
    archivo_bin = models.FileField(upload_to='firmwares/', help_text="Sube tu archivo .bin compilado en ESP-IDF")
    fecha_subida = models.DateTimeField(auto_now_add=True)
    notas_version = models.TextField(blank=True, help_text="Cambios realizados en esta versión en C")
    es_obligatoria = models.BooleanField(default=False, help_text="Fuerza a los equipos a actualizarse")

    def __str__(self):
        return f"Firmware {self.version}"
    
class Dispositivo(models.Model):
    serial = models.CharField(max_length=50, unique=True)
    modelo = models.ForeignKey(ModeloEquipo, on_delete=models.CASCADE, related_name='dispositivos', null=True, blank=True)
    grupo = models.ForeignKey(Grupo, on_delete=models.SET_NULL,null=True, blank=True)
    fecha_registro = models.DateTimeField(auto_now_add=True)
    ultimo_sts1 = models.IntegerField(null=True, blank=True)
    ultimo_sts2 = models.IntegerField(null=True, blank=True)
    ultima_conexion = models.DateTimeField(null=True, blank=True)
    firmware_actual = models.ForeignKey(
        VersionFirmware, 
        on_delete=models.SET_NULL, 
        null=True, 
        blank=True, 
        related_name='equipos_instalados'
    )
    
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
