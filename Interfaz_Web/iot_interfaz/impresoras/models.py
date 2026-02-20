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
    
class Dispositivo(models.Model):
    serial = models.CharField(max_length=50, unique=True)
    modelo = models.ForeignKey(ModeloEquipo, on_delete=models.SET_NULL, null=True)
    grupo = models.ForeignKey(Grupo, on_delete=models.SET_NULL,null=True, blank=True)

    ultimo_sts1 = models.IntegerField(null=True, blank=True)
    ultimo_sts2 = models.IntegerField(null=True, blank=True)
    
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
