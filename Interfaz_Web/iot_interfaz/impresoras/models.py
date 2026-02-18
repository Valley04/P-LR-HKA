from django.db import models

class Grupo(models.Model):
    nombre = models.CharField(max_length=100)

    def __str__(self):
        return self.nombre
    
class Dispositivo(models.Model):
    serial = models.CharField(max_length=50, unique=True)
    modelo = models.CharField(max_length=10, unique=False)
    grupo = models.ForeignKey(Grupo, on_delete=models.SET_NULL,null=True, blank=True)
    pass