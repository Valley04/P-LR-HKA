from django import forms
from .models import Dispositivo

class DispositivoForm(forms.ModelForm):
    class Meta:
        model = Dispositivo
        fields = ['serial', 'modelo', 'grupo']

        widgets = {
            'serial': forms.TextInput(attrs={'class': 'form-control', 'placeholder': 'Ej: Z1F9999988'}),
            'modelo': forms.TextInput(attrs={'class': 'form-control', 'placeholder': 'Ej: HKA80'}),
            'grupo': forms.Select(attrs={'class': 'form-control'}),
        }