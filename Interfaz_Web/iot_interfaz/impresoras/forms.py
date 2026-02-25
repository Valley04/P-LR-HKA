from django import forms
from .models import Dispositivo

class DispositivoForm(forms.ModelForm):
    class Meta:
        model = Dispositivo
        fields = ['serial', 'modelo', 'grupo']

        widgets = {
            'serial': forms.TextInput(attrs={
                'class': 'form-control', 
                'placeholder': 'Ej: Z1F9999988',
                'minlength': '10',
                'maxlength': '10',
                'pattern': '^[A-Z0-9]{10}$',
                'title': 'El serial debe tener exactamente 10 caracteres alfanumericos sin espacios.',
                'required': True
            }),
            'modelo': forms.Select(attrs={'class': 'form-control', 'required': True}),
            'grupo': forms.Select(attrs={'class': 'form-control'}),
        }