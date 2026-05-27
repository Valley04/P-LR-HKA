from django import forms
from .models import Dispositivo

class DispositivoForm(forms.ModelForm):
    class Meta:
        model = Dispositivo
        fields = ['serial', 'modelo', 'grupo']

        # Definimos las clases comunes de Tailwind para mantener consistencia
        input_classes = 'w-full bg-gray-900 border border-gray-600 rounded-xl px-4 py-2.5 text-white focus:ring-2 focus:ring-blue-500 outline-none transition-all'

        widgets = {
            'serial': forms.TextInput(attrs={
                'class': input_classes,
                'placeholder': 'Ej: Z1F9999988',
                'minlength': '10',
                'maxlength': '10',
                'pattern': '^[A-Z0-9]{10}$',
                'title': 'El serial debe tener exactamente 10 caracteres alfanuméricos.',
                'required': True
            }),
            'modelo': forms.Select(attrs={
                'class': input_classes,
                'required': True
            }),
            'grupo': forms.Select(attrs={
                'class': input_classes
            }),
        }