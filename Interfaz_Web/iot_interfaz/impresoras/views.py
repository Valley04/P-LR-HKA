from django.contrib.auth.views import LoginView
from django.shortcuts import render, redirect
from django.contrib.auth.decorators import login_required
from impresoras.forms import DispositivoForm
from .models import Dispositivo

class MiLogin(LoginView):
    template_name = "login.html"

@login_required # Solo para usuarios loggeados
def dashboard_view(request):
    return render(request, 'dashboard.html')

@login_required
def lista_dispositivos(request):

    if request.method == 'POST':

        form = DispositivoForm(request.POST)
        if form.is_valid():
            form.save() 
            return redirect('dispositivos_lista')
    else:
        form = DispositivoForm()

    dispositivos = Dispositivo.objects.all()
    
    context = {
        'dispositivos': dispositivos,
        'form': form,
    }
    return render(request, 'lista_dispositivos.html', context)