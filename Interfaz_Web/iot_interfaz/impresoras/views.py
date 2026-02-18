from django.contrib.auth.views import LoginView
from django.shortcuts import render, redirect
from django.contrib.auth.decorators import login_required
from impresoras.forms import DispositivoForm
from .models import Dispositivo
from django.db.models import Q

class MiLogin(LoginView):
    template_name = "login.html"

@login_required # Solo para usuarios loggeados
def dashboard_view(request):
    return render(request, 'dashboard.html')

@login_required
def lista_dispositivos(request):

    query = request.GET.get('buscar')

    dispositivos = Dispositivo.objects.all()

    if query:
        dispositivos = dispositivos.filter(
            Q(serial__icontains=query) | Q(modelo__icontains=query)
        )

    if request.method == 'POST':

        form = DispositivoForm(request.POST)
        if form.is_valid():
            form.save() 
            return redirect('dispositivos_lista')
    else:
        form = DispositivoForm()    
    
    context = {
        'dispositivos': dispositivos,
        'form': form,
    }
    return render(request, 'lista_dispositivos.html', context)