from django.contrib.auth.views import LoginView
from django.shortcuts import render, redirect, get_object_or_404
from django.contrib.auth.decorators import login_required
from impresoras.forms import DispositivoForm
from .models import Dispositivo, Grupo, ModeloEquipo
from django.db.models import Q
import openpyxl
from django.http import HttpResponse
from django.core.paginator import Paginator
from .models import LogDispositivo

class MiLogin(LoginView):
    template_name = "login.html"

@login_required
def log_capture_view(request):
    logs_list = LogDispositivo.objects.all().select_related('dispositivo')

    serial_query = request.GET.get('serial')
    fecha_inicio = request.GET.get('inicio')
    fecha_fin = request.GET.get('fin')

    if serial_query:
        logs_list = logs_list.filter(dispositivo__serial__icontains=serial_query)

    if fecha_inicio:
        logs_list = logs_list.filter(fecha__date__gte=fecha_inicio)

    if fecha_fin:
        logs_list = logs_list.filter(fecha__date__lte=fecha_fin)

    paginator = Paginator(logs_list, 20)  # 20 logs por página
    page_number = request.GET.get('page')
    page_obj = paginator.get_page(page_number)

    return render(request, 'log_capture.html', {
        'page_obj': page_obj, 
    })

@login_required
def exportar_dispositivos_excel(request):
    # Crear un libro de Excel
    workbook = openpyxl.Workbook()
    sheet = workbook.active
    sheet.title = "Dispositivos"

    # Escribir encabezados
    headers = ['Serial', 'Modelo', 'Marca', 'Grupo']
    sheet.append(headers)

    query = request.GET.get('buscar')
    dispositivos = Dispositivo.objects.all()

    if query:
        dispositivos = dispositivos.filter(
            Q(serial__icontains=query) | Q(modelo__nombre__icontains=query)
        )
    # Escribir datos de los dispositivos
    for dispositivo in dispositivos:
        sheet.append([
            dispositivo.serial,
            dispositivo.modelo.nombre if dispositivo.modelo else 'N/A',
            dispositivo.modelo.marca if dispositivo.modelo else 'N/A',
            dispositivo.grupo.nombre if dispositivo.grupo else 'Sin grupo'
        ])

    # Guardar el libro en una respuesta HTTP
    response = HttpResponse(content_type='application/vnd.openxmlformats-officedocument.spreadsheetml.sheet')
    response['Content-Disposition'] = 'attachment; filename=dispositivos.xlsx'
    workbook.save(response)
    
    return response

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

    grupos = Grupo.objects.all()
    modelos = ModeloEquipo.objects.all()
    
    context = {
        'dispositivos': dispositivos,
        'form': form,
        'grupos': grupos,
        'modelos': modelos
    }
    return render(request, 'lista_dispositivos.html', context)

@login_required
def editar_dispositivo(request, id):
    equipo = get_object_or_404(Dispositivo, id=id)
    
    if request.method == 'POST':
        # Extraemos directamente lo que el usuario escribió en el Modal
        nuevo_serial = request.POST.get('serial')
        modelo_id = request.POST.get('modelo_id')
        grupo_id = request.POST.get('grupo_id')
        
        # Actualizamos el equipo
        if nuevo_serial: equipo.serial = nuevo_serial
        if modelo_id: equipo.modelo_id = modelo_id
        if grupo_id: equipo.grupo_id = grupo_id
        else:
            equipo.grupo = None
        
        # Guardamos los cambios a la fuerza en la base de datos
        equipo.save()
        
    # Recargamos la página
    return redirect('dispositivos_lista')

@login_required
def eliminar_dispositivo(request, id):
    equipo = get_object_or_404(Dispositivo, id=id)
    if request.method == 'POST':
        equipo.delete()
    return redirect('dispositivos_lista')

@login_required
def grupos_modelos(request):
    if request.method == 'POST':
        tipo = request.POST.get('tipo')
        
        # Si el formulario que se envió fue el de Grupo...
        if tipo == 'grupo':
            nombre = request.POST.get('nombre')
            descripcion = request.POST.get('descripcion')
            if nombre:
                Grupo.objects.create(nombre=nombre, descripcion=descripcion)
                
        # Si el formulario que se envió fue el de Modelo...
        elif tipo == 'modelo':
            nombre = request.POST.get('nombre')
            marca = request.POST.get('marca')
            if nombre:
                ModeloEquipo.objects.create(nombre=nombre, marca=marca)
                
        return redirect('grupos_modelos')

    # Traemos todos los datos para mostrarlos en las tablas
    grupos = Grupo.objects.all()
    modelos = ModeloEquipo.objects.all()
    
    return render(request, 'grupos_modelos.html', {
        'grupos': grupos,
        'modelos': modelos
    })

@login_required
def eliminar_grupo(request, id):
    if request.method == 'POST':
        grupo = get_object_or_404(Grupo, id=id)
        grupo.delete()
    return redirect('grupos_modelos')

@login_required
def eliminar_modelo(request, id):
    if request.method == 'POST':
        modelo = get_object_or_404(ModeloEquipo, id=id)
        modelo.delete()
    return redirect('grupos_modelos')