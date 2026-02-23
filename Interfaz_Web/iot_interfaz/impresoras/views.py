from django.contrib.auth.views import LoginView
from django.shortcuts import render, redirect, get_object_or_404
from django.contrib.auth.decorators import login_required
from impresoras.forms import DispositivoForm
from .models import Dispositivo, Grupo, ModeloEquipo
from django.db.models import Q, Count
from django.db.models.functions import TruncMonth
import openpyxl
from datetime import timedelta, datetime
from django.utils import timezone
from django.http import HttpResponse, JsonResponse
from .models import LogDispositivo, Dispositivo, VersionFirmware
import paho.mqtt.client as mqtt
import ssl
import json
from django.contrib import messages

class MiLogin(LoginView):
    template_name = "login.html"

@login_required
def disparar_ota_mqtt(request, dispositivo_id):
    if request.method == 'POST':
        equipo = get_object_or_404(Dispositivo, id=dispositivo_id)
        fw_id = request.POST.get('firmware_id')
        fw_destino = get_object_or_404(VersionFirmware, id=fw_id)

        url_descarga = request.build_absolute_uri(fw_destino.archivo_bin.url)

        payload = {
            "comando": "INICIAR_OTA",
            "version": fw_destino.version,
            "url_descarga": url_descarga,
        }

        topic_comando = f"comandos/{equipo.serial}/ota"

        try:
            BROKER = "832b8689599f4045be005c116bc416f0.s1.eu.hivemq.cloud"
            PORT = 8883
            USER = "workerpython"
            PASS = "Worker12"

            client = mqtt.Client(client_id=f"django_web_cmd_{equipo.serial}")
            client.tls_set(tls_version=ssl.PROTOCOL_TLS)
            client.username_pw_set(USER, PASS)

            client.connect(BROKER, PORT, 10)
            client.publish(topic_comando, json.dumps(payload), qos=1)
            client.disconnect()

            messages.success(request, f"✅ Comando OTA enviado a {equipo.serial} para actualizar a {fw_destino.version}")
        except Exception as e:
            messages.error(request, f"❌ Error al enviar comando OTA: {e}")

    return redirect('dispositivos_lista')

def eliminar_firmware(request, fw_id):
    if request.method == 'POST':
        fw = get_object_or_404(VersionFirmware, id=fw_id)

        if fw.archivo_bin:
            fw.archivo_bin.delete(save=False)
        
        fw.delete()

    return redirect('gestion_firmware')

def api_ultimo_firmware(request):
    ultimo_fw = VersionFirmware.objects.order_by('-fecha_subida').first()

    if not ultimo_fw:
        return JsonResponse({'error': 'No hay firmware disponible'}, status=404)
    
    url_descarga = request.build_absolute_uri(ultimo_fw.archivo_bin.url)
        
    return JsonResponse({
        'version': ultimo_fw.version,
        'archivo_url': url_descarga,
        'es_obligatoria': ultimo_fw.es_obligatoria,
        'notas_version': ultimo_fw.notas_version,        
    })

@login_required
def gestion_firmware_view(request, fw_id=None):

    if request.method == 'POST':
        version = request.POST.get('version')
        archivo = request.FILES.get('archivo_bin')
        notas = request.POST.get('notas_version', '')
        es_obligatoria = request.POST.get('es_obligatoria') == 'on'

        if version and archivo:
            VersionFirmware.objects.create(
                version=version,
                archivo_bin=archivo,
                notas_version=notas,
                es_obligatoria=es_obligatoria
            )
            return redirect('gestion_firmware')

    query = request.GET.get('buscar', '')
    firmwares = VersionFirmware.objects.all().order_by('-fecha_subida')

    if query:
        firmwares = firmwares.filter(version__icontains=query)

    # Determinar que firmware mostrar
    fw_seleccionado = None
    if fw_id:
        fw_seleccionado = get_object_or_404(VersionFirmware, id=fw_id)
    elif firmwares.exists():
        fw_seleccionado = firmwares.first()

    context = {
        'firmwares': firmwares,
        'fw_seleccionado': fw_seleccionado,
        'query': query,
    }
    return render(request, 'gestion_firmware.html', context)

@login_required
def log_capture_view(request):
    query = request.GET.get('buscar')
    
    dispositivos = Dispositivo.objects.all()
    
    if query:
        dispositivos = dispositivos.filter(serial__icontains=query)
        
    return render(request, 'log_capture.html', {'dispositivos': dispositivos})

def descargar_logs_dispositivos(request, dispositivo_id):
    equipo = Dispositivo.objects.get(id=dispositivo_id)
    fecha_limite = timezone.now() - timedelta(days=90)

    logs = LogDispositivo.objects.filter(
        dispositivo=equipo, 
        fecha__gte=fecha_limite
    ).order_by('-fecha')

    wb = openpyxl.Workbook()
    ws = wb.active
    ws.title = f"Logs_{equipo.serial}"
    ws.append(['Fecha/Hora', 'Evento', 'Detalles'])

    for l in logs:
        ws.append([l.fecha.strftime("%d/%m/%Y %H:%M:%S"), l.evento, l.detalles])

    response = HttpResponse(content_type='application/vnd.openxmlformats-officedocument.spreadsheetml.sheet')
    response['Content-Disposition'] = f'attachment; filename="Logs_{equipo.serial}_90dias.xlsx"'
    wb.save(response)
    return response

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
    total_dispositivos = Dispositivo.objects.count()
    hace_2_minutos = timezone.now() - timedelta(seconds=30)
    en_linea = Dispositivo.objects.filter(ultima_conexion__gte=hace_2_minutos).count()
    alertas = Dispositivo.objects.exclude(ultimo_sts2=40).count()

    # Modelos para el grafico de barras
    modelos_data = ModeloEquipo.objects.annotate(cantidad=Count('dispositivos'))
    labels_modelos = [m.nombre for m in modelos_data]
    counts_modelos = [m.cantidad for m in modelos_data]

    # Registro de dispositivos en los ultimos 6 meses
    seis_meses_atras = timezone.now() - timedelta(days=180)
    registro_mensual = Dispositivo.objects.filter(fecha_registro__gte=seis_meses_atras) \
        .annotate(mes=TruncMonth('fecha_registro')) \
        .values('mes') \
        .annotate(cantidad=Count('id')) \
        .order_by('mes')
    labels_meses = [r['mes'].strftime('%b %Y') for r in registro_mensual]
    data_meses = [r['cantidad'] for r in registro_mensual]

    context = {
        'total_dispositivos': total_dispositivos,
        'labels_modelos': labels_modelos,
        'counts_modelos': counts_modelos,
        'labels_meses': labels_meses,
        'data_meses': data_meses,
        'en_linea': en_linea,
        'alertas': alertas
    }
    return render(request, 'dashboard.html', context)

@login_required
def lista_dispositivos(request):

    query = request.GET.get('buscar')
    dispositivos = Dispositivo.objects.all()
    firmwares_disponibles = VersionFirmware.objects.all().order_by('-fecha_subida')

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
        'modelos': modelos,
        'firmwares_disponibles': firmwares_disponibles,
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