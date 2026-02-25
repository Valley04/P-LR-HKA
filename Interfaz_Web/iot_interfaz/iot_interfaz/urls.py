from django.contrib import admin
from django.contrib.auth import views
from django.urls import path
from django.conf import settings
from django.conf.urls.static import static
from impresoras.views import MiLogin, dashboard_view, lista_dispositivos, editar_dispositivo, eliminar_dispositivo  #Importamos views
from impresoras.views import grupos_modelos, eliminar_grupo, eliminar_modelo, exportar_dispositivos_excel, log_capture_view
from impresoras.views import log_capture_view, descargar_logs_dispositivos, gestion_firmware_view, api_ultimo_firmware
from impresoras.views import eliminar_version_firmware, disparar_ota_mqtt, eliminar_proyecto_firmware
from django.contrib.auth.views import LogoutView

urlpatterns = [
    path('admin/', admin.site.urls),
    path('login/', MiLogin.as_view(), name='login'),
    path('dashboard/', dashboard_view, name='dashboard'),
    path('logout/', LogoutView.as_view(), name='logout'),

    # Rutas para administración de dispositivos
    path('dispositivos/', lista_dispositivos, name='dispositivos_lista'),
    path('dispositivos/editar/<int:id>/', editar_dispositivo, name='editar_dispositivo'),
    path('dispositivos/eliminar/<int:id>/', eliminar_dispositivo, name='eliminar_dispositivo'),
    path('dispositivos/exportar/', exportar_dispositivos_excel, name='exportar_excel'),

    # Rutas para Grupos y Modelos
    path('grupos-modelos/', grupos_modelos, name='grupos_modelos'),
    path('grupos/eliminar/<int:id>/', eliminar_grupo, name='eliminar_grupo'),
    path('modelos/eliminar/<int:id>/', eliminar_modelo, name='eliminar_modelo'),

    # Ruta para log
    path('log-capture/', log_capture_view, name='log_capture'),
    path('log-capture/', log_capture_view, name='log_capture'),
    path('log-capture/descargar/<int:dispositivo_id>/', descargar_logs_dispositivos, name='descargar_logs_equipo'),

    # Rutas para Firmware
    path('firmware/', gestion_firmware_view, name='gestion_firmware'),
    path('firmware/proyecto/<int:proyecto_id>/', gestion_firmware_view, name='gestion_firmware_detalle'),
    path('api/firmware/latest/', api_ultimo_firmware, name='api_ultimo_firmware'),
    path('firmware/version/eliminar/<int:v_id>/', eliminar_version_firmware, name='eliminar_version'),
    path('dispositivos/ota/<int:dispositivo_id>/', disparar_ota_mqtt, name='disparar_ota'),
    path('firmware/proyecto/eliminar/<int:p_id>/', eliminar_proyecto_firmware, name='eliminar_proyecto'),
]

if settings.DEBUG:
    urlpatterns += static(settings.MEDIA_URL, document_root=settings.MEDIA_ROOT)