from django.contrib import admin
from django.urls import path
from impresoras.views import MiLogin, dashboard_view, lista_dispositivos, editar_dispositivo, eliminar_dispositivo, grupos_modelos, eliminar_grupo, eliminar_modelo, exportar_dispositivos_excel, log_capture_view #Importamos views
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
]
