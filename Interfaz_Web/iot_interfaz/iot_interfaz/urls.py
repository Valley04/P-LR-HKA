from django.contrib import admin
from django.urls import path
from impresoras.views import MiLogin, dashboard_view, lista_dispositivos #Importamos views
from django.contrib.auth.views import LogoutView

urlpatterns = [
    path('admin/', admin.site.urls),
    path('login/', MiLogin.as_view(), name='login'),
    path('dashboard/', dashboard_view, name='dashboard'),
    path('logout/', LogoutView.as_view(), name='logout'),
    path('dispositivos/', lista_dispositivos, name='dispositivos_lista'),
]
