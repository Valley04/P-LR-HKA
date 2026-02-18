from django.contrib import admin
from django.urls import path
from impresoras.views import MiLogin, dashboard_view #Importamos views

urlpatterns = [
    path('login/', MiLogin.as_view(), name='login'),
    path('dashboard/', dashboard_view, name='dashboard'),
]
