from django.urls import path
from impresoras.views import MiLogin #Importamos views

urlpatterns = [
    path('login/', MiLogin.as_view(), name='login'),
]