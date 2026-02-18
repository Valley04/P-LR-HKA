from django.contrib.auth.views import LoginView
from django.shortcuts import render
from django.contrib.auth.decorators import login_required

class MiLogin(LoginView):
    template_name = "login.html"

@login_required # Solo para usuarios loggeados
def dashboard_view(request):
    # Mas adelante enviaremos datos a BD
    return render(request, 'dashboard.html')