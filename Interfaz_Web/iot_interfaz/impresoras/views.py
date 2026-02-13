from django.contrib.auth.views import LoginView

class MiLogin(LoginView):
    template_name = "login.html" #El HTML que queremos mostrar