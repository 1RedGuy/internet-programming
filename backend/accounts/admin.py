from django.contrib import admin
from django.contrib.auth.admin import UserAdmin

from .models import Profile, User


admin.site.register(User, UserAdmin)


@admin.register(Profile)
class ProfileAdmin(admin.ModelAdmin):
    list_display = ["nickname", "user", "avatar_key"]
    search_fields = ["nickname", "user__username"]
