from django.urls import path

from . import views

urlpatterns = [
    path("join/", views.join_view, name="waiting-room-join"),
    path("leave/", views.leave_view, name="waiting-room-leave"),
]
