from django.contrib import admin

from .models import Capital, Game, Player, Round, Territory


@admin.register(Game)
class GameAdmin(admin.ModelAdmin):
    list_display = ["id", "created_at", "status", "player_count"]
    list_filter = ["status"]

    @admin.display(description="Players")
    def player_count(self, obj):
        return obj.players.count()


@admin.register(Player)
class PlayerAdmin(admin.ModelAdmin):
    list_display = ["user", "game", "color", "score"]
    list_filter = ["game", "color"]
    search_fields = ["user__username"]


@admin.register(Round)
class RoundAdmin(admin.ModelAdmin):
    list_display = ["game", "number", "type", "status", "winner", "created_at", "completed_at"]
    list_filter = ["game", "type", "status"]


# Territories and capitals are created when a game is initialized,
# so in the admin only the score and the health can be changed.
@admin.register(Territory)
class TerritoryAdmin(admin.ModelAdmin):
    list_display = ["name", "slug", "game", "owner", "score", "neighbor_list"]
    list_filter = ["game", "owner"]
    search_fields = ["name", "slug"]
    fields = ["game", "name", "slug", "owner", "neighbor_list", "score"]
    readonly_fields = ["game", "name", "slug", "owner", "neighbor_list"]

    @admin.display(description="Neighbors")
    def neighbor_list(self, obj):
        return ", ".join(neighbor.name for neighbor in obj.neighbors.all())

    def has_add_permission(self, request):
        return False

    def has_delete_permission(self, request, obj=None):
        return False


@admin.register(Capital)
class CapitalAdmin(admin.ModelAdmin):
    list_display = ["player", "territory", "health"]
    list_filter = ["territory__game"]
    search_fields = ["player__user__username", "territory__name"]
    fields = ["player", "territory", "health"]
    readonly_fields = ["player", "territory"]

    def has_add_permission(self, request):
        return False

    def has_delete_permission(self, request, obj=None):
        return False
