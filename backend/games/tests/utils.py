from django.contrib.auth import get_user_model

from games.game_map import NEIGHBORS, TERRITORIES
from games.models import Game, Player, Territory

User = get_user_model()

COLORS = ["red", "green", "blue"]


def create_user(username):
    return User.objects.create_user(username=username, email=f"{username}@example.com")


def create_game(players=3, status="waiting"):
    game = Game.objects.create()
    for number in range(players):
        user = create_user(f"game{game.pk}_player{number + 1}")
        Player.objects.create(game=game, user=user, color=COLORS[number])

    if status != "waiting":
        game.status = status
        game.save()

    return game


def create_map(game):
    """Creates the territories of the project map for a game (only for the tests)."""
    territories = {}
    for slug, name in TERRITORIES.items():
        territories[slug] = Territory.objects.create(game=game, name=name, slug=slug)

    for slug, neighbors in NEIGHBORS.items():
        territories[slug].neighbors.add(*[territories[neighbor] for neighbor in neighbors])

    return territories
