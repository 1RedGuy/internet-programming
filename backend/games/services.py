import random

from django.core.exceptions import ValidationError
from django.db import transaction

from . import game_map
from .models import Capital, Game, Territory


@transaction.atomic
def initialize_game(game, rng=random):
    """
    Creates the territories and the capitals of a game and makes it active.
    Everything is in one transaction, so if something fails nothing is saved.
    `rng` can be changed in the tests to control the random choice.
    """
    # lock the game row so two starts at the same time can't both pass the checks
    game = Game.objects.select_for_update().get(pk=game.pk)

    if game.status != "waiting":
        raise ValidationError("Only a waiting game can be initialized.")

    players = list(game.players.order_by("id"))
    if len(players) != 3:
        raise ValidationError("A game needs exactly 3 players to start.")

    if game.territories.exists() or Capital.objects.filter(player__game=game).exists():
        raise ValidationError("The game already has a map or capitals.")

    game_map.validate_map(game_map.TERRITORIES, game_map.NEIGHBORS)

    territories = {}
    for slug, name in game_map.TERRITORIES.items():
        territories[slug] = Territory.objects.create(game=game, name=name, slug=slug)

    for slug, neighbors in game_map.NEIGHBORS.items():
        territories[slug].neighbors.add(*[territories[neighbor] for neighbor in neighbors])

    # random group of 3 territories that are not neighbors, in random order
    capital_slugs = list(rng.choice(game_map.get_capital_options(game_map.NEIGHBORS)))
    rng.shuffle(capital_slugs)

    for player, slug in zip(players, capital_slugs):
        territory = territories[slug]
        territory.owner = player
        territory.save()

        capital = Capital(territory=territory, player=player)
        capital.full_clean()
        capital.save()

    game_map.validate_game_map(game)

    game.status = "active"
    game.save()
    return game
