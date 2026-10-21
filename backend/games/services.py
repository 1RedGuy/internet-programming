import random

from django.core.exceptions import ValidationError
from django.db import IntegrityError, transaction
from django.db.models import Exists, OuterRef
from django.utils import timezone

from . import game_map
from .models import Capital, Game, Player, Territory, WaitingQueue


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


def is_busy(user):
    return Player.objects.filter(user=user, game__status__in=["waiting", "active"]).exists()


def join_waiting_room(user):
    if WaitingQueue.objects.filter(user=user, status="waiting").exists():
        raise ValidationError("You are already in the waiting room.", code="already_waiting")
    if is_busy(user):
        raise ValidationError("You are already in a game.", code="busy")

    try:
        with transaction.atomic():
            return WaitingQueue.objects.create(user=user)
    except IntegrityError:
        raise ValidationError("You are already in the waiting room.", code="already_waiting")


def leave_waiting_room(user):
    updated = WaitingQueue.objects.filter(user=user, status="waiting").update(
        status="left",
        finished_at=timezone.now(),
    )
    return updated > 0


@transaction.atomic
def gather_waiting_players(rng=random):
    busy = Player.objects.filter(user=OuterRef("user"), game__status__in=["waiting", "active"])
    entries = list(
        WaitingQueue.objects.select_for_update()
        .filter(status="waiting")
        .exclude(Exists(busy))
        .order_by("id")
    )
    if len(entries) < 3:
        return None

    chosen = rng.sample(entries, 3)

    game = Game.objects.create()
    colors = [color for color, _ in Player.COLOR_CHOICES]
    for entry, color in zip(chosen, colors):
        player = Player(game=game, user_id=entry.user_id, color=color, score=0)
        player.full_clean()
        player.save()

    updated = WaitingQueue.objects.filter(pk__in=[entry.pk for entry in chosen], status="waiting").update(
        status="matched",
        game=game,
        finished_at=timezone.now(),
    )
    if updated != 3:
        raise ValidationError("The waiting room changed while the game was formed.")

    return game
