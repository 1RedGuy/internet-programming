import random

from django.test import TestCase

from games.models import Game, Player, WaitingQueue
from games.services import gather_waiting_players

from .utils import create_user


def create_waiting_users(count):
    users = [create_user(f"waiting{number + 1}") for number in range(count)]
    for user in users:
        WaitingQueue.objects.create(user=user)
    return users


class GatherWaitingPlayersTests(TestCase):
    def test_five_waiting_users(self):
        users = create_waiting_users(5)

        game = gather_waiting_players(rng=random.Random(1))

        self.assertIsNotNone(game)
        game.refresh_from_db()
        self.assertEqual(Game.objects.count(), 1)
        self.assertEqual(game.status, "waiting")
        self.assertEqual(game.territories.count(), 0)
        self.assertEqual(game.rounds.count(), 0)

        players = list(game.players.all())
        self.assertEqual(len(players), 3)
        self.assertEqual({player.color for player in players}, {"red", "green", "blue"})
        self.assertTrue(all(player.score == 0 for player in players))

        chosen = {player.user_id for player in players}
        matched = WaitingQueue.objects.filter(status="matched")
        self.assertEqual({entry.user_id for entry in matched}, chosen)
        self.assertTrue(all(entry.game_id == game.pk and entry.finished_at for entry in matched))

        waiting = WaitingQueue.objects.filter(status="waiting")
        self.assertEqual(waiting.count(), 2)
        self.assertEqual({entry.user_id for entry in waiting}, {user.pk for user in users} - chosen)
        self.assertEqual(WaitingQueue.objects.count(), 5)

    def test_two_waiting_users(self):
        create_waiting_users(2)

        self.assertIsNone(gather_waiting_players())

        self.assertEqual(Game.objects.count(), 0)
        self.assertEqual(Player.objects.count(), 0)
        self.assertEqual(WaitingQueue.objects.count(), 2)
        self.assertFalse(
            WaitingQueue.objects.exclude(status="waiting", game=None, finished_at=None).exists()
        )
