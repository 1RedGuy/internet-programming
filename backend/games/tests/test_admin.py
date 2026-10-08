from django.contrib.auth import get_user_model
from django.test import TestCase

from games.models import Capital

from .utils import create_game, create_map

User = get_user_model()


class MapAdminTests(TestCase):
    def setUp(self):
        admin = User.objects.create_superuser(username="admin", email="admin@example.com", password="admin-pass")
        self.client.force_login(admin)
        self.game = create_game()
        self.territories = create_map(self.game)
        self.player = self.game.players.first()
        self.territories["skali"].owner = self.player
        self.territories["skali"].save()
        self.capital = Capital.objects.create(territory=self.territories["skali"], player=self.player)

    def test_territory_search_and_filters(self):
        response = self.client.get("/admin/games/territory/", {"q": "zhitno"})
        self.assertContains(response, "Житно поле")
        self.assertNotContains(response, "Царево")

        response = self.client.get("/admin/games/territory/", {"game__id__exact": self.game.id})
        self.assertEqual(response.status_code, 200)

    def test_capital_search_and_filters(self):
        response = self.client.get("/admin/games/capital/", {"q": self.player.user.username})
        self.assertContains(response, "Скали")

        response = self.client.get("/admin/games/capital/", {"territory__game__id__exact": self.game.id})
        self.assertEqual(response.status_code, 200)

    def test_cannot_add_or_delete(self):
        territory = self.territories["madara"]

        self.assertEqual(self.client.get("/admin/games/territory/add/").status_code, 403)
        self.assertEqual(self.client.get(f"/admin/games/territory/{territory.pk}/delete/").status_code, 403)
        self.assertEqual(self.client.get("/admin/games/capital/add/").status_code, 403)

    def test_territory_score_is_validated(self):
        territory = self.territories["madara"]
        url = f"/admin/games/territory/{territory.pk}/change/"

        self.assertEqual(self.client.get(url).status_code, 200)

        response = self.client.post(url, {"score": -5, "slug": "changed"})
        self.assertEqual(response.status_code, 200)
        territory.refresh_from_db()
        self.assertEqual(territory.score, 0)

        self.client.post(url, {"score": 15, "slug": "changed"})
        territory.refresh_from_db()
        self.assertEqual(territory.score, 15)
        self.assertEqual(territory.slug, "madara")

    def test_capital_health_is_validated(self):
        url = f"/admin/games/capital/{self.capital.pk}/change/"

        response = self.client.post(url, {"health": 4})
        self.assertEqual(response.status_code, 200)
        self.capital.refresh_from_db()
        self.assertEqual(self.capital.health, 3)

        self.client.post(url, {"health": 1})
        self.capital.refresh_from_db()
        self.assertEqual(self.capital.health, 1)
