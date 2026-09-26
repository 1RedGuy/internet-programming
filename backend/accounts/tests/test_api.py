from django.contrib.auth import get_user_model
from rest_framework import status
from rest_framework.test import APIClient, APITestCase

from accounts.models import Profile

User = get_user_model()

CSRF_URL = "/api/auth/csrf/"
REGISTER_URL = "/api/auth/register/"
LOGIN_URL = "/api/auth/login/"
LOGOUT_URL = "/api/auth/logout/"
ME_URL = "/api/auth/me/"

PASSWORD = "example-password"


def create_user(username="player_one", email="player@example.com", nickname="MountainKnight"):
    user = User.objects.create_user(username=username, email=email, password=PASSWORD)
    Profile.objects.create(user=user, nickname=nickname)
    return user


class RegisterTests(APITestCase):
    def get_data(self, **changes):
        data = {
            "username": "player_one",
            "email": "player@example.com",
            "nickname": "MountainKnight",
            "password": PASSWORD,
            "password_confirm": PASSWORD,
        }
        data.update(changes)
        return data

    def test_register_success(self):
        response = self.client.post(REGISTER_URL, self.get_data(), format="json")

        self.assertEqual(response.status_code, status.HTTP_201_CREATED)
        user = User.objects.get(username="player_one")
        self.assertEqual(user.profile.nickname, "MountainKnight")
        self.assertTrue(user.check_password(PASSWORD))
        self.assertEqual(response.data["profile"]["nickname"], "MountainKnight")
        self.assertNotIn("password", response.data)

    def test_register_invalid_data(self):
        create_user(username="taken", email="taken@example.com", nickname="Taken")
        cases = [
            ("username", {"username": "taken"}),
            ("email", {"email": "taken@example.com"}),
            ("nickname", {"nickname": "Taken"}),
            ("password_confirm", {"password_confirm": "other-password"}),
        ]

        for field, changes in cases:
            with self.subTest(field=field):
                response = self.client.post(REGISTER_URL, self.get_data(**changes), format="json")

                self.assertEqual(response.status_code, status.HTTP_400_BAD_REQUEST)
                self.assertIn(field, response.data["errors"])
                self.assertEqual(User.objects.count(), 1)


class LoginTests(APITestCase):
    def setUp(self):
        self.user = create_user()

    def test_login_success(self):
        response = self.client.post(
            LOGIN_URL,
            {"username": "player_one", "password": PASSWORD},
            format="json",
        )

        self.assertEqual(response.status_code, status.HTTP_200_OK)
        self.assertIn("sessionid", response.cookies)

        response = self.client.get(ME_URL)
        self.assertEqual(response.status_code, status.HTTP_200_OK)
        self.assertEqual(response.data["username"], "player_one")

    def test_login_wrong_password(self):
        response = self.client.post(
            LOGIN_URL,
            {"username": "player_one", "password": "wrong-password"},
            format="json",
        )

        self.assertEqual(response.status_code, status.HTTP_400_BAD_REQUEST)
        self.assertNotIn("_auth_user_id", self.client.session)

        response = self.client.get(ME_URL)
        self.assertEqual(response.status_code, status.HTTP_403_FORBIDDEN)


class MeTests(APITestCase):
    def setUp(self):
        self.user = create_user()
        self.other_user = create_user(username="player_two", email="two@example.com", nickname="OtherKnight")

    def test_anonymous_user_is_denied(self):
        response = self.client.get(ME_URL)

        self.assertEqual(response.status_code, status.HTTP_403_FORBIDDEN)
        self.assertIn("errors", response.data)

    def test_returns_only_current_user(self):
        self.client.force_login(self.user)

        response = self.client.get(ME_URL)

        self.assertEqual(response.status_code, status.HTTP_200_OK)
        self.assertEqual(response.data["id"], self.user.id)
        self.assertEqual(response.data["profile"]["nickname"], "MountainKnight")

    def test_update_profile(self):
        self.client.force_login(self.user)

        response = self.client.patch(
            ME_URL,
            {"nickname": "NewKnight", "avatar_key": "knight-3", "is_staff": True},
            format="json",
        )

        self.assertEqual(response.status_code, status.HTTP_200_OK)
        profile = Profile.objects.get(user=self.user)
        self.assertEqual(profile.nickname, "NewKnight")
        self.assertEqual(profile.avatar_key, "knight-3")
        self.user.refresh_from_db()
        self.assertFalse(self.user.is_staff)
        self.assertEqual(Profile.objects.get(user=self.other_user).nickname, "OtherKnight")


class LogoutTests(APITestCase):
    def test_logout(self):
        create_user()
        self.client.post(LOGIN_URL, {"username": "player_one", "password": PASSWORD}, format="json")

        response = self.client.post(LOGOUT_URL)

        self.assertEqual(response.status_code, status.HTTP_204_NO_CONTENT)
        self.assertNotIn("_auth_user_id", self.client.session)
        response = self.client.get(ME_URL)
        self.assertEqual(response.status_code, status.HTTP_403_FORBIDDEN)


class CsrfTests(APITestCase):
    def test_unsafe_request_needs_csrf_token(self):
        create_user()
        client = APIClient(enforce_csrf_checks=True)
        client.login(username="player_one", password=PASSWORD)

        response = client.patch(ME_URL, {"nickname": "NoToken"}, format="json")
        self.assertEqual(response.status_code, status.HTTP_403_FORBIDDEN)

        response = client.get(CSRF_URL)
        self.assertEqual(response.status_code, status.HTTP_204_NO_CONTENT)
        token = response.cookies["csrftoken"].value

        response = client.patch(ME_URL, {"nickname": "WithToken"}, format="json", HTTP_X_CSRFTOKEN=token)
        self.assertEqual(response.status_code, status.HTTP_200_OK)
        self.assertEqual(response.data["profile"]["nickname"], "WithToken")
