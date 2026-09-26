from django.test import TestCase

from accounts.models import Profile
from accounts.serializers import ProfileSerializer, RegisterSerializer


class RegisterSerializerTests(TestCase):
    def get_data(self, **changes):
        data = {
            "username": "player_one",
            "email": "player@example.com",
            "nickname": "MountainKnight",
            "password": "example-password",
            "password_confirm": "example-password",
        }
        data.update(changes)
        return data

    def test_creates_user_and_profile(self):
        serializer = RegisterSerializer(data=self.get_data())
        self.assertTrue(serializer.is_valid(), serializer.errors)

        user = serializer.save()

        self.assertEqual(user.profile.nickname, "MountainKnight")
        self.assertNotIn("password", serializer.data)
        self.assertNotIn("password_confirm", serializer.data)

    def test_rejects_weak_password(self):
        serializer = RegisterSerializer(data=self.get_data(password="123", password_confirm="123"))

        self.assertFalse(serializer.is_valid())
        self.assertIn("password", serializer.errors)


class ProfileSerializerTests(TestCase):
    def test_rejects_unknown_avatar(self):
        serializer = ProfileSerializer(data={"nickname": "Knight", "avatar_key": "dragon"})

        self.assertFalse(serializer.is_valid())
        self.assertIn("avatar_key", serializer.errors)
        self.assertEqual(Profile.objects.count(), 0)
