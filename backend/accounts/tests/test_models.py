from django.contrib.auth import get_user_model
from django.db import IntegrityError
from django.test import TestCase

from accounts.models import Profile

User = get_user_model()


class UserModelTests(TestCase):
    def test_email_is_unique(self):
        User.objects.create_user(username="ivan", email="same@example.com", password="example-password")

        with self.assertRaises(IntegrityError):
            User.objects.create_user(username="petar", email="same@example.com", password="example-password")


class ProfileModelTests(TestCase):
    def setUp(self):
        self.user = User.objects.create_user(
            username="ivan",
            email="ivan@example.com",
            password="example-password",
        )

    def test_profile_is_linked_to_user(self):
        profile = Profile.objects.create(user=self.user, nickname="Ivo")

        self.assertEqual(self.user.profile, profile)
        self.assertEqual(profile.avatar_key, "knight-1")

    def test_nickname_is_unique(self):
        Profile.objects.create(user=self.user, nickname="Ivo")
        other = User.objects.create_user(username="petar", email="petar@example.com", password="example-password")

        with self.assertRaises(IntegrityError):
            Profile.objects.create(user=other, nickname="Ivo")
