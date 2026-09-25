from django.contrib.auth import authenticate, get_user_model, password_validation
from django.db import transaction
from rest_framework import serializers

from .models import Profile

User = get_user_model()


class ProfileSerializer(serializers.ModelSerializer):
    class Meta:
        model = Profile
        fields = ["nickname", "avatar_key"]


class UserSerializer(serializers.ModelSerializer):
    profile = ProfileSerializer(read_only=True)

    class Meta:
        model = User
        fields = ["id", "username", "email", "profile"]


class RegisterSerializer(serializers.ModelSerializer):
    nickname = serializers.CharField(max_length=30)
    password = serializers.CharField(write_only=True)
    password_confirm = serializers.CharField(write_only=True)

    class Meta:
        model = User
        fields = ["username", "email", "nickname", "password", "password_confirm"]

    def validate_nickname(self, value):
        if Profile.objects.filter(nickname=value).exists():
            raise serializers.ValidationError("Profile with this nickname already exists.")
        return value

    def validate_password(self, value):
        password_validation.validate_password(value)
        return value

    def validate(self, data):
        if data["password"] != data["password_confirm"]:
            raise serializers.ValidationError({"password_confirm": ["Passwords do not match."]})
        return data

    def create(self, validated_data):
        nickname = validated_data.pop("nickname")
        validated_data.pop("password_confirm")

        with transaction.atomic():
            user = User.objects.create_user(**validated_data)
            Profile.objects.create(user=user, nickname=nickname)

        return user

    def to_representation(self, instance):
        return UserSerializer(instance).data


class LoginSerializer(serializers.Serializer):
    username = serializers.CharField()
    password = serializers.CharField(write_only=True)

    def validate(self, data):
        user = authenticate(
            request=self.context.get("request"),
            username=data["username"],
            password=data["password"],
        )
        if user is None:
            raise serializers.ValidationError("Invalid username or password.")

        data["user"] = user
        return data
