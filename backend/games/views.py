from django.core.exceptions import ValidationError
from rest_framework import status
from rest_framework.decorators import api_view, permission_classes
from rest_framework.permissions import IsAuthenticated
from rest_framework.response import Response

from .services import gather_waiting_players, join_waiting_room, leave_waiting_room


@api_view(["POST"])
@permission_classes([IsAuthenticated])
def join_view(request):
    try:
        entry = join_waiting_room(request.user)
    except ValidationError as error:
        code = status.HTTP_409_CONFLICT if error.code == "already_waiting" else status.HTTP_400_BAD_REQUEST
        return Response({"errors": {"detail": error.messages}}, status=code)

    game = gather_waiting_players()
    return Response(
        {"entry_id": entry.pk, "game_id": game.pk if game else None},
        status=status.HTTP_201_CREATED,
    )


@api_view(["POST"])
@permission_classes([IsAuthenticated])
def leave_view(request):
    if leave_waiting_room(request.user):
        return Response(status=status.HTTP_204_NO_CONTENT)
    return Response({"left": False})
