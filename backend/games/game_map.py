from itertools import combinations

from django.core.exceptions import ValidationError

# The map of the project. Every game gets its own territories
# with these names and neighbors when it is initialized.
TERRITORIES = {
    "skali": "Скали",
    "dunaviya": "Дунавия",
    "zhitno-pole": "Житно поле",
    "kaliakra": "Калиакра",
    "sredets": "Средец",
    "leventa": "Левента",
    "madara": "Мадара",
    "pelikania": "Пеликания",
    "pirina": "Пирина",
    "ezera": "Езера",
    "balkania": "Балкания",
    "tsarevo": "Царево",
}

NEIGHBORS = {
    "skali": ["dunaviya", "sredets"],
    "dunaviya": ["skali", "zhitno-pole", "leventa", "madara"],
    "zhitno-pole": ["dunaviya", "kaliakra", "madara"],
    "kaliakra": ["zhitno-pole", "pelikania"],
    "sredets": ["skali", "leventa", "pirina", "ezera"],
    "leventa": ["dunaviya", "sredets", "madara", "ezera"],
    "madara": ["dunaviya", "zhitno-pole", "leventa", "pelikania", "balkania"],
    "pelikania": ["kaliakra", "madara", "balkania", "tsarevo"],
    "pirina": ["sredets", "ezera"],
    "ezera": ["sredets", "leventa", "pirina", "balkania"],
    "balkania": ["madara", "pelikania", "ezera", "tsarevo"],
    "tsarevo": ["pelikania", "balkania"],
}


def is_connected(neighbors):
    slugs = list(neighbors)
    visited = {slugs[0]}
    to_visit = [slugs[0]]

    while to_visit:
        current = to_visit.pop()
        for neighbor in neighbors[current]:
            if neighbor not in visited:
                visited.add(neighbor)
                to_visit.append(neighbor)

    return len(visited) == len(slugs)


def get_capital_options(neighbors):
    """Returns all groups of 3 territories that are not neighbors of each other."""
    options = []
    for first, second, third in combinations(neighbors, 3):
        if second in neighbors[first] or third in neighbors[first] or third in neighbors[second]:
            continue
        options.append((first, second, third))
    return options


def validate_map(territories, neighbors):
    count = len(territories)
    if count < 9 or count > 21 or count % 3 != 0:
        raise ValidationError("The map must have between 9 and 21 territories and the number must be divisible by 3.")

    if len(set(territories.values())) != count:
        raise ValidationError("Territory names must be unique.")

    if set(neighbors) != set(territories):
        raise ValidationError("Every territory must have a list of neighbors.")

    for slug, slug_neighbors in neighbors.items():
        if slug in slug_neighbors:
            raise ValidationError(f"{slug} cannot be a neighbor of itself.")
        if len(slug_neighbors) != len(set(slug_neighbors)):
            raise ValidationError(f"{slug} has duplicate neighbors.")
        if len(slug_neighbors) < 2:
            raise ValidationError(f"{slug} must have at least 2 neighbors.")

        for neighbor in slug_neighbors:
            if neighbor not in territories:
                raise ValidationError(f"{neighbor} is not a territory on the map.")
            if slug not in neighbors[neighbor]:
                raise ValidationError(f"{slug} and {neighbor} must be neighbors in both directions.")

    if not is_connected(neighbors):
        raise ValidationError("Every territory must be reachable from every other territory.")

    if not get_capital_options(neighbors):
        raise ValidationError("The map must have 3 territories that are not neighbors of each other.")


def validate_game_map(game):
    """Checks that the territories of a game match the map definition."""
    territories = list(game.territories.prefetch_related("neighbors"))

    if sorted(territory.slug for territory in territories) != sorted(TERRITORIES):
        raise ValidationError("The territories of the game do not match the map.")

    for territory in territories:
        if territory.name != TERRITORIES[territory.slug]:
            raise ValidationError(f"{territory.slug} has a wrong name.")

        neighbors = {(neighbor.game_id, neighbor.slug) for neighbor in territory.neighbors.all()}
        expected = {(game.id, slug) for slug in NEIGHBORS[territory.slug]}
        if neighbors != expected:
            raise ValidationError(f"{territory.slug} has wrong neighbors.")
