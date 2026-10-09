# Quiz Conquest

Triviador style quiz game made with Django REST Framework and React.

## Structure

- `backend/` - Django + Django REST Framework
- `frontend/` - React + Vite

## Backend

```bash
cd backend
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python manage.py migrate
python manage.py runserver
```

Load the question bank:

```bash
python manage.py loaddata questions/question_bank.json
```

The fixture has 6 categories, 12 choice questions (with 4 answers each) and 12 numeric questions.

Tests:

```bash
python manage.py test
```

## Frontend

```bash
cd frontend
npm install
npm run dev
```

The app runs on `http://localhost:5173`. Requests to `/api` are sent to the Django server through the Vite proxy, so Django has to be running too.

## Auth API

| Method | Endpoint | Access |
|---|---|---|
| GET | `/api/auth/csrf/` | Public |
| POST | `/api/auth/register/` | Public |
| POST | `/api/auth/login/` | Public |
| POST | `/api/auth/logout/` | Logged in |
| GET | `/api/auth/me/` | Logged in |
| PATCH | `/api/auth/me/` | Logged in |

## Game map

The project uses one map with 12 territories. It is defined in `backend/games/game_map.py`.
Every game gets its own territory records with the same names and neighbors, so the owner,
score and capitals are different for every game.

```text
Скали ----- Дунавия ----- Житно поле ---- Калиакра
  |           |     \        |               |
Средец ---- Левента ----- Мадара -------- Пеликания
  |    \      |              |       /       |
Пирина ---- Езера ------- Балкания ------ Царево
```

| Territory | Slug | Neighbors |
|---|---|---|
| Скали | `skali` | Дунавия, Средец |
| Дунавия | `dunaviya` | Скали, Житно поле, Левента, Мадара |
| Житно поле | `zhitno-pole` | Дунавия, Калиакра, Мадара |
| Калиакра | `kaliakra` | Житно поле, Пеликания |
| Средец | `sredets` | Скали, Левента, Пирина, Езера |
| Левента | `leventa` | Дунавия, Средец, Мадара, Езера |
| Мадара | `madara` | Дунавия, Житно поле, Левента, Пеликания, Балкания |
| Пеликания | `pelikania` | Калиакра, Мадара, Балкания, Царево |
| Пирина | `pirina` | Средец, Езера |
| Езера | `ezera` | Средец, Левента, Пирина, Балкания |
| Балкания | `balkania` | Мадара, Пеликания, Езера, Царево |
| Царево | `tsarevo` | Пеликания, Балкания |

Map rules (checked by `validate_map`):

- between 9 and 21 territories, divisible by 3
- neighbors are symmetric, without self references and duplicates
- every territory has at least 2 neighbors
- all territories are connected
- there are at least 3 territories that are not neighbors of each other (for the capitals)

Creating a `Game` does not create a map. The territories and the capitals are created
when the game is initialized (see below).

## Game initialization

A game is started with `initialize_game` from `backend/games/services.py`. It is not called
automatically when a game is created.

From the Django admin: Games -> select the games -> action "Initialize selected games".

From the shell:

```bash
python manage.py shell
```

```python
from games.models import Game
from games.services import initialize_game

game = Game.objects.get(pk=1)
initialize_game(game)
```

The game must be `waiting`, have exactly 3 players and no territories or capitals yet.
Initialization:

- creates the 12 territories of the map and their neighbors for this game
- picks a random group of 3 territories that are not neighbors and gives one to every player as a capital (health 3)
- the other 9 territories have no owner, all territory scores are 0
- changes the game status to `active`
- does not change player scores and does not create rounds

Everything runs in one transaction (`transaction.atomic`), so if something fails nothing is saved.
A second initialization of the same game is refused. Players can be added or removed only while
the game is `waiting`.

### Database and concurrency

The game row is locked with `select_for_update()` while the game is initialized and while players
join or leave, so two starts at the same time can't create two maps. There are also database
constraints (unique territory slug per game, one capital per player) as a second protection.

The project uses SQLite for development. SQLite does not support `select_for_update()` (Django ignores it),
it locks the whole database instead. Because of that the database is configured with
`"transaction_mode": "IMMEDIATE"`, so every transaction takes the write lock when it starts and two
initializations of the same game run one after the other. With many users at the same time this
can give "database is locked" errors, so for real use PostgreSQL is better.

The test with two starts at the same time (`ConcurrentInitializationTests`) runs only on a database
that supports `select_for_update()` (for example PostgreSQL). On SQLite it is skipped.
