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

Creating the territories and choosing the capitals for a game is done in M05 (game initialization).
Creating a `Game` does not create a map.
