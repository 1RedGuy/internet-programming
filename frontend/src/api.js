function getCookie(name) {
  const cookies = document.cookie ? document.cookie.split('; ') : []

  for (const cookie of cookies) {
    const [key, ...value] = cookie.split('=')
    if (key === name) {
      return decodeURIComponent(value.join('='))
    }
  }

  return null
}

async function request(url, method = 'GET', data = null) {
  const options = {
    method,
    credentials: 'same-origin',
    headers: {},
  }

  if (method !== 'GET') {
    options.headers['X-CSRFToken'] = getCookie('csrftoken')
  }

  if (data) {
    options.headers['Content-Type'] = 'application/json'
    options.body = JSON.stringify(data)
  }

  const response = await fetch(url, options)
  let body = null

  if (response.status !== 204) {
    body = await response.json().catch(() => null)
  }

  return { ok: response.ok, status: response.status, data: body }
}

export function getCsrfToken() {
  return request('/api/auth/csrf/')
}

export function register(data) {
  return request('/api/auth/register/', 'POST', data)
}

export function login(username, password) {
  return request('/api/auth/login/', 'POST', { username, password })
}

export function logout() {
  return request('/api/auth/logout/', 'POST')
}

export function getMe() {
  return request('/api/auth/me/')
}

export function updateMe(data) {
  return request('/api/auth/me/', 'PATCH', data)
}

export function getErrors(response) {
  if (response.data && response.data.errors) {
    return response.data.errors
  }
  return { detail: 'Something went wrong. Try again.' }
}
