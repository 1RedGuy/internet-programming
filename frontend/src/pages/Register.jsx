import { useState } from 'react'
import { getErrors, register } from '../api'
import ErrorMessage from '../components/ErrorMessage'

const emptyForm = {
  username: '',
  email: '',
  nickname: '',
  password: '',
  password_confirm: '',
}

function Register({ onRegister }) {
  const [form, setForm] = useState(emptyForm)
  const [errors, setErrors] = useState({})

  function handleChange(event) {
    setForm({ ...form, [event.target.name]: event.target.value })
  }

  async function handleSubmit(event) {
    event.preventDefault()
    const response = await register(form)

    if (response.ok) {
      onRegister(response.data)
    } else {
      setErrors(getErrors(response))
    }
  }

  return (
    <form className="card" onSubmit={handleSubmit}>
      <h2>Register</h2>
      <ErrorMessage messages={errors.non_field_errors} />
      <ErrorMessage messages={errors.detail} />

      <label>
        Username
        <input name="username" value={form.username} onChange={handleChange} required />
      </label>
      <ErrorMessage messages={errors.username} />

      <label>
        Email
        <input name="email" type="email" value={form.email} onChange={handleChange} required />
      </label>
      <ErrorMessage messages={errors.email} />

      <label>
        Nickname
        <input name="nickname" value={form.nickname} onChange={handleChange} maxLength={30} required />
      </label>
      <ErrorMessage messages={errors.nickname} />

      <label>
        Password
        <input name="password" type="password" value={form.password} onChange={handleChange} required />
      </label>
      <ErrorMessage messages={errors.password} />

      <label>
        Confirm password
        <input
          name="password_confirm"
          type="password"
          value={form.password_confirm}
          onChange={handleChange}
          required
        />
      </label>
      <ErrorMessage messages={errors.password_confirm} />

      <button type="submit">Create account</button>
    </form>
  )
}

export default Register
