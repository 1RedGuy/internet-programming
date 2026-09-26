function ErrorMessage({ messages }) {
  if (!messages) {
    return null
  }

  const list = Array.isArray(messages) ? messages : [messages]

  return (
    <div className="error">
      {list.map((message) => (
        <p key={message}>{message}</p>
      ))}
    </div>
  )
}

export default ErrorMessage
