curl http://localhost:5050/v1/chat/completions \
  -H "Content-Type: application/json" \
  -d '{
    "model": "gpt-oss-120b-mxfp4",
    "messages": [{"role": "user", "content": "Hello assistant., please tell me who you are. Please respond only with your answer. Do not include any reasoning, introduction, or explanation!"}],
    "max_tokens": 50
  }'
