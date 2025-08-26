# Time-of-flight camera images reconstruction

### how to launch 

---
```bash
docker compose up -d 
```

then use openapi schema or send a request to localhost via `curl`.

example:
```bash
curl -v -X POST "http://localhost:8000/convert" \ 
-H "accept: application/octet-stream" \ 
-F "file=@./background0.depth;type=application/octet-stream" \ 
-o out.ply
```
