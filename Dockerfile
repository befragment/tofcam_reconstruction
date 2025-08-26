FROM python:3.10-bullseye

# Системные зависимости для open3d / opencv / numpy
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    libgl1 \
    libglib2.0-0 \
    libx11-6 \
    libstdc++6 \
    libgomp1 \
    ca-certificates \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Кэшируем зависимости
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Копируем проект
COPY . .

# (На всякий случай) явный порт
EXPOSE 8000

# По умолчанию команду задаём в compose
