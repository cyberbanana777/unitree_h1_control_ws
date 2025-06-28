#!/bin/bash

# Включаем автоматическую синхронизацию времени через systemd-timesyncd
sudo systemctl enable systemd-timesyncd.service
sudo systemctl start systemd-timesyncd.service

# Делаем небольшую паузу, чтобы время успело синхронизироваться
sleep 5

# Получаем текущее время
CURRENT_DATE=$(date +"%Y-%m-%d_%H:%M:%S")

# Добавляем все изменения в индекс
git add .

# Создаём коммит с сообщением, содержащим дату и время
git commit -m "Commit on $CURRENT_DATE"

git push -u origin main
