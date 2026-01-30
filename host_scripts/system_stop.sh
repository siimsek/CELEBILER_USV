#!/bin/bash
# Hızlı Kapatma Scripti
# Kullanım: ./host_scripts/system_stop.sh

echo -e "\033[0;31m🛑 Sistem HIZLA Durduruluyor...\033[0m"

# 1. Host Kamera - Beklemeden Öldür (Force Kill)
# Önce normal kapatmayı dene, olmazsa zorla
sudo pkill -9 rpicam-vid > /dev/null 2>&1
sudo fuser -k 8888/tcp > /dev/null 2>&1

# 2. Docker Konteynerini Durdur
# 'docker stop' normalde 10 saniye bekler. Bunu 1 saniyeye düşürüyoruz (-t 1).
# Eğer kapanmazsa 'docker kill' devreye girer.
echo "   -> Docker (ege_ros) kapatılıyor..."
docker stop -t 1 ege_ros > /dev/null 2>&1 || docker kill ege_ros > /dev/null 2>&1

echo -e "\033[0;32m✅ Sistem anında kapatıldı.\033[0m"
