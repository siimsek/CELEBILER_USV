#!/bin/bash
# USV ACİL DURDURMA SCRİPTİ (ALL-IN-ONE)
# Kullanım: ./host_scripts/system_stop.sh

echo -e "\033[0;31m🛑 TÜM SİSTEM DURDURULUYOR...\033[0m"

# 1. Host Kamera ve Kritik Portlar
# Önce portları kullanan her şeyi öldür (Host tarafında)
sudo pkill -9 rpicam-vid 2>/dev/null
sudo fuser -k 8888/tcp 2>/dev/null
sudo fuser -k 5000/tcp 2>/dev/null
sudo fuser -k 5001/tcp 2>/dev/null
sudo fuser -k 8080/tcp 2>/dev/null

# 2. Docker Konteynerini Zorla Durdur
# -t 0: Sıfır saniye bekle (Anında SIGKILL gönderir, bekleme yapmaz)
if docker ps -q -f name=ege_ros | grep -q .; then
    echo "   -> Docker (ege_ros) kapatılıyor..."
    docker stop -t 0 ege_ros > /dev/null 2>&1
fi

# 3. Arkaplan Log İzleyicilerini Temizle (Varsa)
pkill -f "tail -f.*logs" 2>/dev/null
pkill -f "usv_logs.sh" 2>/dev/null

echo -e "\033[0;32m✅ SİSTEM TAMAMEN VE GÜVENLE KAPATILDI.\033[0m"
