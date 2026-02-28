#!/bin/bash
# Dosya Yeri: CELEBILER_USV/host_scripts/system_start.sh

CONTAINER_NAME="ege_ros"
# Config path: script dizininden veya $HOME/CELEBILER_USV
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")"; pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
if [ ! -f "$PROJECT_ROOT/config/usv_mode.cfg" ]; then
    PROJECT_ROOT="$HOME/CELEBILER_USV"
fi
CONFIG_FILE="$PROJECT_ROOT/config/usv_mode.cfg"

# Renk Kodları
GREEN='\033[0;32m'
CYAN='\033[0;36m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BOLD='\033[1m'
DIM='\033[2m'
WHITE='\033[1;37m'
BLUE='\033[0;34m'
NC='\033[0m'

clear
echo -e "${CYAN}=================================================${NC}"
echo -e "${CYAN}   🚀 ÇELEBİLER USV - YER İSTASYONU BAŞLATICI   ${NC}"
echo -e "${CYAN}=================================================${NC}"

# --- USV MODE (test | race) ---
# Argüman: ./system_start.sh [test|race]  veya  config/usv_mode.cfg
if [ "$1" = "race" ] || [ "$1" = "test" ]; then
    USV_MODE="$1"
    echo -e "📋 [MOD] Komut satırından: ${USV_MODE}"
elif [ -f "$CONFIG_FILE" ]; then
    USV_MODE=$(grep -E "^USV_MODE=" "$CONFIG_FILE" 2>/dev/null | cut -d'=' -f2 | tr -d ' ' | tr '[:upper:]' '[:lower:]')
    if [ "$USV_MODE" != "race" ] && [ "$USV_MODE" != "test" ]; then
        USV_MODE="test"
    fi
    echo -e "📋 [MOD] Config'den: ${USV_MODE}"
else
    USV_MODE="test"
    echo -e "📋 [MOD] Varsayılan: test"
fi

if [ "$USV_MODE" = "race" ]; then
    echo -e "${YELLOW}⚠️  [ŞARTNAME 3.4] Yarışma modunda Raspberry Pi WiFi kapatılmalıdır.${NC}"
fi

LOG_DIR="$HOME/CELEBILER_USV/logs"
echo "🧹 [HOST] Eski loglar temizleniyor..."
sudo rm -f "$LOG_DIR"/*.log
sudo rm -f "$LOG_DIR"/docker/*.log
mkdir -p "$LOG_DIR/docker"

# --- 0. AĞ YAPILANDIRMASI (ÖNCELİKLİ) ---
# Modemin resetlenmesi durumunda Lidar IP'sini (192.168.11.5) kaybetmemek için en başta yapıyoruz.
TARGET_IP="192.168.11.5"
CURRENT_IPS=$(hostname -I)

# Ethernet Arayüzünü Bul (eth0, end0, enp3s0 vb.)
ETH_IFACE=$(ip -o link show | awk -F': ' '{print $2}' | grep -E '^(e|en|eth)' | grep -v 'lo' | head -n 1)

if [[ -z "$ETH_IFACE" ]]; then
    echo -e "${RED}[AĞ HATASI]${NC} Ethernet kartı bulunamadı! Kabloyu kontrol edin."
else
    # IP var mı kontrol et
    if [[ "$CURRENT_IPS" != *"$TARGET_IP"* ]]; then
        echo -e "${YELLOW}[AĞ]${NC} Lidar ağı ($ETH_IFACE) için $TARGET_IP atanıyor..."
        sudo ip addr add 192.168.11.5/24 dev $ETH_IFACE > /dev/null 2>&1 || true
        sleep 1
        CURRENT_IPS=$(hostname -I) # Güncelle
    else
        echo -e "${GREEN}[AĞ]${NC} Lidar IP ($TARGET_IP) zaten $ETH_IFACE üzerinde aktif."
    fi
fi

# --- 1. DONANIM ÖN KONTROLÜ (Pre-Flight Check) ---
echo -e "\n🛡️  [GÜVENLİK] Donanım Bağlantıları Kontrol Ediliyor..."

# A) LIDAR KONTROLÜ
if ping -c 1 -W 1 192.168.11.2 > /dev/null; then
    echo "   ✅ Lidar (192.168.11.2) - BAĞLI"
else
    echo -e "   ❌ [HATA] Lidar Bulunamadı! (IP: 192.168.11.2 Erişim Yok)"
    echo "      -> Modem veya Switch resetlenmiş olabilir."
    echo "      -> Statik IP ataması ($TARGET_IP) yapıldı ama Lidar cevap vermiyor."
    exit 1
fi

# B) SERİ PORT KONTROLÜ (Pixhawk + STM32)
port_count=$(ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null | wc -l)
if [ "$port_count" -ge 2 ]; then
    echo "   ✅ Seri Portlar ($port_count Aygıt) - TAMAM"
else
    echo -e "   ❌ [HATA] Eksik Seri Cihaz! (Bulunan: $port_count, Beklenen: 2+)"
    echo "      -> Pixhawk ve STM32 USB kablolarını kontrol et."
    echo "      -> Bulunanlar: $(ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null)"
    exit 1
fi

# C) KAMERA KONTROLÜ
if rpicam-vid --list-cameras > /dev/null 2>&1; then
    echo "   ✅ Kamera Modülü - BAĞLI"
else
    echo -e "   ❌ [HATA] Kamera Algılanmadı! (rpicam-vid listeleme başarısız)"
    echo "      -> Şerit kabloyu (csi/dsi) kontrol et."
    exit 1
fi

echo "   🚀 Tüm Sistemler Hazır. Başlatılıyor..."
echo "-----------------------------------------------------------"

# --- 2. LOG ve DİZİN AYARLARI ---
LOG_DIR="$HOME/CELEBILER_USV/logs"
mkdir -p "$LOG_DIR"
DOCKER_LOG_SOURCE="$HOME/CELEBILER_USV/docker_workspace/logs"
if [ -d "$DOCKER_LOG_SOURCE" ]; then
    ln -sfn "$DOCKER_LOG_SOURCE" "$LOG_DIR/docker"
fi
LOG_FILE="$LOG_DIR/system_boot.log"

echo "Log Merkezi: $LOG_DIR"
echo "Başlatılıyor..."

# 3. TEMİZLİK & KAMERA BAŞLATMA (HOST)
sudo fuser -k 8888/tcp > /dev/null 2>&1
echo -e "${GREEN}[KAMERA]${NC} Port 8888 temizleniyor ve yayın başlatılıyor..."
sudo pkill -9 rpicam-vid || true
sudo nice -n -20 rpicam-vid -t 0 --codec mjpeg --inline --listen -o tcp://0.0.0.0:8888 --width 1280 --height 720 --framerate 25 --quality 95 2>&1 | grep --line-buffered -vE "^#|WARN CameraSensor|INFO Camera" > "$LOG_DIR/cam_host.log" &
sleep 2

# 4. DOCKER BAŞLATMA
echo -e "${GREEN}[DOCKER]${NC} Konteyner ($CONTAINER_NAME) Kontrol Ediliyor..."
if [ ! "$(docker ps -q -f name=$CONTAINER_NAME)" ]; then
    if [ "$(docker ps -aq -f name=$CONTAINER_NAME)" ]; then
        docker start $CONTAINER_NAME > /dev/null
        echo -e "${GREEN}[DOCKER]${NC} Konteyner Uyandırıldı."
    else
        echo -e "${RED}[HATA]${NC} Konteyner bulunamadı! Lütfen kurulum yapın."
        exit 1
    fi
fi

# 5. İÇ SCRİPTİ TETİKLEME
echo -e "${GREEN}[SİSTEM]${NC} Otonom Pilot ve Web Sunucular Başlatılıyor... (Mod: $USV_MODE)"
docker exec -e USV_MODE="$USV_MODE" -d $CONTAINER_NAME /bin/bash -c "/root/workspace/scripts/internal_start.sh"

# 6. BAĞLANTI KONTROLÜ (5 Saniye İzle)
echo -e "\n🔎 [KONTROL] Donanım Bağlantıları Bekleniyor (5sn)..."
end=$((SECONDS+5))

# Log dosyasını önceden oluştur (Hata vermemesi için)
touch "$LOG_DIR/docker/telemetry.log"

tail -n 0 -f "$LOG_DIR/docker/telemetry.log" | while read line; do
    if [[ "$line" == *"✅"* ]] || [[ "$line" == *"🎮"* ]] || [[ "$line" == *"💓"* ]]; then
        echo "   -> $line"
    fi
    if [ $SECONDS -ge $end ]; then
        pkill -P $$ tail
        break
    fi
done || true

# 7. FİNAL BİLGİ TABLOSU
sleep 1
clear
echo -e "${CYAN}"
echo -e "   ██████╗███████╗██╗     ███████╗██████╗ ██╗██╗     ███████╗██████╗     ██╗   ██╗███████╗██╗   ██╗"
echo -e "  ██╔════╝██╔════╝██║     ██╔════╝██╔══██╗██║██║     ██╔════╝██╔══██╗    ██║   ██║██╔════╝██║   ██║"
echo -e "  ██║     █████╗  ██║     █████╗  ██████╔╝██║██║     █████╗  ██████╔╝    ██║   ██║███████╗██║   ██║"
echo -e "  ██║     ██╔══╝  ██║     ██╔══╝  ██╔══██╗██║██║     ██╔══╝  ██╔══██╗    ██║   ██║╚════██║╚██╗ ██╔╝"
echo -e "  ╚██████╗███████╗███████╗███████╗██████╔╝██║███████╗███████╗██║  ██║    ╚██████╔╝███████║ ╚████╔╝ "
echo -e "   ╚═════╝╚══════╝╚══════╝╚══════╝╚═════╝ ╚═╝╚══════╝╚══════╝╚═╝  ╚═╝     ╚═════╝ ╚══════╝  ╚═══╝ "
echo -e "${NC}"
echo -e "${BLUE}  ══════════════════════════════════════════════════════════════════════════════════════════${NC}"
echo -e "${BOLD}  🚀  MISSION CONTROL CENTER${NC}                          ${DIM}SYSTEM TIME: $(date +'%H:%M:%S')${NC}"
echo -e "${BLUE}  ══════════════════════════════════════════════════════════════════════════════════════════${NC}"
echo ""

echo -e "  ${BOLD}${WHITE}STATUS MODULES:${NC}"
echo -e "  ┌───────────────────────────┐  ┌──────────────────────────────────────────────────┐"
echo -e "  │  ${GREEN}● SYSTEM ONLINE${NC}          │  │  ${BOLD}📡 TELEMETRY & CONTROL LINK${NC}                     │"
echo -e "  │  ${BLUE}● DOCKER ENGINE${NC}          │  │                                                  │"
echo -e "  │  ${CYAN}● SENSORS:${NC}     2 ACTIVE  │  │   DASHBOARD URL:                                 │"

# IP Listesi (Dinamik Hizalama)
first_ip=true
for ip in $CURRENT_IPS; do
    if [[ $ip != "127."* ]]; then
        if [ "$first_ip" = true ]; then
            printf "  │                           │  │   👉 ${GREEN}http://%-28s${NC}      │\n" "$ip:8080"
            first_ip=false
        else
            printf "  │                           │  │      ${DIM}http://%-28s${NC}      │\n" "$ip:8080"
        fi
    fi
done

# Eğer hiç IP yoksa
if [ "$first_ip" = true ]; then
     printf "  │                           │  │   ${RED}NO NETWORK CONNECTION${NC}                          │\n"
fi

echo -e "  └───────────────────────────┘  │  ${BOLD}MOD:${NC} ${USV_MODE^^}                                        │"
echo -e "                                 └──────────────────────────────────────────────────┘"
echo ""
echo -e "  ${BOLD}${WHITE}SUBSYSTEMS:${NC}"
echo -e "  ╔══════════════════════════════════════════╗"
if [ "$USV_MODE" = "race" ]; then
    echo -e "  ║  🏁  YARIŞMA MODU - Görüntü kapalı (IDA 3.7) ║"
    echo -e "  ║  🎥  CAMERA FEED  : ${YELLOW}Off${NC} (Yasak)        ║"
    echo -e "  ║  🗺️   LIDAR MAP    : ${YELLOW}Off${NC} (Yasak)        ║"
else
    echo -e "  ║  🎥  CAMERA FEED  : ${GREEN}Active${NC} (Port 5000)   ║"
    echo -e "  ║  🗺️   LIDAR MAP    : ${GREEN}Active${NC} (Port 5001)   ║"
fi
echo -e "  ╚══════════════════════════════════════════╝"
echo ""
echo -e "${DIM}  [COMMANDS]  Type 'stop' to shutdown system  |  Type 'loglar' to view real-time logs${NC}"
echo ""
