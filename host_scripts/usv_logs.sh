#!/bin/bash
# USV Log Viewer & Setup
# Konum: CELEBILER_USV/host_scripts/usv_logs.sh

LOG_DIR="$HOME/CELEBILER_USV/logs"

# Renkler
GREEN='\033[0;32m'
CYAN='\033[0;36m'
NC='\033[0m'

function setup_alias() {
    echo -e "${CYAN}[SETUP]${NC} 'loglar' kısayolu ekleniyor..."
    if grep -q "alias loglar=" ~/.bashrc; then
        echo -e "${GREEN}✅ Kısayol zaten mevcut.${NC}"
    else
        echo "alias loglar='bash $HOME/CELEBILER_USV/host_scripts/usv_logs.sh'" >> ~/.bashrc
        echo -e "${GREEN}✅ Kısayol eklendi! Lütfen 'source ~/.bashrc' çalıştırın.${NC}"
    fi
}

function view_all() {
    echo -e "${CYAN}📡 LOG İZLEME MERKEZİ ($LOG_DIR)${NC}"
    echo "------------------------------------------------"
    echo "Host Logları: system_boot.log, cam_host.log"
    echo "Docker Logları: logs/docker/*.log"
    echo "------------------------------------------------"
    
    # Host logları ve Docker (symlink) loglarını izle
    # docker klasörü varsa oradaki .log dosyalarını da dahil et
    FILES="$LOG_DIR/*.log"
    if [ -d "$LOG_DIR/docker" ]; then
        FILES="$FILES $LOG_DIR/docker/*.log"
    fi
    
    # Hata bastırma (dosya yoksa) ve tail başlatma
    tail -f $FILES 2>/dev/null
}

case "$1" in
    setup)
        setup_alias
        ;;
    *)
        view_all
        ;;
esac
