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
    # Kullanıcı isteği: Hepsini cat ile dök
    
    for logfile in $LOG_DIR/*.log; do
        if [ -f "$logfile" ]; then
            echo -e "\n${CYAN}==> $(basename "$logfile") <==${NC}"
            cat "$logfile"
        fi
    done
    
    if [ -d "$LOG_DIR/docker" ]; then
        for logfile in $LOG_DIR/docker/*.log; do
            if [ -f "$logfile" ]; then
                echo -e "\n${CYAN}==> docker/$(basename "$logfile") <==${NC}"
                cat "$logfile"
            fi
        done
    fi
}

case "$1" in
    setup)
        setup_alias
        ;;
    *)
        view_all
        ;;
esac
