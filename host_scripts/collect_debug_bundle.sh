#!/bin/bash
# Debug Bundle Collector - CELEBILER USV
# Collects logs and artifacts for debugging
# Usage: ./collect_debug_bundle.sh [run_id]

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
LOGS_DIR="$PROJECT_ROOT/logs"
BUNDLE_DIR="$PROJECT_ROOT/debug_bundles"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)

# Create bundle directory
mkdir -p "$BUNDLE_DIR"

# Determine run_id
if [ -n "$1" ]; then
    RUN_ID="$1"
else
    # Try to find latest run from artifacts
    if [ -d "$LOGS_DIR/artifacts" ]; then
        RUN_ID=$(ls -t "$LOGS_DIR/artifacts"/*.json 2>/dev/null | head -1 | xargs basename 2>/dev/null | sed 's/run_\(.*\)\.json/\1/' || echo "unknown")
    else
        RUN_ID="unknown_$TIMESTAMP"
    fi
fi

BUNDLE_NAME="debug_bundle_${RUN_ID}_${TIMESTAMP}"
BUNDLE_PATH="$BUNDLE_DIR/$BUNDLE_NAME.tar.gz"

echo "=== CELEBILER USV Debug Bundle Collector ==="
echo "Run ID: $RUN_ID"
echo "Bundle: $BUNDLE_PATH"
echo ""

# Create temporary directory for collection
TEMP_DIR=$(mktemp -d)
trap "rm -rf $TEMP_DIR" EXIT

# Collect logs
echo "Collecting logs..."
if [ -d "$LOGS_DIR" ]; then
    # System logs
    if [ -d "$LOGS_DIR/system" ]; then
        mkdir -p "$TEMP_DIR/logs/system"
        find "$LOGS_DIR/system" -type f \( -name "*.log" -o -name "*.jsonl" -o -name "*.json" \) -exec cp {} "$TEMP_DIR/logs/system/" \; 2>/dev/null || true
    fi
    
    # Simulation logs
    if [ -d "$LOGS_DIR/simulation" ]; then
        mkdir -p "$TEMP_DIR/logs/simulation"
        find "$LOGS_DIR/simulation" -type f \( -name "*.log" -o -name "*.jsonl" -o -name "*.json" \) -exec cp {} "$TEMP_DIR/logs/simulation/" \; 2>/dev/null || true
    fi
    
    # Startup logs
    if [ -d "$LOGS_DIR/startup" ]; then
        mkdir -p "$TEMP_DIR/logs/startup"
        find "$LOGS_DIR/startup" -type f -exec cp {} "$TEMP_DIR/logs/startup/" \; 2>/dev/null || true
    fi
    
    # Shutdown logs
    if [ -d "$LOGS_DIR/shutdown" ]; then
        mkdir -p "$TEMP_DIR/logs/shutdown"
        find "$LOGS_DIR/shutdown" -type f -exec cp {} "$TEMP_DIR/logs/shutdown/" \; 2>/dev/null || true
    fi
    
    # Artifacts
    if [ -d "$LOGS_DIR/artifacts" ]; then
        mkdir -p "$TEMP_DIR/logs/artifacts"
        find "$LOGS_DIR/artifacts" -type f -name "*.json" -exec cp {} "$TEMP_DIR/logs/artifacts/" \; 2>/dev/null || true
    fi
fi

# Collect configuration
echo "Collecting configuration..."
mkdir -p "$TEMP_DIR/config"
if [ -d "$PROJECT_ROOT/config" ]; then
    find "$PROJECT_ROOT/config" -type f \( -name "*.json" -o -name "*.yaml" -o -name "*.yml" -o -name "*.param" \) -exec cp {} "$TEMP_DIR/config/" \; 2>/dev/null || true
fi

# Collect mission files
echo "Collecting mission files..."
mkdir -p "$TEMP_DIR/mission"
if [ -f "$PROJECT_ROOT/docker_workspace/mission.json" ]; then
    cp "$PROJECT_ROOT/docker_workspace/mission.json" "$TEMP_DIR/mission/"
fi

# Collect system info
echo "Collecting system info..."
mkdir -p "$TEMP_DIR/system"
{
    echo "=== System Information ==="
    echo "Timestamp: $(date -Iseconds)"
    echo "Hostname: $(hostname)"
    echo "User: $(whoami)"
    echo "PWD: $PROJECT_ROOT"
    echo ""
    echo "=== Git Status ==="
    cd "$PROJECT_ROOT" && git status --short 2>/dev/null || echo "Not a git repository"
    echo ""
    echo "=== Git Log (last 5) ==="
    cd "$PROJECT_ROOT" && git log --oneline -5 2>/dev/null || echo "No git history"
    echo ""
    echo "=== Python Version ==="
    python3 --version 2>/dev/null || echo "Python3 not found"
    echo ""
    echo "=== Disk Usage ==="
    df -h "$PROJECT_ROOT" 2>/dev/null || echo "Disk info unavailable"
    echo ""
    echo "=== Memory Usage ==="
    free -h 2>/dev/null || echo "Memory info unavailable"
} > "$TEMP_DIR/system/system_info.txt"

# Create manifest
echo "Creating manifest..."
cat > "$TEMP_DIR/manifest.json" <<EOF
{
  "run_id": "$RUN_ID",
  "timestamp": "$(date -Iseconds)",
  "hostname": "$(hostname)",
  "user": "$(whoami)",
  "project_root": "$PROJECT_ROOT",
  "bundle_name": "$BUNDLE_NAME"
}
EOF

# Create tarball
echo "Creating bundle..."
cd "$TEMP_DIR"
tar -czf "$BUNDLE_PATH" . 2>/dev/null

echo ""
echo "=== Bundle Created ==="
echo "Path: $BUNDLE_PATH"
echo "Size: $(du -h "$BUNDLE_PATH" | cut -f1)"
echo ""
echo "To extract: tar -xzf $BUNDLE_PATH"
