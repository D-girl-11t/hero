#!/bin/sh
# Copy the Linux app (and the M33 firmware, if built) to the board and restart the services.
#   ./linux/install.sh [root@192.168.1.230] [path/to/hero_m33.elf]
set -eu
BOARD=${1:-root@192.168.1.230}
ELF=${2:-}
HERE=$(cd "$(dirname "$0")" && pwd)

# The board's /root/hero may hold newer work (voice, LLM, camera code) that isn't in
# this repo yet. Don't overwrite it unless it came from here or FORCE=1.
if ssh "$BOARD" '[ -f /root/hero/hero.py ] && [ ! -f /root/hero/.from-repo ]' && [ "${FORCE:-0}" != 1 ]; then
  echo "The board has its own /root/hero/hero.py that wasn't installed from this repo." >&2
  echo "Copy it into the repo and merge first, or run with FORCE=1 (a backup is kept)." >&2
  exit 1
fi

echo "-> app to $BOARD:/root/hero"
ssh "$BOARD" 'mkdir -p /root/hero && cp -a /root/hero /root/hero.bak-$(date +%Y%m%d%H%M) 2>/dev/null || true'
scp -q "$HERE/hero/"*.py "$HERE/hero/index.html" "$BOARD:/root/hero/"
ssh "$BOARD" 'touch /root/hero/.from-repo'
scp -q "$HERE/systemd/hero.service" "$BOARD:/etc/systemd/system/"

if [ -n "$ELF" ]; then
  echo "-> firmware $ELF to /lib/firmware/hero_m33.elf"
  scp -q "$ELF" "$BOARD:/lib/firmware/hero_m33.elf"
  scp -q "$HERE/systemd/hero-m33.service" "$BOARD:/etc/systemd/system/"
  ssh "$BOARD" 'systemctl daemon-reload && systemctl enable hero-m33.service && systemctl restart hero-m33.service'
fi

ssh "$BOARD" 'systemctl daemon-reload && systemctl enable hero.service && systemctl restart hero.service && sleep 1 && systemctl --no-pager -n 5 status hero.service'
echo "Done. Open the app with: ssh -f -N -L 8080:localhost:8080 $BOARD  then http://localhost:8080"
