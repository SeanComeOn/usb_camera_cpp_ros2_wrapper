#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
RULE_NAME="99-orin-usb-camera-stability.rules"
RULE_SOURCE="${PROJECT_ROOT}/config/udev/${RULE_NAME}"
RULE_DEST="/etc/udev/rules.d/${RULE_NAME}"
GUARD_SOURCE="${PROJECT_ROOT}/scripts/orin_usb_camera_guard.py"
GUARD_DEST="/usr/local/sbin/orin-usb-camera-guard"
SERVICE_NAME="orin-usb-camera-guard.service"
SERVICE_SOURCE="${PROJECT_ROOT}/config/systemd/${SERVICE_NAME}"
SERVICE_DEST="/etc/systemd/system/${SERVICE_NAME}"
DEFAULT_SOURCE="${PROJECT_ROOT}/config/default/orin-usb-camera-guard"
DEFAULT_DEST="/etc/default/orin-usb-camera-guard"

case "$(hostname)" in
  beside*)
    DEFAULT_SOURCE="${PROJECT_ROOT}/config/default/beside-usb-camera-guard"
    ;;
esac

for required_file in \
  "${RULE_SOURCE}" \
  "${GUARD_SOURCE}" \
  "${SERVICE_SOURCE}" \
  "${DEFAULT_SOURCE}"; do
  if [[ ! -f "${required_file}" ]]; then
    echo "Missing stability file: ${required_file}" >&2
    exit 1
  fi
done

sudo install -m 0644 "${RULE_SOURCE}" "${RULE_DEST}"
sudo install -m 0755 "${GUARD_SOURCE}" "${GUARD_DEST}"
sudo install -m 0644 "${SERVICE_SOURCE}" "${SERVICE_DEST}"
sudo install -m 0644 "${DEFAULT_SOURCE}" "${DEFAULT_DEST}"
sudo udevadm control --reload-rules

updated=0
for usb_device in /sys/bus/usb/devices/*; do
  [[ -f "${usb_device}/idVendor" && -f "${usb_device}/idProduct" ]] || continue
  vendor_id="$(<"${usb_device}/idVendor")"
  product_id="$(<"${usb_device}/idProduct")"
  case "${vendor_id}:${product_id}" in
    1bcf:0b15|1e45:0209|0bda:5883|0bda:5489|0bda:5411|0bda:0489|0bda:0411)
      if [[ -w "${usb_device}/power/control" ]]; then
        echo on > "${usb_device}/power/control"
      else
        echo on | sudo tee "${usb_device}/power/control" >/dev/null
      fi
      echo "Disabled autosuspend for ${vendor_id}:${product_id} at ${usb_device}."
      updated=$((updated + 1))
      ;;
  esac
done

sudo systemctl daemon-reload
sudo systemctl enable --now "${SERVICE_NAME}"
sudo systemctl restart "${SERVICE_NAME}"

echo "Installed USB camera stability support:"
echo "  profile: ${DEFAULT_SOURCE}"
echo "  udev rule: ${RULE_DEST}"
echo "  guard: ${GUARD_DEST}"
echo "  service: ${SERVICE_NAME}"
echo "Updated ${updated} currently connected camera/hub device(s)."
echo "Inspect recovery events with: journalctl -u ${SERVICE_NAME}"
