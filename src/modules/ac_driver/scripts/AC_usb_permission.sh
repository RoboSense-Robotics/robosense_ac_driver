#!/bin/bash
set -euo pipefail  # Strict mode: exit on error, undefined variable, or pipe failure

##############################################################################
# Configure USB permissions for AC devices
##############################################################################
declare -a DEVICES=(
    "3840 1010 AC1 USB Device"
    "3840 0000 AC1 RNDIS Device"
    "3840 1020 AC2 USB Device"
)
RULE_FILE="/etc/udev/rules.d/99-usb-RS-AC-permissions.rules"
BACKUP_FILE="${RULE_FILE}.bak.$(date +%Y%m%d%H%M%S)"


##############################################################################
# Functions
##############################################################################

# show help message
show_help() {
    cat << EOF
Usage: sudo $0 [OPTION]

Manage udev rules for AC1/AC2 USB devices (grant/revoke user access permissions).

Options:
  -h, --help      Show this help message and exit
  -i, --install   Install udev rules: grant read/write access (MODE=0666) to all users (default)
  -c, --clean     Clean up udev rules: delete installed rules and restore default permissions

Examples:
  sudo $0                # Default: Install udev rules
  sudo $0 -i             # Explicitly install udev rules
  sudo $0 --install      # Explicitly install udev rules
  sudo $0 -c             # Clean up installed udev rules
  sudo $0 --clean        # Clean up installed udev rules
  sudo $0 -h             # Show help information
EOF
}

# check if the script is run as root
check_root() {
    if [ "$(id -u)" -ne 0 ]; then
        echo "Error: This operation requires root privileges. Run with 'sudo'." >&2
        exit 1
    fi
}

# apply udev rules
apply_udev_config() {
    local action="$1"
    echo -e "\nApplying udev configuration (after $action)..."
    
    udevadm control --reload-rules
    udevadm trigger --subsystem-match=usb
    systemctl restart systemd-udevd

    if systemctl is-active --quiet systemd-udevd; then
        echo "Success: udev service restarted normally."
    else
        echo "Warning: udev service restart failed. Reconnect your USB device or reboot to apply changes." >&2
    fi
}

# install udev rules
install_rules() {
    echo "=== Starting udev rule installation for AC1/AC2 USB devices ==="

    > "$RULE_FILE"
    echo "Cleared existing rule file (to avoid duplicates): $RULE_FILE"

    for device in "${DEVICES[@]}"; do
        local vid=$(echo "$device" | awk '{print $1}')
        local pid=$(echo "$device" | awk '{print $2}')
        local desc=$(echo "$device" | cut -d' ' -f3-)
        
        echo "Adding rule: $desc (VID: $vid, PID: $pid)"
        echo "SUBSYSTEM==\"usb\", ATTR{idVendor}==\"$vid\", ATTR{idProduct}==\"$pid\", MODE=\"0666\"" >> "$RULE_FILE"
    done

    apply_udev_config "install"

    echo -e "\n=== Installation Complete ==="
    echo "1. Udev rules installed to: $RULE_FILE"
    echo "2. All users now have read/write access to AC1/AC2 devices."
    echo "3. Note: Reconnect your USB device if it was already plugged in."
}

# clean udev rules
clean_rules() {
    echo "=== Starting udev rule cleanup for AC1/AC2 USB devices ==="

    if [ ! -f "$RULE_FILE" ]; then
        echo "Info: No installed udev rules found (already cleaned or never installed):"
        echo "      $RULE_FILE"
        exit 0
    fi

    echo "Step 1: Backing up existing rule file..."
    cp --preserve=mode,ownership "$RULE_FILE" "$BACKUP_FILE"
    if [ -f "$BACKUP_FILE" ]; then
        echo "Success: Backup saved to: $BACKUP_FILE"
    else
        echo "Warning: Failed to back up the rule file. Continue deletion? (y/N)" >&2
        read -r confirm
        if [[ ! "$confirm" =~ ^[Yy]$ ]]; then
            echo "Aborted: Cleanup operation canceled by user."
            exit 1
        fi
    fi

    echo -e "\nStep 2: Deleting installed rule file..."
    rm -f "$RULE_FILE"
    if [ ! -f "$RULE_FILE" ]; then
        echo "Success: Rule file deleted: $RULE_FILE"
    else
        echo "Error: Failed to delete rule file. Please delete manually: $RULE_FILE" >&2
        exit 1
    fi

    apply_udev_config "clean"

    echo -e "\n=== Cleanup Complete ==="
    echo "1. Installed udev rules have been deleted."
    echo "2. USB device permissions restored to system default."
    echo "3. Backup retained (recommend keeping for 7 days): $BACKUP_FILE"
    echo "4. To restore original rules: sudo cp $BACKUP_FILE $RULE_FILE && sudo udevadm control --reload-rules"
}


##############################################################################
# Main function
##############################################################################
main() {
    # default action is to install the rule
    local action="install"

    # parse command-line arguments
    while [[ $# -gt 0 ]]; do
        case "$1" in
            -h|--help)
                show_help
                exit 0
                ;;
            -i|--install)
                action="install"
                shift
                ;;
            -c|--clean)
                action="clean"
                shift
                ;;
            *)
                echo "Error: Invalid option '$1'." >&2
                echo "Run 'sudo $0 -h' for help." >&2
                exit 1
                ;;
        esac
    done

    # check root permission and execute the install or clean rules
    check_root
    if [ "$action" = "install" ]; then
        install_rules
    else
        clean_rules
    fi
}

main "$@"
