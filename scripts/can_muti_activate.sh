#!/bin/bash
declare -A USB_PORTS 

USB_PORTS["1-6.1:1.0"]="can0:1000000"
USB_PORTS["1-6.2:1.0"]="can1:1000000"
USB_PORTS["1-6.3:1.0"]="can2:1000000"
USB_PORTS["1-6.4:1.0"]="can3:1000000"
USB_PORTS["1-2:1.0"]="can4:1000000"

# Whether to ignore CAN quantity check (default false)
IGNORE_CHECK=false

# Parsing parameters
for arg in "$@"; do
    if [ "$arg" == "--ignore" ]; then
        IGNORE_CHECK=true
    fi
done

# Step 1: print USB_PORTS mapping and check for duplicate target names
echo "🔧 Checking USB_PORTS configuration:"
declare -A TARGET_NAMES_COUNT
LINE_NUM=0
HAS_DUPLICATE=false

for k in "${!USB_PORTS[@]}"; do
    LINE_NUM=$((LINE_NUM + 1))
    IFS=':' read -r name bitrate <<< "${USB_PORTS[$k]}"
    
    # detect duplicate target names
    if [[ -n "${TARGET_NAMES_COUNT[$name]}" ]]; then
        echo "→ [$LINE_NUM] \"$k\"=\"${USB_PORTS[$k]}\"  ❌ Duplicate target CAN name: '$name'"
        HAS_DUPLICATE=true
    else
        echo "  [$LINE_NUM] \"$k\"=\"${USB_PORTS[$k]}\""
        TARGET_NAMES_COUNT["$name"]=1
    fi
done

if $HAS_DUPLICATE; then
    echo "❌ [ERROR]: Found duplicate target CAN interface name(s) above. Please resolve before proceeding."
    exit 1
fi

PREDEFINED_COUNT=${#USB_PORTS[@]}
CURRENT_CAN_COUNT=$(ip link show type can | grep -c "link/can")

if [ "$IGNORE_CHECK" = false ] && [ "$CURRENT_CAN_COUNT" -ne "$PREDEFINED_COUNT" ]; then
    echo "[WARN]: The detected number of CAN modules ($CURRENT_CAN_COUNT) does not match the expected number ($PREDEFINED_COUNT)."
    read -p "Do you want to continue? (y/N): " user_input
    case "$user_input" in
        [yY]|[yY][eE][sS])
            echo "Continue execution..."
            ;;
        *)
            echo "Exited."
            exit 1
            ;;
    esac
else
    echo "CAN quantity check ignored or matched, continuing..."
fi

# Load the gs_usb module
sudo modprobe gs_usb
if [ $? -ne 0 ]; then
    echo "[ERROR]: Unable to load gs_usb module."
    exit 1
fi

# Auto-restart after bus-off (ms). 0 disables.
RESTART_MS="${RESTART_MS:-100}"
# tx queue length for burst traffic
TX_QUEUE_LEN="${TX_QUEUE_LEN:-1024}"

configure_can_iface() {
    local iface="$1"
    local bitrate="$2"
    local restart_ms="$3"
    local txqlen="$4"

    sudo ip link set "$iface" down || return 1

    # Try restart-ms first (bus-off auto recovery). Some drivers may not support it.
    if sudo ip link set "$iface" type can bitrate "$bitrate" restart-ms "$restart_ms" 2>/dev/null; then
        :
    elif sudo ip link set "$iface" type can bitrate "$bitrate" 2>/dev/null; then
        :
    else
        return 1
    fi

    sudo ip link set "$iface" txqueuelen "$txqlen" 2>/dev/null || true
    sudo ip link set "$iface" up || return 1
    return 0
}

get_bus_info() {
    local iface="$1"
    sudo ethtool -i "$iface" 2>/dev/null | awk -F': *' '/^bus-info:/ {print $2}'
}

resolve_usb_port_key() {
    local bus_info="$1"
    local k=""
    local matched_key=""
    local matched_count=0

    if [ -n "${USB_PORTS[$bus_info]}" ]; then
        echo "$bus_info"
        return 0
    fi

    for k in "${!USB_PORTS[@]}"; do
        if [[ "$k" == "$bus_info":* ]]; then
            matched_key="$k"
            matched_count=$((matched_count + 1))
        fi
    done

    if [ "$matched_count" -eq 1 ]; then
        echo "$matched_key"
        return 0
    fi

    return 1
}

is_iface_admin_up() {
    local iface="$1"
    ip -o link show "$iface" | grep -qE '<[^>]*\bUP\b'
}

find_iface_by_usb_key() {
    local target_key="$1"
    local iface=""
    local bus_info=""
    local resolved_key=""

    for iface in $(ip -br link show type can | awk '{print $1}'); do
        bus_info=$(get_bus_info "$iface")
        if resolved_key=$(resolve_usb_port_key "$bus_info"); then
            if [ "$resolved_key" = "$target_key" ]; then
                echo "$iface"
                return 0
            fi
        fi
    done

    return 1
}

# Safely rename interface to target name.
# Supports direct swap case like can0 <-> can1 by using a temporary name.
safe_rename_iface() {
    local src_iface="$1"
    local target_iface="$2"

    if [ "$src_iface" = "$target_iface" ]; then
        return 0
    fi

    # Target name does not exist, normal rename.
    if ! ip link show "$target_iface" &>/dev/null; then
        sudo ip link set "$src_iface" down || return 1
        sudo ip link set "$src_iface" name "$target_iface" || return 1
        sudo ip link set "$target_iface" up || return 1
        return 0
    fi

    # Target exists: detect whether it's a direct swap.
    local occupied_iface="$target_iface"
    local occupied_bus=""
    local occupied_bus_key=""
    local occupied_target=""
    occupied_bus=$(get_bus_info "$occupied_iface")

    if occupied_bus_key=$(resolve_usb_port_key "$occupied_bus"); then
        IFS=':' read -r occupied_target _ <<< "${USB_PORTS[$occupied_bus_key]}"
    fi

    # Only handle direct swap (A->B and B->A). Otherwise keep safe fail.
    if [ "$occupied_target" != "$src_iface" ]; then
        return 2
    fi

    local tmp_iface="tmp_${src_iface}_$RANDOM"
    if ip link show "$tmp_iface" &>/dev/null; then
        tmp_iface="tmp_${src_iface}_$$"
    fi

    sudo ip link set "$occupied_iface" down || return 1
    sudo ip link set "$src_iface" down || return 1
    sudo ip link set "$occupied_iface" name "$tmp_iface" || return 1
    sudo ip link set "$src_iface" name "$target_iface" || return 1
    sudo ip link set "$tmp_iface" name "$src_iface" || return 1
    sudo ip link set "$target_iface" up || return 1
    sudo ip link set "$src_iface" up || return 1
    return 0
}

# Rename multiple interfaces whose target names form a cycle (e.g. can1->can0, can0->can4).
# Uses temporary names so every source can reach its target without collision.
batch_rename_pending_interfaces() {
    declare -A RENAME_SRC_TO_TARGET
    declare -A RENAME_SRC_TO_TEMP
    local src target temp k IFACE TARGET_NAME

    for k in "${!USB_PORTS[@]}"; do
        IFS=':' read -r TARGET_NAME _ <<< "${USB_PORTS[$k]}"
        IFACE=$(find_iface_by_usb_key "$k" || true)
        if [ -n "$IFACE" ] && [ "$IFACE" != "$TARGET_NAME" ]; then
            RENAME_SRC_TO_TARGET["$IFACE"]="$TARGET_NAME"
        fi
    done

    if [ ${#RENAME_SRC_TO_TARGET[@]} -eq 0 ]; then
        return 0
    fi

    echo "[INFO]: Batch renaming ${#RENAME_SRC_TO_TARGET[@]} interface(s) via temporary names..."

    for src in "${!RENAME_SRC_TO_TARGET[@]}"; do
        temp="tmp_${src}_$$"
        while ip link show "$temp" &>/dev/null; do
            temp="tmp_${src}_${RANDOM}"
        done
        RENAME_SRC_TO_TEMP["$src"]="$temp"
        echo "[INFO]: Temp rename '$src' -> '$temp'"
        sudo ip link set "$src" down || return 1
        sudo ip link set "$src" name "$temp" || return 1
    done

    for src in "${!RENAME_SRC_TO_TARGET[@]}"; do
        temp="${RENAME_SRC_TO_TEMP[$src]}"
        target="${RENAME_SRC_TO_TARGET[$src]}"
        echo "[INFO]: Final rename '$temp' -> '$target'"
        sudo ip link set "$temp" name "$target" || return 1
        sudo ip link set "$target" up || return 1
    done

    return 0
}

SUCCESS_COUNT=0  # Number of CAN interfaces successfully processed
FAILED_COUNT=0   # Expected number of interfaces that failed or were not processed

# Copy a list of USB_PORTS keys and mark each one for success
declare -A USB_PORT_STATUS
for k in "${!USB_PORTS[@]}"; do
    USB_PORT_STATUS["$k"]="pending"
done
# Handle multiple CAN modules
# Iterate over all CAN interfaces
SYS_INTERFACE=$(ip -br link show type can | awk '{print $1}')

echo -e "\n🔍 [INFO]: The following CAN interfaces were detected in the system:"
for iface in $SYS_INTERFACE; do
    echo "  - $iface"
done

echo -e "\n⚠️  [HINT]: Please make sure none of the above interface names conflict with the predefined names in your USB_PORTS config."

for iface in $SYS_INTERFACE; do
    # Get bus-info using ethtool
    echo "--------------------------- $iface ------------------------------"
    BUS_INFO=$(get_bus_info "$iface")
    BUS_INFO_KEY=""
    
    if [ -z "$BUS_INFO" ];then
        echo "[ERROR]: Unable to get bus-info information for interface '$iface'."
        echo "[DEBUG]: ethtool -i output for '$iface':"
        sudo ethtool -i "$iface" 2>/dev/null | sed 's/^/        /'
        continue
    fi
    
    echo "[INFO]: System interface '$iface' is plugged into USB port '$BUS_INFO'"
    # Check if bus-info is in the list of predefined USB ports
    if BUS_INFO_KEY=$(resolve_usb_port_key "$BUS_INFO"); then
        IFS=':' read -r TARGET_NAME TARGET_BITRATE <<< "${USB_PORTS[$BUS_INFO_KEY]}"
        
        # Check if the current interface is activated
        IS_LINK_UP=$(is_iface_admin_up "$iface" && echo "yes" || echo "no")

        # Get the bit rate of the current interface
        CURRENT_BITRATE=$(ip -details link show "$iface" | grep -oP 'bitrate \K\d+' || echo "")
        
        if [ "$IS_LINK_UP" = "yes" ] && [ -n "$CURRENT_BITRATE" ] && [ "$CURRENT_BITRATE" -eq "$TARGET_BITRATE" ]; then
            echo "[INFO]: Interface '$iface' is activated and bitrate is $TARGET_BITRATE"
            
            # Check if the interface name matches the target name
            if [ "$iface" != "$TARGET_NAME" ]; then
                echo "[INFO]: Rename interface '$iface' to '$TARGET_NAME'"
                if safe_rename_iface "$iface" "$TARGET_NAME"; then
                    echo "[INFO]: The interface was renamed to '$TARGET_NAME' and reactivated."
                else
                    rc=$?
                    if [ "$rc" -eq 2 ]; then
                        echo "[WARN]: Cannot rename '$iface' to '$TARGET_NAME' because target name is occupied by another interface with different mapping."
                    else
                        echo "[ERROR]: Failed to rename '$iface' to '$TARGET_NAME'."
                    fi
                    continue
                fi
            else
                echo "[INFO]: The USB port '$BUS_INFO' interface name is already '$TARGET_NAME'"
            fi
        else
            # If the interface name already matches the target name, just check activation
            if [ "$iface" = "$TARGET_NAME" ]; then
                # Name already matches, just need to check activation and bitrate
                if [ "$IS_LINK_UP" = "no" ]; then
                    echo "[INFO]: Interface '$iface' name matches target, but is not activated. Activating..."
                    if configure_can_iface "$iface" "$TARGET_BITRATE" "$RESTART_MS" "$TX_QUEUE_LEN"; then
                        echo "[INFO]: Interface '$iface' has been activated with bitrate $TARGET_BITRATE."
                    else
                        echo "[ERROR]: Failed to activate '$iface' (bitrate=$TARGET_BITRATE)."
                        continue
                    fi
                elif [ -n "$CURRENT_BITRATE" ] && [ "$CURRENT_BITRATE" -ne "$TARGET_BITRATE" ]; then
                    echo "[INFO]: Interface '$iface' is activated, but bitrate $CURRENT_BITRATE does not match target $TARGET_BITRATE. Updating..."
                    if configure_can_iface "$iface" "$TARGET_BITRATE" "$RESTART_MS" "$TX_QUEUE_LEN"; then
                        echo "[INFO]: Interface '$iface' bitrate has been updated to $TARGET_BITRATE."
                    else
                        echo "[ERROR]: Failed to reconfigure '$iface' (bitrate=$TARGET_BITRATE)."
                        continue
                    fi
                else
                    echo "[INFO]: Interface '$iface' is already activated with correct name and bitrate. Skipping."
                fi
            else
                # Interface name doesn't match, and target name is available - proceed with rename
                # If the interface is not active or the bit rate is different, set
                if [ "$IS_LINK_UP" = "yes" ]; then
                    echo "[INFO]: Interface '$iface' is activated, but the bitrate $CURRENT_BITRATE does not match the set $TARGET_BITRATE."
                else
                    echo "[INFO]: Interface '$iface' is not activated or the bitrate is not set."
                fi
                
                # Set the interface bit rate and activate it
                if configure_can_iface "$iface" "$TARGET_BITRATE" "$RESTART_MS" "$TX_QUEUE_LEN"; then
                    echo "[INFO]: Interface '$iface' has been reset to bitrate $TARGET_BITRATE and activated."
                else
                    echo "[ERROR]: Failed to configure '$iface' (bitrate=$TARGET_BITRATE)."
                    continue
                fi
                
                # Rename the interface to the target name
                if [ "$iface" != "$TARGET_NAME" ]; then
                    echo "[INFO]: Rename interface $iface to '$TARGET_NAME'"
                    if safe_rename_iface "$iface" "$TARGET_NAME"; then
                        echo "[INFO]: The interface was renamed to '$TARGET_NAME' and reactivated."
                    else
                        rc=$?
                        if [ "$rc" -eq 2 ]; then
                            echo "[WARN]: Cannot rename '$iface' to '$TARGET_NAME' because interface '$TARGET_NAME' already exists and is not a direct swap pair."
                            echo "[HINT]: Please check if another interface already occupies this name, or fix your USB_PORTS configuration."
                        else
                            echo "[ERROR]: Failed to rename '$iface' to '$TARGET_NAME'."
                        fi
                        echo "-----------------------------------------------------------------"
                        continue
                    fi
                fi
            fi
        fi
        USB_PORT_STATUS["$BUS_INFO_KEY"]="success"
    else
        # echo "↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓---err---↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓"
        echo "[ERROR]: The USB port '$BUS_INFO' of interface '$iface' was not found in the predefined USB_PORTS list."
        echo "[INFO]: Current predefined USB_PORTS configuration:"
        for k in "${!USB_PORTS[@]}"; do
            echo "        '$k'"
        done
        echo "[HINT]: Please check if the USB device is inserted into the correct port, or update the USB_PORTS config if needed."
        # echo "↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑---err---↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑"
    fi
    echo "-----------------------------------------------------------------"
done

echo -e "\n🔁 [INFO]: Final reconciliation for pending USB ports..."
if ! batch_rename_pending_interfaces; then
    echo "[WARN]: Batch rename encountered errors; continuing with per-port reconciliation."
fi

for k in "${!USB_PORT_STATUS[@]}"; do
    if [ "${USB_PORT_STATUS[$k]}" = "success" ]; then
        continue
    fi

    IFS=':' read -r TARGET_NAME TARGET_BITRATE <<< "${USB_PORTS[$k]}"
    IFACE_FOUND=$(find_iface_by_usb_key "$k" || true)

    if [ -z "$IFACE_FOUND" ]; then
        continue
    fi

    CURRENT_BITRATE=$(ip -details link show "$IFACE_FOUND" | grep -oP 'bitrate \K\d+' || echo "")
    IS_LINK_UP=$(is_iface_admin_up "$IFACE_FOUND" && echo "yes" || echo "no")

    if [ "$IFACE_FOUND" != "$TARGET_NAME" ]; then
        echo "[INFO]: Reconcile rename '$IFACE_FOUND' -> '$TARGET_NAME' for USB '$k'"
        if safe_rename_iface "$IFACE_FOUND" "$TARGET_NAME"; then
            IFACE_FOUND="$TARGET_NAME"
            CURRENT_BITRATE=$(ip -details link show "$IFACE_FOUND" | grep -oP 'bitrate \K\d+' || echo "")
            IS_LINK_UP=$(is_iface_admin_up "$IFACE_FOUND" && echo "yes" || echo "no")
        else
            continue
        fi
    fi

    if [ "$IS_LINK_UP" != "yes" ] || [ -z "$CURRENT_BITRATE" ] || [ "$CURRENT_BITRATE" -ne "$TARGET_BITRATE" ]; then
        if ! configure_can_iface "$IFACE_FOUND" "$TARGET_BITRATE" "$RESTART_MS" "$TX_QUEUE_LEN"; then
            continue
        fi
    fi

    USB_PORT_STATUS["$k"]="success"
done

# Recalculate success count from final status to avoid double counting after rename/swap.
SUCCESS_COUNT=0
for k in "${!USB_PORT_STATUS[@]}"; do
    if [ "${USB_PORT_STATUS[$k]}" = "success" ]; then
        SUCCESS_COUNT=$((SUCCESS_COUNT+1))
    fi
done

# Calculation failed USB port
for k in "${!USB_PORT_STATUS[@]}"; do
    if [ "${USB_PORT_STATUS[$k]}" != "success" ]; then
        echo "❌ Expected CAN interface on USB port '$k' was not found or not activated."
        FAILED_COUNT=$((FAILED_COUNT+1))
    fi
done

# Final Tips
if [ "$SUCCESS_COUNT" -gt 0 ]; then
    echo "[RESULT]: ✅ $SUCCESS_COUNT expected CAN interfaces processed successfully."
else
    echo "[RESULT]: ❌ No USB interface matches the preset CAN configuration, please check whether the USB port is connected correctly."
fi

if [ "$FAILED_COUNT" -gt 0 ]; then
    echo "[RESULT]: 🚫 $FAILED_COUNT expected CAN interfaces failed to activate or were not found."
fi
