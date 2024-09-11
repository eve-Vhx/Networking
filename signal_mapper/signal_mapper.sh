
#!/bin/bash

DEVICE="/dev/ttyUSB2"
BAUDRATE=115200
LOGS_DIR="logs"
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
OUTPUT_FILE="${LOGS_DIR}/signal_strength_log_${TIMESTAMP}.csv"
METRICS_FILE="${LOGS_DIR}/signal_metrics_log_${TIMESTAMP}.csv"
INTERVAL=1  # Interval in seconds

# Create logs directory if it doesn't exist
mkdir -p "$LOGS_DIR"

# Function to send AT command and read response
send_at_command() {
    command=$1
    timeout=$2

    # Set up the serial port
    stty -F $DEVICE $BAUDRATE raw -echo

    # Send the command
    echo -e "$command\r" > $DEVICE

    # Read the response
    response=""
    end_time=$((SECONDS + timeout))
    while [ $SECONDS -lt $end_time ]; do
        if read -t 1 -r line < $DEVICE; then
            response+="$line"$'\n'
            if [[ $response == *"OK"* || $response == *"ERROR"* ]]; then
                break
            fi
        fi
    done

    echo "$response"
}

# Function to get network info
get_network_info() {
    network_info=$(send_at_command "AT+QNWINFO" 1)
    echo "$network_info" | grep "+QNWINFO:" | cut -d ' ' -f 2- | tr ',' ';'
}

# Function to get signal info
get_signal_info() {
    signal_info=$(send_at_command 'AT+QENG="servingcell"' 1)
    echo "$signal_info" | grep "+QENG:" | cut -d ' ' -f 2- | tr ',' ';'
}

# Function to get signal quality
get_signal_quality() {
    signal_quality=$(send_at_command "AT+CSQ" 1)
    echo "$signal_quality" | grep "+CSQ:" | cut -d ' ' -f 2- | tr ',' ';'
}

# Function to extract RSSI from AT+CSQ response
get_servingcell_rssi() {
    csq_response=$1
    rssi=$(echo $csq_response | awk -F';' '{print $1}')
    if [ "$rssi" -ne "99" ]; then
        echo $((2 * rssi - 113))
    else
        echo "N/A"
    fi
}

# Function to extract metrics from AT+QENG response
get_metrics() {
    qeng_response=$1
    IFS=';' read -ra fields <<< "$qeng_response"
    rsrp=${fields[13]}
    rsrq=${fields[14]}
    rssi=${fields[15]}
    echo "$rsrp;$rsrq;$rssi"
}

# Create CSV headers
echo "Timestamp,Network Info,Signal Info,Signal Quality" > "$OUTPUT_FILE"
echo "Timestamp,servingcell_RSSI,RSSI,RSRP,RSRQ" > "$METRICS_FILE"

echo "Logging data to:"
echo "  $OUTPUT_FILE"
echo "  $METRICS_FILE"

# Main loop
while true; do
    timestamp=$(date +"%Y-%m-%d %H:%M:%S")
    network_info=$(get_network_info)
    signal_info=$(get_signal_info)
    signal_quality=$(get_signal_quality)

    # Format CSV line with proper quoting
    csv_line="\"$timestamp\",\"$network_info\",\"$signal_info\",\"$signal_quality\""

    echo "$csv_line" >> "$OUTPUT_FILE"

    # Extract and log metrics
    servingcell_rssi=$(get_servingcell_rssi "$signal_quality")
    metrics=$(get_metrics "$signal_info")
    IFS=';' read -r rsrp rsrq rssi <<< "$metrics"
    echo "\"$timestamp\",\"$servingcell_rssi\",\"$rssi\",\"$rsrp\",\"$rsrq\"" >> "$METRICS_FILE"

    echo "Logged at $timestamp"
    echo "Network Info: $network_info"
    echo "Signal Info: $signal_info"
    echo "Signal Quality: $signal_quality"
    echo "Metrics - servingcell_RSSI: $servingcell_rssi, RSSI: $rssi, RSRP: $rsrp, RSRQ: $rsrq"
    echo "------------------------"

    sleep $INTERVAL
done