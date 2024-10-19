#!/bin/bash

# Initialize the FRANKA_IP to a default value
FRANKA_IP="172.16.0.255"

# Initialize a variable to track whether any long ping times were encountered
long_ping_found=false

# Loop through IP addresses from 172.16.0.2 to 172.16.0.99
for ((i=2; i<100; i++)); do
    ip="172.16.0.$i"
    
    echo "Pinging $ip..."

    # Ping the IP address and capture the ping response time
    ping_result=$(ping -c 1 -W 1 "$ip" 2>&1)

    if [[ $? -eq 0 ]]; then
        # Ping was successful
        response_time=$(echo "$ping_result" | awk -F'/' 'END {print $5}')
        
        if (( $(bc <<< "$response_time > 2.0") )); then
            long_ping_found=true
        fi

        if (( $(bc <<< "$response_time <= 2.0") )); then
            FRANKA_IP="$ip"
            break
        fi
    fi
done

# Export the FRANKA_IP as an environment variable
export FRANKA_IP

# Print the selected IP address
echo "FRANKA_IP is set to: $FRANKA_IP"

# Print a warning message if long ping times were encountered
if $long_ping_found; then
    echo "Warning: Some IP addresses had ping times longer than 2 ms."
fi
