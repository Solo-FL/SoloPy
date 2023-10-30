#!/bin/bash

# Default bitrate
BITRATE=1000000


# Check if the service already exists
if systemctl --quiet is-active can_setup.service; then
  echo "The CAN setup service is already active."
  read -p "Do you want to stop, update, and restart the service? (y/n): " -n 1 -r
  echo
  if [[ $REPLY =~ ^[Yy]$ ]]; then
    sudo systemctl stop can_setup.service
  else
    echo "Exiting without making changes."
    exit 0
  fi
fi

# Create the script for setting up CAN interfaces
cat <<EOL | sudo tee /etc/init.d/can_setup.sh
#!/bin/bash
sudo ip link set can0 up type can bitrate $BITRATE
sudo ifconfig can0 txqueuelen 65536
EOL

# Make the script executable
sudo chmod +x /etc/init.d/can_setup.sh

# Create the systemd service file
cat <<EOL | sudo tee /etc/systemd/system/can_setup.service
[Unit]
Description=Setup CAN interfaces
After=network.target

[Service]
ExecStart=/etc/init.d/can_setup.sh

[Install]
WantedBy=multi-user.target
EOL

# Reload the systemd manager configuration
sudo systemctl daemon-reload

# Enable the service
sudo systemctl enable can_setup.service

# Start the service
sudo systemctl start can_setup.service

echo "CAN interfaces setup complete with a bitrate of $BITRATE."
