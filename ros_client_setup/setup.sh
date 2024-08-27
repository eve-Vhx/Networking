# Make sure to run this script as root
if [ "$EUID" -ne 0 ]; then
    echo "Please run this script as root"
    exit 1
fi

# Prompt the user for the location of the conf file
echo "Please enter the location of the wireguard conf file"
read conf_file

# Check if the file exists. If not, ask again until the file exists
if [ ! -f "$conf_file" ]; then
    echo "File not found. Please enter the location of the wireguard conf file"
    read conf_file
fi

# Check if the file is empty. If it is, ask again until the file is not empty
if [ ! -s "$conf_file" ]; then
    echo "File is empty. Please enter the location of the wireguard conf file"
    read conf_file
fi

# Check if the file is a valid wireguard conf file. If it is not, ask again until the file is a valid wireguard conf file
if ! grep -q "PrivateKey" "$conf_file"; then
    echo "File is not a valid wireguard conf file. Please enter the location of the wireguard conf file"
    read conf_file
fi

echo "File is a valid wireguard conf file."
echo "Starting ROS client setup..."

echo "Installing wireguard..."

# Install wireguard
sudo apt-get update -y
sudo apt-get upgrade -y

sudo apt-get install wireguard -y

# Copy conf file to /etc/wireguard/wg0.conf
sudo cp "$conf_file" /etc/wireguard/wg0.conf

echo "Wireguard installed."
echo "Enabling wg system daemon..."

# Enable wireguard on boot
systemctl enable wg-quick@wg0

# Start wireguard
systemctl start wg-quick@wg0

# Check if wireguard is running
systemctl status wg-quick@wg0

echo "wg system daemon enabled."

echo "Setting up ROS client..."

# Add ROS_DISCOVERY_SERVER to .bashrc to be VPN server IP:11811
echo "export ROS_DISCOVERY_SERVER=10.8.0.1:11811" >> ~/.bashrc

# Copy fastdds_wan.xml from the same directory as this setup script to /home/$USER/.ros
mkdir -p /home/$USER/.ros
cp fastdds_wan.xml /home/$USER/.ros/fastdds_wan.xml

# Add RMW_IMPLEMENTATION to .bashrc
echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc

# Add FASTRTPS_DEFAULT_PROFILES_FILE to .bashrc
echo "export FASTRTPS_DEFAULT_PROFILES_FILE=/home/$USER/.ros/fastdds_wan.xml" >> ~/.bashrc

# Source .bashrc
source ~/.bashrc

echo "ROS client setup complete."


