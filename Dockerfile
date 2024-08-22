# Use the official Ubuntu 20.04 image from the Docker Hub
FROM ubuntu:20.04

# Set the working directory
WORKDIR /app

# Install dependencies
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*

# Setup ROS 2 sources
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
    && sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Install ROS 2 packages
RUN apt-get update && apt-get install -y \
    ros-foxy-ros-base \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN apt-get update && apt-get install -y \
    python3-pip \
    && pip3 install flask pyyaml \
    && rm -rf /var/lib/apt/lists/*

# Install rclpy
RUN apt-get update && apt-get install -y \
    python3-rclpy \
    && rm -rf /var/lib/apt/lists/*

# Copy the current directory contents into the container at /app
COPY . /app

# Make port 5000 available to the world outside this container
EXPOSE 5000

# Define environment variable
ENV NAME FlaskApp

# Source the ROS 2 setup script and run app.py when the container launches
CMD ["bash", "-c", "source /opt/ros/foxy/setup.bash && python3 app.py"]