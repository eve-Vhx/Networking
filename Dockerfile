# Use the official Python image from the Docker Hub
FROM ros:foxy-ros-base

# Set the working directory
WORKDIR /app

# Copy the current directory contents into the container at /app
COPY . /app

# Install the required packages
RUN apt-get update && \
    apt-get install -y \
    python3-pip \
    && pip3 install flask rclpy

# Make port 5000 available to the world outside this container
EXPOSE 5000

# Define environment variable
ENV NAME FlaskApp

# Run app.py when the container launches
CMD ["python", "app.py"]