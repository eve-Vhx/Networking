import time
import subprocess
from datetime import datetime
import socket
import argparse
import os
import requests
import traceback
import platform
import sys

def run_command(command):
    try:
        result = subprocess.run(command, capture_output=True, text=True, check=True)
        return result.stdout
    except subprocess.CalledProcessError as e:
        print(f"Command '{' '.join(command)}' failed with error: {e}")
        print(f"STDOUT: {e.stdout}")
        print(f"STDERR: {e.stderr}")
        return None

def measure_latency(host):
    if platform.system().lower() == "windows":
        command = ["ping", "-n", "1", host]
    else:
        command = ["ping", "-c", "1", host]
    
    output = run_command(command)
    if output:
        try:
            if platform.system().lower() == "windows":
                latency = float(output.split("time=")[-1].split("ms")[0].strip())
            else:
                latency = float(output.split("time=")[-1].split()[0])
            return latency
        except Exception as e:
            print(f"Error parsing latency: {e}")
    return None

def measure_packet_loss(host, count=10):
    if platform.system().lower() == "windows":
        command = ["ping", "-n", str(count), host]
    else:
        command = ["ping", "-c", str(count), host]
    
    output = run_command(command)
    if output:
        try:
            if platform.system().lower() == "windows":
                packet_loss_line = [line for line in output.split('\n') if "loss" in line][0]
                packet_loss = int(packet_loss_line.split("(")[1].split("%")[0])
            else:
                packet_loss = float(output.split("%")[0].split()[-1])
            return packet_loss
        except Exception as e:
            print(f"Error parsing packet loss: {e}")
            print(f"Packet loss output: {output}")
    return None

def measure_jitter(host, count=5):
    latencies = []
    for _ in range(count):
        latency = measure_latency(host)
        if latency is not None:
            latencies.append(latency)
        time.sleep(0.1)
    
    if len(latencies) < 2:
        return None
    
    jitter = sum(abs(latencies[i] - latencies[i-1]) for i in range(1, len(latencies))) / (len(latencies) - 1)
    return jitter

def measure_bandwidth(host, duration=1):
    url = f"http://{host}:8000/dummy_file"
    start_time = time.time()
    total_bytes = 0
    
    try:
        response = requests.get(url, stream=True, timeout=duration)
        for chunk in response.iter_content(chunk_size=1024):
            if time.time() - start_time > duration:
                break
            total_bytes += len(chunk)
        
        elapsed_time = time.time() - start_time
        bandwidth_mbps = (total_bytes * 8) / (elapsed_time * 1_000_000)
        return bandwidth_mbps
    except requests.RequestException as e:
        print(f"Bandwidth measurement error: {str(e)}")
        return None

def format_metric(value):
    return f"{value:.2f}" if value is not None else "N/A"

def log_metrics(log_file, source, destination, latency, packet_loss, jitter, bandwidth):
    timestamp = datetime.now().isoformat()
    log_entry = f"{timestamp},{source},{destination},{format_metric(latency)},{format_metric(packet_loss)},{format_metric(jitter)},{format_metric(bandwidth)}\n"
    
    with open(log_file, 'a') as f:
        f.write(log_entry)

def main(ec2_ip, log_file, interval):
    source = socket.gethostname()
    
    if not os.path.exists(log_file):
        with open(log_file, 'w') as f:
            f.write("Timestamp,Source,Destination,Latency(ms),PacketLoss(%),Jitter(ms),Bandwidth(Mbps)\n")
    
    print(f"Logging connection metrics to {log_file}")
    print(f"Press Ctrl+C to stop logging")
    
    bandwidth = None
    last_bandwidth_measurement = 0
    
    try:
        while True:
            start_time = time.time()
            
            print(f"Measuring latency to {ec2_ip}...")
            latency = measure_latency(ec2_ip)
            print(f"Measuring packet loss to {ec2_ip}...")
            packet_loss = measure_packet_loss(ec2_ip)
            print(f"Measuring jitter to {ec2_ip}...")
            jitter = measure_jitter(ec2_ip)
            
            if time.time() - last_bandwidth_measurement >= 60:
                print(f"Measuring bandwidth to {ec2_ip}...")
                bandwidth = measure_bandwidth(ec2_ip)
                last_bandwidth_measurement = time.time()
            
            log_metrics(log_file, source, ec2_ip, latency, packet_loss, jitter, bandwidth)
            print(f"Logged metrics - Latency: {format_metric(latency)}ms, "
                  f"Packet Loss: {format_metric(packet_loss)}%, "
                  f"Jitter: {format_metric(jitter)}ms, "
                  f"Bandwidth: {format_metric(bandwidth)} Mbps")
            
            elapsed_time = time.time() - start_time
            sleep_time = max(0, interval - elapsed_time)
            time.sleep(sleep_time)
    except KeyboardInterrupt:
        print("\nLogging stopped")
    except Exception as e:
        print(f"An unexpected error occurred: {str(e)}")
        print(traceback.format_exc())

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Log connection metrics to EC2 instance")
    parser.add_argument("ec2_ip", help="IP address of the EC2 instance")
    parser.add_argument("--log-file", default="connection_metrics.csv", help="Path to the log file (default: connection_metrics.csv)")
    parser.add_argument("--interval", type=float, default=3, help="Interval between measurements in seconds (default: 3)")
    
    args = parser.parse_args()
    
    main(args.ec2_ip, args.log_file, args.interval)