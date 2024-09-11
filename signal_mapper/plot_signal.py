# Given a csv file of the RSRP, RSRQ, RSSI, and servingcell_RSSI values, and another csv file containing the timestamped GPS data, the goal is to create a plot of the signal metrics overlayed on a map.
# The drone's flight path is represented by a line plot on the map, and the signal strength is represented by a heatmap.

import glob
import pandas as pd
import plotly.express as px
import os
import re, math
import plotly.graph_objects as go
from io import StringIO


def extract_gps_data(gps_data_file):
    # Read the CSV file into a DataFrame
    df = pd.read_csv(gps_data_file)

    # Define a regex pattern to extract lat, lon, and alt from the data column
    pattern = re.compile(r'lat=([-\d.]+), lon=([-\d.]+), alt=([-\d.]+)')

    # Initialize lists to store the extracted values
    timestamps = []
    lats = []
    lons = []
    alts = []

    # Iterate over the rows and extract the GPS data
    for index, row in df.iterrows():
        match = pattern.search(row['data'])
        if match:
            timestamps.append(row['timestamp'])
            lats.append(float(match.group(1)))
            lons.append(float(match.group(2)))
            alts.append(float(match.group(3)))

    # Create a new DataFrame with the extracted GPS data
    gps_df = pd.DataFrame({
        'timestamp': timestamps,
        'lat': lats,
        'lon': lons,
        'alt': alts
    })

    # Convert the GPS timestamps to Unix timestamps in seconds
    gps_df['timestamp'] = pd.to_datetime(gps_df['timestamp']).astype('int64') // 10**9

    # Round the GPS timestamps to the nearest whole second and drop duplicates
    gps_df['timestamp'] = gps_df['timestamp'].round().astype('int64')
    gps_df = gps_df.drop_duplicates(subset=['timestamp'])

    return gps_df


def user_select_folder():
    # Go into the logs folder and print the folders inside with a number for the user to select
    folders = os.listdir('./logs')
    print('Select the folder to read from:')
    for i, folder in enumerate(folders):
        print(f'{i+1}. {folder}')
    folder_number = int(input('Enter the number for the folder: '))
    folder_name = folders[folder_number-1]

    return folder_name


def get_csv_files(folder_name):
    # Get the signal metrics log file
    signal_metrics_pattern = os.path.join('./logs', folder_name, 'signal_logs', 'signal_metrics_log_*.csv')
    signal_metrics_files = glob.glob(signal_metrics_pattern)
    if not signal_metrics_files:
        raise FileNotFoundError(f"No signal metrics log file found in {signal_metrics_pattern}")
    signal_metrics_file = signal_metrics_files[0]

    # Get the GPS data file
    gps_data_file = os.path.join('./logs', folder_name, 'output.csv')
    if not os.path.exists(gps_data_file):
        raise FileNotFoundError(f"No GPS data file found at {gps_data_file}")

    return signal_metrics_file, gps_data_file


def calculate_zoom_level(min_lat, max_lat, min_lon, max_lon):
    # Calculate the span of the latitude and longitude
    lat_span = max_lat - min_lat
    lon_span = max_lon - min_lon

    # Use a heuristic to calculate the zoom level
    max_span = max(lat_span, lon_span)
    zoom_level = 11 - math.log(max_span + 1e-6) / math.log(2)

    # Adjust the zoom level to avoid being too close
    zoom_level = max(zoom_level - 2, 0)  # Subtract 1 to zoom out a bit, ensure zoom level is not negative

    return zoom_level


def plot_drone_flight_path(gps_df):
    # Calculate the bounding box of the GPS coordinates
    min_lat = gps_df['lat'].min()
    max_lat = gps_df['lat'].max()
    min_lon = gps_df['lon'].min()
    max_lon = gps_df['lon'].max()

    # Calculate the center of the bounding box
    center_lat = (min_lat + max_lat) / 2
    center_lon = (min_lon + max_lon) / 2

    # Calculate the zoom level
    zoom_level = calculate_zoom_level(min_lat, max_lat, min_lon, max_lon)

    # Create a scatter plot of the drone's flight path
    fig = px.scatter_mapbox(
        gps_df,
        lat='lat',
        lon='lon',
        hover_data=['timestamp', 'alt'],
        zoom=zoom_level
    )

    # Update the layout with the calculated center and zoom level
    fig.update_layout(
        mapbox_style='open-street-map',
        mapbox_center={"lat": center_lat, "lon": center_lon},
        title='Drone Flight Path',
        showlegend=False
    )

    return fig


def plot_drone_flight_path_with_heatmap(gps_df, signal_df):
    # Ensure the timestamp columns are of the same type
    gps_df['timestamp'] = gps_df['timestamp'].astype('int64')
    signal_df['Timestamp'] = pd.to_datetime(signal_df['Timestamp']).astype('int64') // 10**9  # Convert to seconds

    # Merge GPS data with signal metrics data based on the closest timestamp
    merged_df = pd.merge_asof(gps_df.sort_values('timestamp'), signal_df.sort_values('Timestamp'), left_on='timestamp', right_on='Timestamp')

    # Filter the merged DataFrame to include only rows where RSRQ is -16 or below
    filtered_df = merged_df[merged_df['RSRQ'] <= -16]

    # Calculate the bounding box of the GPS coordinates
    min_lat = gps_df['lat'].min()
    max_lat = gps_df['lat'].max()
    min_lon = gps_df['lon'].min()
    max_lon = gps_df['lon'].max()

    # Calculate the center of the bounding box
    center_lat = (min_lat + max_lat) / 2
    center_lon = (min_lon + max_lon) / 2

    # Calculate the zoom level
    zoom_level = calculate_zoom_level(min_lat, max_lat, min_lon, max_lon)

    # Create a scatter plot of the drone's flight path
    fig = px.scatter_mapbox(
        gps_df,
        lat='lat',
        lon='lon',
        hover_data=['timestamp', 'alt'],
        zoom=zoom_level
    )

    # Add a heatmap layer for the RSRQ LTE signal quality
    fig.add_trace(go.Densitymapbox(
        lat=filtered_df['lat'],
        lon=filtered_df['lon'],
        z=filtered_df['RSRQ'],
        radius=10,
        colorscale='Viridis',
        showscale=True,
        colorbar=dict(title='RSRQ (dB)'),
        hoverinfo='lat+lon+z+text',
        text=filtered_df['timestamp']
    ))

    # Update the layout with the calculated center and zoom level
    fig.update_layout(
        mapbox_style='open-street-map',
        mapbox_center={"lat": center_lat, "lon": center_lon},
        title='Drone Flight Path with RSRQ LTE Signal Quality Heatmap',
        showlegend=False
    )

    return fig


def main():
    # Ask user for the folder name to read from logs folder
    folder_name = user_select_folder()
    signal_metrics_file, gps_data_file = get_csv_files(folder_name)
    print(f"Signal Metrics File: {signal_metrics_file}")
    print(f"GPS Data File: {gps_data_file}")

    # Read the signal metrics CSV file
    df = pd.read_csv(signal_metrics_file)

    # Use extract_gps_data to extract the GPS data from the GPS data file and put into a dataframe
    gps_df = extract_gps_data(gps_data_file)
        
    # Create a plot of the drone's flight path
    # fig = plot_drone_flight_path(gps_df)

    # Create a plot of the drone's flight path with heatmap
    fig = plot_drone_flight_path_with_heatmap(gps_df, df)

    # Show the plot
    fig.show()


if __name__ == '__main__':
    main()
