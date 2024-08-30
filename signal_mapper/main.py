import pandas as pd
import plotly.graph_objects as go
from plotly.subplots import make_subplots

# Ask user for the file name to read from logs folder
file_name = input('Enter the file name to read from logs folder: ')

# Read the CSV file from logs folder
df = pd.read_csv(f'./logs/{file_name}')

# Convert Timestamp to datetime and extract time
df['Timestamp'] = pd.to_datetime(df['Timestamp'])
df['Time'] = df['Timestamp'].dt.strftime('%H:%M:%S')

# Create subplots
fig = make_subplots(rows=4, cols=1, shared_xaxes=True, subplot_titles=(
    'LTE Reference Signal Received Power (RSRP)',
    'LTE Reference Signal Received Quality (RSRQ)',
    'Received Signal Strength Indicator (RSSI)',
    'Serving Cell RSSI'
))

# Add RSRP trace
fig.add_trace(go.Scatter(x=df.index, y=df['RSRP'], mode='lines', name='RSRP', line=dict(color='red')), row=1, col=1)

# Add RSRQ trace
fig.add_trace(go.Scatter(x=df.index, y=df['RSRQ'], mode='lines', name='RSRQ', line=dict(color='blue')), row=2, col=1)

# Add RSSI trace
fig.add_trace(go.Scatter(x=df.index, y=df['RSSI'], mode='lines', name='RSSI', line=dict(color='green')), row=3, col=1)

# Add servingcell_RSSI trace
fig.add_trace(go.Scatter(x=df.index, y=df['servingcell_RSSI'], mode='lines', name='servingcell_RSSI', line=dict(color='purple')), row=4, col=1)

# Update y-axis titles
fig.update_yaxes(title_text='RSRP (dBm)', range=[-140, -44], row=1, col=1)
fig.update_yaxes(title_text='RSRQ (dB)', range=[-20, -3], row=2, col=1)
fig.update_yaxes(title_text='RSSI', range=[-113, 0], row=3, col=1)
fig.update_yaxes(title_text='servingcell_RSSI', row=4, col=1)

# Update x-axis
fig.update_xaxes(title_text='Time', tickvals=df.index[::len(df)//10], ticktext=df['Time'][::len(df)//10], row=4, col=1)

# Update layout
fig.update_layout(height=800, width=1200, title_text='Signal Metrics Over Time', showlegend=True)

# Show the plot
fig.show()