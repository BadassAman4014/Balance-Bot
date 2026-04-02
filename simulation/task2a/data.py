import pandas as pd
import plotly.graph_objects as go
import re

# Load the data from the CSV file
with open('d.csv', 'r') as file:
    lines = file.readlines()

# Initialize empty lists to store the values
tur_yaw_values = []
bal_yaw_values = []
pitch_values = []
roll_values = []
yxref_values = []
txref_values = []

# Extract all values from each line (including YxRef and TxRef)
for line in lines:
    match = re.search(r'tur_yaw = ([^,]+), bal_yaw = ([^,]+), pitch = ([^,]+), roll = ([^,]+), YxRef = ([^,]+), TxRef = ([^,]+)', line)
    if match:
        tur_yaw_values.append(float(match.group(1)))
        bal_yaw_values.append(float(match.group(2)))
        pitch_values.append(float(match.group(3)))
        roll_values.append(float(match.group(4)))
        yxref_values.append(float(match.group(5)))
        txref_values.append(float(match.group(6)))

# Create a DataFrame with the extracted values
df = pd.DataFrame({
    'tur_yaw': tur_yaw_values,
    'bal_yaw': bal_yaw_values,
    'pitch': pitch_values,
    'roll': roll_values,
    'YxRef': yxref_values,
    'TxRef': txref_values
})

# Create an interactive plot using Plotly
fig = go.Figure()

# Add the data to the plot
fig.add_trace(go.Scatter(x=df.index, y=df['tur_yaw'], mode='lines', name='Tur Yaw', line=dict(color='blue')))
fig.add_trace(go.Scatter(x=df.index, y=df['bal_yaw'], mode='lines', name='Bal Yaw', line=dict(color='purple')))
fig.add_trace(go.Scatter(x=df.index, y=df['pitch'], mode='lines', name='Pitch', line=dict(color='green')))
fig.add_trace(go.Scatter(x=df.index, y=df['roll'], mode='lines', name='Roll', line=dict(color='red')))
fig.add_trace(go.Scatter(x=df.index, y=df['YxRef'], mode='lines', name='YxRef', line=dict(color='orange')))
fig.add_trace(go.Scatter(x=df.index, y=df['TxRef'], mode='lines', name='TxRef', line=dict(color='brown')))

# Update layout for better interactivity
fig.update_layout(
    title='All Variables (Tur Yaw, Bal Yaw, Pitch, Roll, YxRef, TxRef) over Time',
    xaxis_title='Index',
    yaxis_title='Values',
    template='plotly_dark',  # Optional: change theme (light, dark, etc.)
    hovermode='closest',  # Enable hover to display data
    showlegend=True
)

# Show the interactive plot
fig.show()
