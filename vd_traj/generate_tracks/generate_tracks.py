import numpy as np
import matplotlib.pyplot as plt

# Function to generate a straight line track
def generate_line_track(num_points=100, length=100):
    x = np.linspace(0, length, num_points)
    y = np.zeros(num_points)  # y is constant for a straight line
    return x, y

# Function to generate a circular track
def generate_circle_track(num_points=100, radius=50, center=(0, 0)):
    theta = np.linspace(0, 2 * np.pi, num_points)
    x = center[0] + radius * np.cos(theta)
    y = center[1] + radius * np.sin(theta)
    return x, y

# Function to generate a sine wave track
def generate_sine_wave_track(num_points=100, length=100, amplitude=10, frequency=0.1):
    x = np.linspace(0, length, num_points)
    y = amplitude * np.sin(frequency * x)
    return x, y

# Function to generate a figure-eight shape track
def generate_figure_eight_track(num_points=100, length=100, amplitude=10):
    t = np.linspace(0, length, num_points)
    x = amplitude * np.sin(t) + amplitude * np.sin(2 * t)
    y = amplitude * np.cos(t) - amplitude * np.cos(2 * t)
    return x, y

# Function to generate a general curved track (using a simple quadratic curve for example)
def generate_curve_track(num_points=100, length=100, curve_factor=0.1):
    x = np.linspace(0, length, num_points)
    y = curve_factor * x ** 2  # Parabolic curve
    return x, y

# Function to compute yaw angles from waypoints
def compute_yaw_angles(x, y):
    yaw = np.zeros(len(x))
    for i in range(1, len(x)):
        dx = x[i] - x[i-1]
        dy = y[i] - y[i-1]
        yaw[i] = np.arctan2(dy, dx)  # Yaw angle in radians
    return yaw

# Function to generate vehicle velocity (longitudinal velocity)
def generate_velocity(num_points=100, max_velocity=30):
    # For simplicity, use a constant velocity or vary it
    Vx = np.ones(num_points) * max_velocity  # Constant velocity for simplicity
    return Vx

# Updated function to save track data with cumulative length
def save_track_data(x, y, yaw, Vx, track_type):
    # Calculate cumulative track length (s)
    s = np.zeros(len(x))
    for i in range(1, len(x)):
        dx = x[i] - x[i-1]
        dy = y[i] - y[i-1]
        s[i] = s[i-1] + np.sqrt(dx**2 + dy**2)

    # Save the data to a file
    filename = f"{track_type}.txt"
    with open(filename, 'w') as f:
        for i in range(len(x)):
            f.write(f"{s[i]:.2f} {x[i]:.2f} {y[i]:.2f} {yaw[i]:.2f} {Vx[i]:.2f}\n")
    print(f"Track data saved to {filename}")

# Main function to generate and visualize track
def main(track_type='sine_wave', num_points=100, length=100):
    # Generate track based on selected type
    if track_type == 'line':
        x, y = generate_line_track(num_points, length)
    elif track_type == 'circle':
        x, y = generate_circle_track(num_points, radius=50)
    elif track_type == 'sine_wave':
        x, y = generate_sine_wave_track(num_points, length)
    elif track_type == 'figure_eight':
        x, y = generate_figure_eight_track(num_points, length)
    elif track_type == 'curve':
        x, y = generate_curve_track(num_points, length)
    else:
        raise ValueError("Unknown track type. Available types: 'line', 'circle', 'sine_wave', 'figure_eight', 'curve'.")

    # Compute yaw angles
    yaw = compute_yaw_angles(x, y)

    # Generate velocity (constant for simplicity)
    Vx = generate_velocity(num_points)

    # Save track data to a file
    save_track_data(x, y, yaw, Vx, track_type)

    # Plot the track
    plt.figure(figsize=(10, 6))
    plt.plot(x, y, label=f'{track_type} Track')
    plt.title(f'Generated {track_type.capitalize()} Track')
    plt.xlabel('X Position (meters)')
    plt.ylabel('Y Position (meters)')
    plt.grid(True)
    plt.legend()
    plt.show()

if __name__ == '__main__':
    # Example of generating a sine wave track
    main(track_type='curve', num_points=300, length=50)
