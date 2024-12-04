import numpy as np
import csv

R_base = 0.8  
T = 10  
omega = 2 * np.pi / T  
phases = [0, 2 * np.pi / 3, 4 * np.pi / 3]  
total_time = 2 * T  
num_points = 100

def compute_positions(t):
    top_point = [0, 0, 1.5]

    R2 = R_base * 0.5  
    layer2 = [
        [R2 * np.cos(omega * t + phase), R2 * np.sin(omega * t + phase), 1.0]
        for phase in phases
    ]
    R3 = R_base  
    layer3 = [
        [R3 * np.cos(omega * t + phase), R3 * np.sin(omega * t + phase), 0.5]
        for phase in phases
    ]
    return [top_point] + layer2 + layer3


def main():
    t_array = np.linspace(0, total_time, num_points)

    paths = {i: {'t': [], 'x': [], 'y': [], 'z': [], 'yaw': []} for i in range(7)}
    
    for t in t_array:
        positions = compute_positions(t)
        for i, pos in enumerate(positions):
            paths[i]['t'].append(t)
            paths[i]['x'].append(pos[0])
            paths[i]['y'].append(pos[1])
            paths[i]['z'].append(pos[2])
            paths[i]['yaw'].append(0)  # Constant yaw
    
    file_names = [
        'top_point_path.csv',
        'layer2_point1_path.csv',
        'layer2_point2_path.csv',
        'layer2_point3_path.csv',
        'layer3_point1_path.csv',
        'layer3_point2_path.csv',
        'layer3_point3_path.csv',
    ]
    
    # Save paths to CSV files
    for i, file_name in enumerate(file_names):
        with open(file_name, mode='w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['t', 'x', 'y', 'z', 'yaw'])  
            for t_val, x_val, y_val, z_val, yaw_val in zip(
                paths[i]['t'], paths[i]['x'], paths[i]['y'], paths[i]['z'], paths[i]['yaw']
            ):
                writer.writerow([t_val, x_val, y_val, z_val, yaw_val])
    
    print("Paths for all pyramid points written to CSV files.")


if __name__ == '__main__':
    main()


