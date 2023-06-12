import numpy as np
import cv2
import yaml

def create_pgm_map(polygon_points, resolution, output_file):
    # Calculate the dimensions of the map
    min_x = min(p[0] for p in polygon_points)
    max_x = max(p[0] for p in polygon_points)
    min_y = min(p[1] for p in polygon_points)
    max_y = max(p[1] for p in polygon_points)

    # Calculate the dimensions in pixels
    width = int((max_x - min_x) / resolution)
    height = int((max_y - min_y) / resolution)

    # Create a blank image with the expanded dimensions
    margin = 1
    expanded_width = int(width + (2 * margin / resolution))
    expanded_height = int(height + (2 * margin / resolution))
    image = np.ones((expanded_height, expanded_width), dtype=np.uint8)

    # Convert polygon points to pixels
    polygon_pixels = []
    for point in polygon_points:
        x = int((point[0] - min_x + margin) / resolution)
        y = int((point[1] - min_y + margin) / resolution)
        polygon_pixels.append((x, y))

    # Fill the polygon
    cv2.fillPoly(image, [np.array(polygon_pixels)], color=255)

    # Write the image to a PGM file
    cv2.imwrite(output_file, image)

    # Generate the YAML metadata
    yaml_data = {
        'image': output_file,
        'resolution': resolution,
        'origin': [min_x - margin, min_y - margin, 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.196
    }

    # Write the YAML metadata to a file
    yaml_file = output_file.replace('.pgm', '.yaml')
    with open(yaml_file, 'w') as f:
        yaml.dump(yaml_data, f)

    print(f"Map files generated: {output_file}, {yaml_file}")


# Define the inputs
# My Room
polygon_points = [(5, 0), (5, 3.2), (0.83, 3.2), (0.83, 1.1), (0, 1.1), (0, 0)]
# My Dinning Room 
# Rectangle 4.6m x 3.1m
polygon_points = [(0, 0), (0, 3.1), (4.6, 3.1), (4.6, 0)]
# Dr. Herman's Lab
# Triangle 8.05m x 10m x 12.83m
polygon_points = [(10, 0), (0, -8.05), (0, 0)]
resolution = 0.05
output_file = 'DrHermanLab.pgm'

# Generate the map files
create_pgm_map(polygon_points, resolution, output_file)
