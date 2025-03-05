import cv2
import numpy as np
import math

# Parameters
marker_ids = range(1, 11)  # Marker IDs 1 to 10
marker_size = 200          # Each marker will be 200x200 pixels
gap = 10                   # Gap between markers in pixels
margin = 10                # Margin around the markers
cols = 3                   # Number of markers per row
num_markers = len(list(marker_ids))
rows = math.ceil(num_markers / cols)  # Calculate the number of rows needed

# Create the ArUco dictionary (using 4x4 markers with 50 markers available)
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

# Calculate the overall canvas dimensions
canvas_width = margin * 2 + cols * marker_size + (cols - 1) * gap
canvas_height = margin * 2 + rows * marker_size + (rows - 1) * gap

# Create a white canvas image
canvas = 255 * np.ones((canvas_height, canvas_width), dtype=np.uint8)

# Draw markers and place them on the canvas in a grid
for i, marker_id in enumerate(marker_ids):
    # Create an empty image for the marker
    marker_img = np.zeros((marker_size, marker_size), dtype=np.uint8)
    
    # Draw the marker using the specified ArUco dictionary
    cv2.aruco.drawMarker(aruco_dict, marker_id, marker_size, marker_img, 1)
    
    # Determine the row and column for this marker
    row = i // cols
    col = i % cols
    
    # Calculate the placement coordinates with margin and gap
    x_offset = margin + col * (marker_size + gap)
    y_offset = margin + row * (marker_size + gap)
    
    # Place the marker image onto the canvas
    canvas[y_offset:y_offset+marker_size, x_offset:x_offset+marker_size] = marker_img

# Save the final image
output_filename = "aruco_markers_3_per_row.png"
cv2.imwrite(output_filename, canvas)
print(f"Saved image as {output_filename}")
