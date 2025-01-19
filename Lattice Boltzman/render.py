import os
import struct
import numpy as np
import matplotlib.pyplot as plt

def read_binary_file(filename):
    with open(filename, "rb") as file:
        # Read lattice size
        lattice_size = struct.unpack("3i", file.read(12))  # 3 integers for lattice dimensions
        struct.unpack("i", file.read(4))  # Skip 4 bytes due to simd_int3 alignment
        n_points = struct.unpack("i", file.read(4))[0]    # Number of lattice points

        print(n_points, lattice_size[0] * lattice_size[1] * lattice_size[2])
        # Initialize arrays
        positions = []
        densities = []
        velocities = []
        walls = []

        # Read each lattice point's data
        for _ in range(n_points):
            # Position (simd_int3)
            pos = struct.unpack("3i", file.read(12))  # 3 integers
            struct.unpack("i", file.read(4))  # Skip 4 bytes due to simd alignment
            positions.append(pos)

            # Density (float)
            density = struct.unpack("f", file.read(4))[0]
            densities.append(density)

            # Velocity (simd_float3)
            vel = struct.unpack("3f", file.read(12))  # 3 floats
            struct.unpack("i", file.read(4))  # Skip 4 bytes due to simd_int3 alignment
            velocities.append(vel)

            # Wall flag (bool)
            wall = struct.unpack("?", file.read(1))[0]
            walls.append(wall)

        return lattice_size, positions, densities, velocities, walls

def render_data(lattice_size, positions, densities, velocities, walls, filename):
    # Convert data into a 2D grid
    grid_density = np.zeros((lattice_size[1], lattice_size[0]))
    grid_velocity_x = np.zeros((lattice_size[1], lattice_size[0]))
    grid_velocity_y = np.zeros((lattice_size[1], lattice_size[0]))
    wall_mask = np.zeros((lattice_size[1], lattice_size[0]), dtype=bool)

    for pos, density, velocity, wall in zip(positions, densities, velocities, walls):
        x, y, z = pos
        if 0 <= x < lattice_size[0] and 0 <= y < lattice_size[1]:
            grid_density[y, x] = density
            grid_velocity_x[y, x] = velocity[0]
            grid_velocity_y[y, x] = velocity[1]
            wall_mask[x, y] = wall
    
    # Plot density as a heatmap
    plt.figure(figsize=(10, 8))
    plt.imshow(grid_density, origin="lower", cmap="viridis", interpolation="nearest")
    plt.colorbar(label="Density")

    # Overlay velocity vectors
    Y, X = np.meshgrid(range(lattice_size[1]), range(lattice_size[0]), indexing="ij")
    plt.quiver(X, Y, grid_velocity_x, grid_velocity_y, scale=20, color="white", alpha=0.8)

    # Overlay wall points
    plt.scatter(*np.where(wall_mask), color="red", s=5, label="Wall")

    plt.title(f"Fluid Flow Visualization - {filename}")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.legend()
    plt.tight_layout()

    # Save the figure
    output_image = f"{filename}.png"
    plt.savefig(output_image)
    plt.close()
    print(f"Rendered frame saved as {output_image}")

def process_and_render_frames(input_dir):
    # List all files in the input directory
    files = sorted([f for f in os.listdir(input_dir) if f.isdigit()], key=lambda x: int(x))

    for file in files:
        filepath = os.path.join(input_dir, file)
        lattice_size, positions, densities, velocities, walls = read_binary_file(filepath)
        render_data(lattice_size, positions, densities, velocities, walls, file)

# Example usage
process_and_render_frames("./images/")
