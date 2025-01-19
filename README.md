# Lattice Boltzmann Simulation in Metal C++

This project implements a high-performance Lattice Boltzmann Method (LBM) simulation using Metal C++. Designed to leverage the full power of Apple Silicon GPUs, it achieves **real-time simulation** speeds even for large-scale problems.

Expects the model files to be input as an stl.

## Setup and Usage
1. Clone the repository:
   ```bash
   git clone [<repository-url>](https://github.com/Charlie-Close/Lattice-Boltzman.git)
   ```
1. Open the project in Xcode:
   ```bash
   cd "Lattice Boltzman"
   open "Lattice Boltzman.xcodeproj"
   ```
3. Build and run the project:
  - Click the Run button in Xcode, and the simulation will start.
  - No additional setup is needed—dependencies are included.
  - Simulation parameters can be tuned in Parameters/Paramers.h.

## Performance Benchmarks

From my testing, it can run at 35 FPS with 10 million lattice points on my Macbook Pro M2 Max.
   
