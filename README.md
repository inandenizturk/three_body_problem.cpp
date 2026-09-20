

https://github.com/user-attachments/assets/4bf50d5e-17bd-4559-adaf-f9951e01e828

<img width="791" height="801" alt="Screenshot 2026-09-13 at 15 55 31" src="https://github.com/user-attachments/assets/616e8e68-14f8-4543-909b-238645d90e95" />
<img width="791" height="801" alt="3body" src="https://github.com/user-attachments/assets/7b4a08c4-5b8e-4081-8689-86d7bbed33c3" />
# 3-Body Problem Simulation (C++)

A zero-dependency, high-performance gravitational simulation written in C++17. Solves the planar three-body problem numerically using a 4th-order Runge-Kutta (RK4) integrator and pipes raw frames directly to FFmpeg to generate an MP4 video.

---

## Quick Start (macOS)

Run the one-liner below in your terminal to compile with maximum optimizations, run the simulation, and automatically open the resulting video:

```bash
clang++ -O3 -std=c++17 render_mp4.cpp -o render_mp4 && ./render_mp4 && open three_body_cxx.mp4


Key Features
RK4 Numerical Integrator: 4th-order Runge-Kutta integration with a gravitational softening parameter (ϵ) to eliminate division-by-zero singularities.
Algorithmic Efficiency: Gravitational potential energy is calculated in O(N(N−1)/2) by traversing the upper triangular matrix (j=i+1).
Hamiltonian Verification: Tracks total mechanical energy (E=T+U) frame-by-frame to monitor numerical stability.
Software Rasterizer: Renders directly to a flat row-major pixel buffer with an exponential decay motion blur trail.
FFmpeg Pipe: Streams raw RGB bytes directly to the FFmpeg process using popen, avoiding intermediate disk writes.


Prerequisites
C++17 compliant compiler (clang++ or g++)
FFmpeg installed and available in your system PATH


License
This project is open source and available under the MIT License.
