cat << 'EOF' > render_mp4.cpp
#include <iostream>
#include <vector>
#include <cmath>
#include <deque>
#include <cstdint>
#include <cstdio>

struct Vec2 {
    double x{0.0}, y{0.0};
    Vec2 operator+(const Vec2& o) const { return {x + o.x, y + o.y}; }
    Vec2 operator-(const Vec2& o) const { return {x - o.x, y - o.y}; }
    Vec2 operator*(double s) const { return {x * s, y * s}; }
    double norm_sq() const { return x * x + y * y; }
    double norm() const { return std::sqrt(norm_sq()); }
};

struct PixelRGB {
    uint8_t r, g, b;
};

struct Body {
    double mass;
    Vec2 pos;
    Vec2 vel;
    PixelRGB color;
    std::deque<std::pair<int, int>> trail;
};

const double G = 1.0;
const double softening = 1e-3;
const int W = 800;
const int H = 800;
const double scale = 260.0;

std::vector<Vec2> compute_accelerations(const std::vector<Vec2>& pos, const std::vector<Body>& bodies) {
    size_t n = pos.size();
    std::vector<Vec2> acc(n, {0.0, 0.0});
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = i + 1; j < n; ++j) {
            Vec2 r_vec = pos[j] - pos[i];
            double dist_sq = r_vec.norm_sq() + softening * softening;
            double dist = std::sqrt(dist_sq);
            double inv_r3 = 1.0 / (dist_sq * dist);
            Vec2 force_dir = r_vec * (G * inv_r3);
            acc[i] = acc[i] + force_dir * bodies[j].mass;
            acc[j] = acc[j] - force_dir * bodies[i].mass;
        }
    }
    return acc;
}

void step_rk4(std::vector<Body>& bodies, double dt) {
    size_t n = bodies.size();
    std::vector<Vec2> r0(n), v0(n);
    for (size_t i = 0; i < n; ++i) {
        r0[i] = bodies[i].pos;
        v0[i] = bodies[i].vel;
    }

    auto a0 = compute_accelerations(r0, bodies);
    std::vector<Vec2> dr1(n), dv1(n);
    for (size_t i = 0; i < n; ++i) { dr1[i] = v0[i] * dt; dv1[i] = a0[i] * dt; }

    std::vector<Vec2> r1(n);
    for (size_t i = 0; i < n; ++i) r1[i] = r0[i] + dr1[i] * 0.5;
    auto a1 = compute_accelerations(r1, bodies);
    std::vector<Vec2> dr2(n), dv2(n);
    for (size_t i = 0; i < n; ++i) { dr2[i] = (v0[i] + dv1[i] * 0.5) * dt; dv2[i] = a1[i] * dt; }

    std::vector<Vec2> r2(n);
    for (size_t i = 0; i < n; ++i) r2[i] = r0[i] + dr2[i] * 0.5;
    auto a2 = compute_accelerations(r2, bodies);
    std::vector<Vec2> dr3(n), dv3(n);
    for (size_t i = 0; i < n; ++i) { dr3[i] = (v0[i] + dv2[i] * 0.5) * dt; dv3[i] = a2[i] * dt; }

    std::vector<Vec2> r3(n);
    for (size_t i = 0; i < n; ++i) r3[i] = r0[i] + dr3[i];
    auto a3 = compute_accelerations(r3, bodies);
    std::vector<Vec2> dr4(n), dv4(n);
    for (size_t i = 0; i < n; ++i) { dr4[i] = (v0[i] + dv3[i]) * dt; dv4[i] = a3[i] * dt; }

    for (size_t i = 0; i < n; ++i) {
        bodies[i].pos = bodies[i].pos + (dr1[i] + dr2[i] * 2.0 + dr3[i] * 2.0 + dr4[i]) * (1.0 / 6.0);
        bodies[i].vel = bodies[i].vel + (dv1[i] + dv2[i] * 2.0 + dv3[i] * 2.0 + dv4[i]) * (1.0 / 6.0);
    }
}

void draw_circle(std::vector<PixelRGB>& buf, int cx, int cy, int radius, PixelRGB color) {
    for (int dy = -radius; dy <= radius; ++dy) {
        for (int dx = -radius; dx <= radius; ++dx) {
            if (dx * dx + dy * dy <= radius * radius) {
                int px = cx + dx;
                int py = cy + dy;
                if (px >= 0 && px < W && py >= 0 && py < H) {
                    buf[py * W + px] = color;
                }
            }
        }
    }
}

int main() {
    std::vector<Body> bodies = {
        {1.0, {-0.97000436,  0.24308753}, { 0.46620531,  0.43236573}, {255, 75, 75}, {}},
        {1.0, { 0.0,         0.0        }, {-0.93241062, -0.86473146}, {0, 230, 118}, {}},
        {1.0, { 0.97000436, -0.24308753}, { 0.46620531,  0.43236573}, {0, 176, 255}, {}}
    };

    const double dt = 0.001;
    const int sub_steps = 10;
    const int total_frames = 600;
    const size_t max_trail = 90;

    const char* ffmpeg_cmd = "ffmpeg -y -f rawvideo -pixel_format rgb24 -video_size 800x800 -framerate 60 "
                            "-i - -c:v libx264 -pix_fmt yuv420p -b:v 4000k three_body_cxx.mp4 2>/dev/null";

    FILE* ffmpeg_pipe = popen(ffmpeg_cmd, "w");
    if (!ffmpeg_pipe) {
        std::cerr << "FFmpeg boru hattı başlatılamadı!\n";
        return 1;
    }

    std::vector<PixelRGB> framebuffer(W * H);
    std::cout << "Starting C++ RK4 Render -> generating three_body_cxx.mp4...\n";

    for (int frame = 0; frame < total_frames; ++frame) {
        for (int s = 0; s < sub_steps; ++s) {
            step_rk4(bodies, dt);
        }

        for (auto& p : framebuffer) p = {13, 17, 23};

        for (auto& b : bodies) {
            int cx = static_cast<int>(W / 2.0 + b.pos.x * scale);
            int cy = static_cast<int>(H / 2.0 - b.pos.y * scale);

            b.trail.push_back({cx, cy});
            if (b.trail.size() > max_trail) b.trail.pop_front();

            for (size_t t = 0; t < b.trail.size(); ++t) {
                int tx = b.trail[t].first;
                int ty = b.trail[t].second;
                double alpha = static_cast<double>(t) / b.trail.size();
                PixelRGB tc = {
                    static_cast<uint8_t>(b.color.r * alpha * 0.6),
                    static_cast<uint8_t>(b.color.g * alpha * 0.6),
                    static_cast<uint8_t>(b.color.b * alpha * 0.6)
                };
                if (tx >= 0 && tx < W && ty >= 0 && ty < H) {
                    framebuffer[ty * W + tx] = tc;
                }
            }

            draw_circle(framebuffer, cx, cy, 6, b.color);
        }

        fwrite(framebuffer.data(), sizeof(PixelRGB), W * H, ffmpeg_pipe);

        if (frame % 50 == 0 || frame == total_frames - 1) {
            std::cout << "\rIlerleme: %" << (frame + 1) * 100 / total_frames << " tamamlandi..." << std::flush;
        }
    }

    pclose(ffmpeg_pipe);
    std::cout << "\n[Bitti] 'three_body_cxx.mp4' olusturuldu!\n";
    return 0;
}
EOF
clang++ -O3 -std=c++17 render_mp4.cpp -o render_mp4 && ./render_mp4

#code copy paste -> terminal
#for macOS starting animation: clang++ -O3 -std=c++17 render_mp4.cpp -o render_mp4 && ./render_mp4 && open three_body_cxx.mp4
