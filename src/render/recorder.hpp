#pragma once
#include <cstdint>
#include <cstdio>
#include <string>

namespace skein {

class VideoRecorder {
public:
    ~VideoRecorder();

    bool open(const std::string& path, int width, int height, int fps, std::string& error);
    void capture(unsigned framebuffer = 0);
    void close();

    bool recording() const { return pipe_ != nullptr; }
    uint64_t frames() const { return frames_; }
    int width() const { return width_; }
    int height() const { return height_; }

private:
    static constexpr int RING = 3;
    void drain(int slot);

    FILE* pipe_ = nullptr;
    int width_ = 0;
    int height_ = 0;
    uint64_t frames_ = 0;
    uint64_t written_ = 0;
    unsigned pbos_[RING] = {0, 0, 0};
};

}  // namespace skein
