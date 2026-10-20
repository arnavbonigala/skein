#pragma once
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

namespace skein {

class VideoRecorder {
public:
    ~VideoRecorder();

    bool open(const std::string& path, int width, int height, int fps, std::string& error);
    void capture();
    void close();

    bool recording() const { return pipe_ != nullptr; }
    uint64_t frames() const { return frames_; }

private:
    FILE* pipe_ = nullptr;
    int width_ = 0;
    int height_ = 0;
    uint64_t frames_ = 0;
    std::vector<uint8_t> pixels_;
};

}  // namespace skein
