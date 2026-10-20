#include "render/recorder.hpp"

#include "render/gl.hpp"

namespace skein {

VideoRecorder::~VideoRecorder() { close(); }

bool VideoRecorder::open(const std::string& path, int width, int height, int fps, std::string& error) {
    close();
    char command[1024];
    std::snprintf(command, sizeof(command),
                  "ffmpeg -loglevel error -y -f rawvideo -pix_fmt rgb24 -s %dx%d -r %d -i - -vf vflip "
                  "-c:v h264_videotoolbox -b:v 60M -pix_fmt yuv420p \"%s\"",
                  width, height, fps, path.c_str());
    pipe_ = popen(command, "w");
    if (!pipe_) {
        error = "could not start ffmpeg";
        return false;
    }
    width_ = width;
    height_ = height;
    frames_ = 0;
    pixels_.resize(static_cast<size_t>(width) * static_cast<size_t>(height) * 3);
    return true;
}

void VideoRecorder::capture() {
    if (!pipe_) return;
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadPixels(0, 0, width_, height_, GL_RGB, GL_UNSIGNED_BYTE, pixels_.data());
    std::fwrite(pixels_.data(), 1, pixels_.size(), pipe_);
    ++frames_;
}

void VideoRecorder::close() {
    if (!pipe_) return;
    pclose(pipe_);
    pipe_ = nullptr;
}

}  // namespace skein
