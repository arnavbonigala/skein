#include "render/recorder.hpp"

#include "render/gl.hpp"

namespace skein {

VideoRecorder::~VideoRecorder() { close(); }

bool VideoRecorder::open(const std::string& path, int width, int height, int fps, std::string& error) {
    close();
    char command[1024];
    std::snprintf(command, sizeof(command),
                  "ffmpeg -loglevel error -y -f rawvideo -pix_fmt rgba -s %dx%d -r %d -i - -vf vflip "
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
    written_ = 0;
    const GLsizeiptr bytes = static_cast<GLsizeiptr>(width) * height * 4;
    glGenBuffers(RING, pbos_);
    for (unsigned pbo : pbos_) {
        glBindBuffer(GL_PIXEL_PACK_BUFFER, pbo);
        glBufferData(GL_PIXEL_PACK_BUFFER, bytes, nullptr, GL_STREAM_READ);
    }
    glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
    return true;
}

void VideoRecorder::drain(int slot) {
    glBindBuffer(GL_PIXEL_PACK_BUFFER, pbos_[slot]);
    const void* pixels = glMapBuffer(GL_PIXEL_PACK_BUFFER, GL_READ_ONLY);
    if (pixels) std::fwrite(pixels, 1, static_cast<size_t>(width_) * static_cast<size_t>(height_) * 4, pipe_);
    glUnmapBuffer(GL_PIXEL_PACK_BUFFER);
    glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
    ++written_;
}

void VideoRecorder::capture(unsigned framebuffer) {
    if (!pipe_) return;
    const int slot = static_cast<int>(frames_ % RING);
    if (frames_ >= RING) drain(slot);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, framebuffer);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glBindBuffer(GL_PIXEL_PACK_BUFFER, pbos_[slot]);
    glReadPixels(0, 0, width_, height_, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
    glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
    ++frames_;
}

void VideoRecorder::close() {
    if (!pipe_) return;
    while (written_ < frames_) drain(static_cast<int>(written_ % RING));
    glDeleteBuffers(RING, pbos_);
    for (unsigned& pbo : pbos_) pbo = 0;
    pclose(pipe_);
    pipe_ = nullptr;
}

}  // namespace skein
