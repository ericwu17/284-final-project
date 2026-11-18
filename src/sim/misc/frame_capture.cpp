#include "frame_capture.h"

#include <ctime>
#include <filesystem>
#include <iostream>
#include <vector>

#include <glad/glad.h>

#include "CGL/lodepng.h"

namespace FrameCapture {

bool save_framebuffer_png(const std::string &path, int fb_width, int fb_height) {
  if (fb_width <= 0 || fb_height <= 0) return false;

  std::filesystem::path parent = std::filesystem::path(path).parent_path();
  if (!parent.empty()) {
    std::error_code ec;
    std::filesystem::create_directories(parent, ec);
    if (ec) {
      std::cerr << "[Capture] Could not create " << parent << ": " << ec.message() << std::endl;
      return false;
    }
  }

  const size_t row_bytes = (size_t)fb_width * 3;
  std::vector<unsigned char> pixels(row_bytes * fb_height);

  glPixelStorei(GL_PACK_ALIGNMENT, 1);
  glReadBuffer(GL_BACK);
  glReadPixels(0, 0, fb_width, fb_height, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());

  // OpenGL's row 0 is the bottom of the image; PNG's row 0 is the top.
  std::vector<unsigned char> flipped(pixels.size());
  for (int y = 0; y < fb_height; y++) {
    std::copy(pixels.begin() + (size_t)(fb_height - 1 - y) * row_bytes,
              pixels.begin() + (size_t)(fb_height - y) * row_bytes,
              flipped.begin() + (size_t)y * row_bytes);
  }

  unsigned err = lodepng::encode(path, flipped, fb_width, fb_height, LCT_RGB, 8);
  if (err) {
    std::cerr << "[Capture] PNG encode failed for " << path << ": "
              << lodepng_error_text(err) << std::endl;
    return false;
  }
  return true;
}

std::string timestamp() {
  std::time_t now = std::time(nullptr);
  char buf[32];
  std::strftime(buf, sizeof(buf), "%Y%m%d-%H%M%S", std::localtime(&now));
  return buf;
}

} // namespace FrameCapture
