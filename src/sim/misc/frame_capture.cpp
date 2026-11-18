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

  // Always write plain 8-bit RGB. By default lodepng shrinks images with few
  // colors to a 1-bit palette, which some tools (e.g. ffmpeg's scaler) misread.
  lodepng::State state;
  state.info_raw.colortype = LCT_RGB;
  state.info_raw.bitdepth = 8;
  state.info_png.color.colortype = LCT_RGB;
  state.info_png.color.bitdepth = 8;
  state.encoder.auto_convert = 0;
  std::vector<unsigned char> png;
  unsigned err = lodepng::encode(png, flipped, fb_width, fb_height, state);
  if (!err) err = lodepng::save_file(png, path);
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
