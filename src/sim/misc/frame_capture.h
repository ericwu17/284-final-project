#ifndef CGL_FRAME_CAPTURE_H
#define CGL_FRAME_CAPTURE_H

#include <string>

namespace FrameCapture {

// Reads the current back buffer (fb_width x fb_height pixels, i.e. the
// framebuffer size, which is 2x the window size on Retina displays), flips it
// so row 0 is the top of the image, and writes it to `path` as an RGB PNG.
// Creates parent directories as needed. Returns true on success.
bool save_framebuffer_png(const std::string &path, int fb_width, int fb_height);

// Local time as "YYYYmmdd-HHMMSS", for naming capture files and folders.
std::string timestamp();

} // namespace FrameCapture

#endif // CGL_FRAME_CAPTURE_H
