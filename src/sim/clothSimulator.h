#ifndef CGL_CLOTH_SIMULATOR_H
#define CGL_CLOTH_SIMULATOR_H

#include <nanogui/nanogui.h>
#include <memory>

#include "camera.h"
#include "cloth.h"
#include "collision/collisionObject.h"

using namespace nanogui;

struct UserShader;
enum ShaderTypeHint { WIREFRAME = 0, NORMALS = 1, PHONG = 2 };

class ClothSimulator {
public:
  ClothSimulator(std::string project_root, Screen *screen);
  ~ClothSimulator();

  void init();

  void loadCloth(Cloth *cloth);
  void loadClothParameters(ClothParameters *cp);
  void loadCollisionObjects(vector<CollisionObject *> *objects);
  virtual bool isAlive();
  virtual void drawContents();

  // Screen events

  virtual bool cursorPosCallbackEvent(double x, double y);
  virtual bool mouseButtonCallbackEvent(int button, int action, int modifiers);
  virtual bool keyCallbackEvent(int key, int scancode, int action, int mods);
  virtual bool dropCallbackEvent(int count, const char **filenames);
  virtual bool scrollCallbackEvent(double x, double y);
  virtual bool resizeCallbackEvent(int width, int height);

  // Frame capture. Call captureFrame() once per loop iteration, after
  // drawContents() and before the GUI widgets are drawn, so captures show
  // the scene without the NanoGUI panel.
  //
  // Recording saves frames on a fixed simulated-time grid (capture_fps), not
  // per drawn frame, so the resulting video plays back in real time no matter
  // how fast the simulation runs or what frames_per_sec is set to.
  void captureFrame(int fb_width, int fb_height);
  void requestScreenshot();
  // Starts recording into `dir`. If max_seconds > 0, stops after that much
  // simulated time; with exit_when_done it also unpauses at the start and
  // closes the app at the end (used by the -R/-T command-line options).
  void startRecording(const std::string &dir, double max_seconds = -1,
                      bool exit_when_done = false);
  void stopRecording();

private:
  virtual void initGUI(Screen *screen);
  void drawWireframe(GLShader &shader);
  void drawNormals(GLShader &shader);
  void drawPhong(GLShader &shader);
  
  void load_shaders();
  void load_textures();
  
  // File management
  
  std::string m_project_root;

  // Camera methods

  virtual void resetCamera();
  virtual Matrix4f getProjectionMatrix();
  virtual Matrix4f getViewMatrix();

  // Default simulation values

  int frames_per_sec = 90;
  int simulation_steps = 30;

  CGL::Vector3D gravity = CGL::Vector3D(0, -9.8, 0);
  nanogui::Color color = nanogui::Color(1.0f, 1.0f, 1.0f, 1.0f);

  Cloth *cloth;
  ClothParameters *cp;
  vector<CollisionObject *> *collision_objects;

  // OpenGL attributes

  int active_shader_idx = 0;

  vector<UserShader> shaders;
  vector<std::string> shaders_combobox_names;
  
  // OpenGL textures
  
  Vector3D m_gl_texture_1_size;
  Vector3D m_gl_texture_2_size;
  Vector3D m_gl_texture_3_size;
  Vector3D m_gl_texture_4_size;
  GLuint m_gl_texture_1;
  GLuint m_gl_texture_2;
  GLuint m_gl_texture_3;
  GLuint m_gl_texture_4;
  GLuint m_gl_cubemap_tex;
  
  // OpenGL customizable inputs
  
  double m_normal_scaling = 2.0;
  double m_height_scaling = 0.1;

  // Camera attributes

  CGL::Camera camera;
  CGL::Camera canonicalCamera;

  double view_distance;
  double canonical_view_distance;
  double min_view_distance;
  double max_view_distance;

  double scroll_rate;

  // Screen methods

  Screen *screen;
  void mouseLeftDragged(double x, double y);
  void mouseRightDragged(double x, double y);
  void mouseMoved(double x, double y);

  // Mouse flags

  bool left_down = false;
  bool right_down = false;
  bool middle_down = false;

  // Keyboard flags

  bool ctrl_down = false;

  // Simulation flags

  bool is_paused = true;

  // Capture state

  static constexpr double capture_fps = 30.0;
  double sim_time = 0;            // simulated seconds since launch
  bool screenshot_requested = false;
  bool recording = false;
  bool record_exit_when_done = false;
  std::string record_dir;
  int record_frame_idx = 0;
  double record_next_time = 0;    // sim time of the next frame to save
  double record_stop_time = -1;   // sim time to stop at (< 0: no limit)

  // Screen attributes

  int mouse_x;
  int mouse_y;

  int screen_w;
  int screen_h;

  bool is_alive = true;

  Vector2i default_window_size = Vector2i(1024, 800);
};

struct UserShader {
  UserShader(std::string display_name, std::shared_ptr<GLShader> nanogui_shader, ShaderTypeHint type_hint)
  : display_name(display_name)
  , nanogui_shader(nanogui_shader)
  , type_hint(type_hint) {
  }
  
  std::shared_ptr<GLShader> nanogui_shader;
  std::string display_name;
  ShaderTypeHint type_hint;
  
};

#endif // CGL_CLOTH_SIM_H
