#include <iostream>
#include <fstream>
#include <nanogui/nanogui.h>
#include <stdio.h>
#include <stdlib.h>
#ifdef _WIN32
#include "misc/getopt.h" // getopt for windows
#else
#include <getopt.h>
#include <unistd.h>
#endif
#include <unordered_set>
#include <stdlib.h> // atoi for getopt inputs

#include "CGL/CGL.h"
#include "collision/plane.h"
#include "collision/sphere.h"
#include "cloth.h"
#include "clothSimulator.h"
#include "json.hpp"
#include "jello/jelloScene.h"
#include "misc/file_utils.h"

typedef uint32_t gid_t;

using namespace std;
using namespace nanogui;

using json = nlohmann::json;

#define msg(s) cerr << "[ClothSim] " << s << endl;

const string SPHERE = "sphere";
const string PLANE = "plane";
const string CLOTH = "cloth";

const unordered_set<string> VALID_KEYS = {SPHERE, PLANE, CLOTH};

ClothSimulator *app = nullptr;
GLFWwindow *window = nullptr;
Screen *screen = nullptr;

void error_callback(int error, const char* description) {
  puts(description);
}

void createGLContexts() {
  if (!glfwInit()) {
    return;
  }

  glfwSetTime(0);

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

  glfwWindowHint(GLFW_SAMPLES, 0);
  glfwWindowHint(GLFW_RED_BITS, 8);
  glfwWindowHint(GLFW_GREEN_BITS, 8);
  glfwWindowHint(GLFW_BLUE_BITS, 8);
  glfwWindowHint(GLFW_ALPHA_BITS, 8);
  glfwWindowHint(GLFW_STENCIL_BITS, 8);
  glfwWindowHint(GLFW_DEPTH_BITS, 24);
  glfwWindowHint(GLFW_RESIZABLE, GL_TRUE);

  // Create a GLFWwindow object
  window = glfwCreateWindow(800, 800, "Cloth Simulator", nullptr, nullptr);
  if (window == nullptr) {
    std::cout << "Failed to create GLFW window" << std::endl;
    glfwTerminate();
    return;
  }
  glfwMakeContextCurrent(window);

  if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
    throw std::runtime_error("Could not initialize GLAD!");
  }
  glGetError(); // pull and ignore unhandled errors like GL_INVALID_ENUM

  glClearColor(0.2f, 0.25f, 0.3f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT);

  // Create a nanogui screen and pass the glfw pointer to initialize
  screen = new Screen();
  screen->initialize(window, true);

  int width, height;
  glfwGetFramebufferSize(window, &width, &height);
  glViewport(0, 0, width, height);
  glfwSwapInterval(1);
  glfwSwapBuffers(window);
}

void setGLFWCallbacks() {
  glfwSetCursorPosCallback(window, [](GLFWwindow *, double x, double y) {
    if (!screen->cursorPosCallbackEvent(x, y)) {
      app->cursorPosCallbackEvent(x / screen->pixelRatio(),
                                  y / screen->pixelRatio());
    }
  });

  glfwSetMouseButtonCallback(
      window, [](GLFWwindow *, int button, int action, int modifiers) {
        if (!screen->mouseButtonCallbackEvent(button, action, modifiers) ||
            action == GLFW_RELEASE) {
          app->mouseButtonCallbackEvent(button, action, modifiers);
        }
      });

  glfwSetKeyCallback(
      window, [](GLFWwindow *, int key, int scancode, int action, int mods) {
        if (!screen->keyCallbackEvent(key, scancode, action, mods)) {
          app->keyCallbackEvent(key, scancode, action, mods);
        }
      });

  glfwSetCharCallback(window, [](GLFWwindow *, unsigned int codepoint) {
    screen->charCallbackEvent(codepoint);
  });

  glfwSetDropCallback(window,
                      [](GLFWwindow *, int count, const char **filenames) {
                        screen->dropCallbackEvent(count, filenames);
                        app->dropCallbackEvent(count, filenames);
                      });

  glfwSetScrollCallback(window, [](GLFWwindow *, double x, double y) {
    if (!screen->scrollCallbackEvent(x, y)) {
      app->scrollCallbackEvent(x, y);
    }
  });

  glfwSetFramebufferSizeCallback(window,
                                 [](GLFWwindow *, int width, int height) {
                                   screen->resizeCallbackEvent(width, height);
                                   app->resizeCallbackEvent(width, height);
                                 });
}

void usageError(const char *binaryName) {
  printf("Usage: %s [options]\n", binaryName);
  printf("Required program options:\n");
  printf("  -f     <STRING>    Filename of scene\n");
  printf("  -r     <STRING>    Project root.\n");
  printf("                     Should contain \"shaders/Default.vert\".\n");
  printf("                     Automatically searched for by default.\n");
  printf("  -a     <INT>       Sphere vertices latitude direction.\n");
  printf("  -o     <INT>       Sphere vertices longitude direction.\n");
  printf("  -R     <STRING>    Record frames (PNG, 30 fps of sim time) into this\n");
  printf("                     directory, starting unpaused at launch.\n");
  printf("  -T     <FLOAT>     With -R: stop after this many simulated seconds\n");
  printf("                     and exit.\n");
  printf("  -S     <STRING>    Shader to start with (Wireframe, Normal, Phong, ...).\n");
  printf("  -C     <INT>       Jello: camera preset 1 front, 2 three-quarter, 3 top.\n");
  printf("  -K                 Jello: start with the cutaway on.\n");
  printf("  -E                 Jello: start with the edge overlay on.\n");
  printf("  -O     <KEY=VALUE> Jello: override a scene value, e.g. -O mass_spring.ks=1000\n");
  printf("                     (repeatable; same as jellosim --set).\n");
  printf("  -U                 Jello: keep running when the stability check fails\n");
  printf("                     (default: pause), e.g. to record a blow-up.\n");
  printf("  -P     <STRING>    Save a screenshot of the first frame to this PNG\n");
  printf("                     and exit.\n");
  printf("\n");
  printf("Scenes with a top-level \"jello\" object (scene/jello/*.json) load a\n");
  printf("tet-mesh soft body instead of a cloth.\n");
  printf("\n");
  printf("Keys: P pause, N step, R reset, Space reset camera,\n");
  printf("      S screenshot, V start/stop recording (saved under out/capture/),\n");
  printf("      jello only: 1/2/3 camera presets, C cutaway, E edge overlay.\n");
  exit(-1);
}

void incompleteObjectError(const char *object, const char *attribute) {
  cout << "Incomplete " << object << " definition, missing " << attribute << endl;
  exit(-1);
}

bool loadObjectsFromFile(string filename, Cloth *cloth, ClothParameters *cp, vector<CollisionObject *>* objects, int sphere_num_lat, int sphere_num_lon) {
  // Read JSON from file
  ifstream i(filename);
  if (!i.good()) {
    return false;
  }
  json j;
  i >> j;

  // Loop over objects in scene
  for (json::iterator it = j.begin(); it != j.end(); ++it) {
    string key = it.key();

    // Check that object is valid
    unordered_set<string>::const_iterator query = VALID_KEYS.find(key);
    if (query == VALID_KEYS.end()) {
      cout << "Invalid scene object found: " << key << endl;
      exit(-1);
    }

    // Retrieve object
    json object = it.value();

    // Parse object depending on type (cloth, sphere, or plane)
    if (key == CLOTH) {
      // Cloth
      double width, height;
      int num_width_points, num_height_points;
      float thickness;
      e_orientation orientation;
      vector<vector<int>> pinned;

      auto it_width = object.find("width");
      if (it_width != object.end()) {
        width = *it_width;
      } else {
        incompleteObjectError("cloth", "width");
      }

      auto it_height = object.find("height");
      if (it_height != object.end()) {
        height = *it_height;
      } else {
        incompleteObjectError("cloth", "height");
      }

      auto it_num_width_points = object.find("num_width_points");
      if (it_num_width_points != object.end()) {
        num_width_points = *it_num_width_points;
      } else {
        incompleteObjectError("cloth", "num_width_points");
      }

      auto it_num_height_points = object.find("num_height_points");
      if (it_num_height_points != object.end()) {
        num_height_points = *it_num_height_points;
      } else {
        incompleteObjectError("cloth", "num_height_points");
      }

      auto it_thickness = object.find("thickness");
      if (it_thickness != object.end()) {
        thickness = *it_thickness;
      } else {
        incompleteObjectError("cloth", "thickness");
      }

      auto it_orientation = object.find("orientation");
      if (it_orientation != object.end()) {
        orientation = *it_orientation;
      } else {
        incompleteObjectError("cloth", "orientation");
      }

      auto it_pinned = object.find("pinned");
      if (it_pinned != object.end()) {
        vector<json> points = *it_pinned;
        for (auto pt : points) {
          vector<int> point = pt;
          pinned.push_back(point);
        }
      }

      cloth->width = width;
      cloth->height = height;
      cloth->num_width_points = num_width_points;
      cloth->num_height_points = num_height_points;
      cloth->thickness = thickness;
      cloth->orientation = orientation;
      cloth->pinned = pinned;

      // Cloth parameters
      bool enable_structural_constraints, enable_shearing_constraints, enable_bending_constraints;
      double damping, density, ks;

      auto it_enable_structural = object.find("enable_structural");
      if (it_enable_structural != object.end()) {
        enable_structural_constraints = *it_enable_structural;
      } else {
        incompleteObjectError("cloth", "enable_structural");
      }

      auto it_enable_shearing = object.find("enable_shearing");
      if (it_enable_shearing != object.end()) {
        enable_shearing_constraints = *it_enable_shearing;
      } else {
        incompleteObjectError("cloth", "it_enable_shearing");
      }

      auto it_enable_bending = object.find("enable_bending");
      if (it_enable_bending != object.end()) {
        enable_bending_constraints = *it_enable_bending;
      } else {
        incompleteObjectError("cloth", "it_enable_bending");
      }

      auto it_damping = object.find("damping");
      if (it_damping != object.end()) {
        damping = *it_damping;
      } else {
        incompleteObjectError("cloth", "damping");
      }

      auto it_density = object.find("density");
      if (it_density != object.end()) {
        density = *it_density;
      } else {
        incompleteObjectError("cloth", "density");
      }

      auto it_ks = object.find("ks");
      if (it_ks != object.end()) {
        ks = *it_ks;
      } else {
        incompleteObjectError("cloth", "ks");
      }

      cp->enable_structural_constraints = enable_structural_constraints;
      cp->enable_shearing_constraints = enable_shearing_constraints;
      cp->enable_bending_constraints = enable_bending_constraints;
      cp->density = density;
      cp->damping = damping;
      cp->ks = ks;
    } else if (key == SPHERE) {
      Vector3D origin;
      double radius, friction;

      auto it_origin = object.find("origin");
      if (it_origin != object.end()) {
        vector<double> vec_origin = *it_origin;
        origin = Vector3D(vec_origin[0], vec_origin[1], vec_origin[2]);
      } else {
        incompleteObjectError("sphere", "origin");
      }

      auto it_radius = object.find("radius");
      if (it_radius != object.end()) {
        radius = *it_radius;
      } else {
        incompleteObjectError("sphere", "radius");
      }

      auto it_friction = object.find("friction");
      if (it_friction != object.end()) {
        friction = *it_friction;
      } else {
        incompleteObjectError("sphere", "friction");
      }

      Sphere *s = new Sphere(origin, radius, friction, sphere_num_lat, sphere_num_lon);
      objects->push_back(s);
    } else { // PLANE
      Vector3D point, normal;
      double friction;

      auto it_point = object.find("point");
      if (it_point != object.end()) {
        vector<double> vec_point = *it_point;
        point = Vector3D(vec_point[0], vec_point[1], vec_point[2]);
      } else {
        incompleteObjectError("plane", "point");
      }

      auto it_normal = object.find("normal");
      if (it_normal != object.end()) {
        vector<double> vec_normal = *it_normal;
        normal = Vector3D(vec_normal[0], vec_normal[1], vec_normal[2]);
      } else {
        incompleteObjectError("plane", "normal");
      }

      auto it_friction = object.find("friction");
      if (it_friction != object.end()) {
        friction = *it_friction;
      } else {
        incompleteObjectError("plane", "friction");
      }

      Plane *p = new Plane(point, normal, friction);
      objects->push_back(p);
    }
  }

  i.close();
  
  return true;
}

bool is_valid_project_root(const std::string& search_path) {
    std::stringstream ss;
    ss << search_path;
    ss << "/";
    ss << "shaders/Default.vert";
    
    return FileUtils::file_exists(ss.str());
}

// Attempt to locate the project root automatically
bool find_project_root(const std::vector<std::string>& search_paths, std::string& retval) {
  
  for (std::string search_path : search_paths) {
    if (is_valid_project_root(search_path)) {
      retval = search_path;
      return true;
    }
  }
  return false;
}

int main(int argc, char **argv) {
  // Attempt to find project root
  std::vector<std::string> search_paths = {
    ".",
    "..",
    "../..",
    "../../.."
  };
  std::string project_root;
  bool found_project_root = find_project_root(search_paths, project_root);
  
  Cloth cloth;
  ClothParameters cp;
  vector<CollisionObject *> objects;
  
  int c;
  
  int sphere_num_lat = 40;
  int sphere_num_lon = 40;
  
  std::string file_to_load_from;
  bool file_specified = false;

  std::string record_dir;
  double record_seconds = -1;
  std::string shader_name;
  int camera_preset = 0;
  bool start_cutaway = false;
  bool start_edges = false;
  bool ignore_instability = false;
  std::vector<std::pair<std::string, std::string>> scene_overrides;
  std::string screenshot_path;
  
  while ((c = getopt (argc, argv, "f:r:a:o:R:T:S:C:KEUO:P:")) != -1) {
    switch (c) {
      case 'f': {
        file_to_load_from = optarg;
        file_specified = true;
        break;
      }
      case 'r': {
        project_root = optarg;
        if (!is_valid_project_root(project_root)) {
          std::cout << "Warn: Could not find required file \"shaders/Default.vert\" in specified project root: " << project_root << std::endl;
        }
        found_project_root = true;
        break;
      }
      case 'a': {
        int arg_int = atoi(optarg);
        if (arg_int < 1) {
          arg_int = 1;
        }
        sphere_num_lat = arg_int;
        break;
      }
      case 'o': {
        int arg_int = atoi(optarg);
        if (arg_int < 1) {
          arg_int = 1;
        }
        sphere_num_lon = arg_int;
        break;
      }
      case 'R': {
        record_dir = optarg;
        break;
      }
      case 'T': {
        record_seconds = atof(optarg);
        break;
      }
      case 'S': {
        shader_name = optarg;
        break;
      }
      case 'C': {
        camera_preset = atoi(optarg);
        break;
      }
      case 'K': {
        start_cutaway = true;
        break;
      }
      case 'E': {
        start_edges = true;
        break;
      }
      case 'U': {
        ignore_instability = true;
        break;
      }
      case 'O': {
        std::string kv = optarg;
        size_t eq = kv.find('=');
        if (eq == std::string::npos) {
          std::cout << "Error: -O expects KEY=VALUE, got " << kv << std::endl;
          return -1;
        }
        scene_overrides.push_back({kv.substr(0, eq), kv.substr(eq + 1)});
        break;
      }
      case 'P': {
        screenshot_path = optarg;
        break;
      }
      default: {
        usageError(argv[0]);
        break;
      }
    }
  }
  
  if (!found_project_root) {
    std::cout << "Error: Could not find required file \"shaders/Default.vert\" anywhere!" << std::endl;
    return -1;
  } else {
    std::cout << "Loading files starting from: " << project_root << std::endl;
  }

  if (!file_specified) { // No arguments, default initialization
    std::stringstream def_fname;
    def_fname << project_root;
    def_fname << "/scene/pinned2.json";
    file_to_load_from = def_fname.str();
  }
  
  // Jello scenes build a tet-mesh soft body; everything else is a cloth scene.
  bool jello_mode = Jello::JelloScene::isJelloScene(file_to_load_from);
  Jello::JelloScene jello_scene;
  std::unique_ptr<Jello::SoftBody> jello_body;
  std::vector<std::unique_ptr<Jello::Collider>> jello_colliders;
  std::vector<const Jello::Collider *> jello_collider_ptrs;
  if (jello_mode) {
    std::string error;
    if (!jello_scene.load(file_to_load_from, &error, scene_overrides) || !(jello_body = jello_scene.buildBody(&error))) {
      std::cout << "Error: " << error << std::endl;
      return -1;
    }
    jello_colliders = jello_scene.buildColliders();
    for (const auto &c : jello_colliders) jello_collider_ptrs.push_back(c.get());
    const Jello::TetMesh &mesh = jello_body->mesh();
    std::cout << "Jello: " << mesh.nx << "x" << mesh.ny << "x" << mesh.nz << " cells, "
              << mesh.numVerts() << " verts, " << mesh.tets.size() << " tets, "
              << mesh.surfaceTris.size() << " surface triangles, solver " << jello_body->name()
              << std::endl;
  } else {
    bool success = loadObjectsFromFile(file_to_load_from, &cloth, &cp, &objects, sphere_num_lat, sphere_num_lon);
    if (!success) {
      std::cout << "Warn: Unable to load from file: " << file_to_load_from << std::endl;
    }
  }

  glfwSetErrorCallback(error_callback);

  createGLContexts();

  // Initialize the Cloth object
  if (!jello_mode) {
    cloth.buildGrid();
    cloth.buildClothMesh();
  }

  // Initialize the ClothSimulator object
  app = new ClothSimulator(project_root, screen);
  app->loadCloth(&cloth);
  app->loadClothParameters(&cp);
  app->loadCollisionObjects(&objects);
  if (jello_mode) {
    app->loadSoftBody(jello_body.get(), jello_collider_ptrs, jello_scene.hasGround, jello_scene.groundY);
    app->setPauseOnInstability(!ignore_instability);
    app->setGravity(jello_scene.gravity);
  }
  if (!shader_name.empty() && !app->setShaderByName(shader_name)) {
    std::cout << "Warn: no shader named \"" << shader_name << "\"" << std::endl;
  }
  app->init();
  if (camera_preset) app->setCameraPreset(camera_preset);
  if (start_cutaway) app->setCutaway(true);
  if (start_edges) app->setShowEdges(true);
  if (!screenshot_path.empty()) app->requestScreenshotTo(screenshot_path, /*exit_after=*/true);

  if (!record_dir.empty()) {
    app->startRecording(record_dir, record_seconds, /*exit_when_done=*/record_seconds > 0);
  }

  // Call this after all the widgets have been defined

  screen->setVisible(true);
  screen->performLayout();

  // Attach callbacks to the GLFW window

  setGLFWCallbacks();

  while (!glfwWindowShouldClose(window)) {
    glfwPollEvents();

    glClearColor(0.25f, 0.25f, 0.25f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    app->drawContents();

    // Capture before the GUI panel is drawn so frames show only the scene.
    int fb_width, fb_height;
    glfwGetFramebufferSize(window, &fb_width, &fb_height);
    app->captureFrame(fb_width, fb_height);

    // Draw nanogui
    screen->drawContents();
    screen->drawWidgets();

    glfwSwapBuffers(window);

    if (!app->isAlive()) {
      glfwSetWindowShouldClose(window, 1);
    }
  }

  return 0;
}
