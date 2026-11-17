#version 330

uniform vec4 u_color;
uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

in vec4 v_position;
in vec4 v_normal;
in vec2 v_uv;

out vec4 out_color;

void main() {
  // Material and lighting parameters
  float k_a = 0.1;
  vec3 I_a = vec3(1.0, 1.0, 1.0);
  float k_d = 1.0;
  float k_s = 0.5;
  float p = 100.0;

  vec3 position = v_position.xyz;
  vec3 normal = normalize(v_normal.xyz);

  // Vector from the surface point to the light
  vec3 to_light = u_light_pos - position;
  float r_squared = dot(to_light, to_light);
  vec3 light_dir = normalize(to_light);

  // Unit vector from the surface point to the camera
  vec3 view_dir = normalize(u_cam_pos - position);

  // Blinn-Phong half vector
  vec3 half_vector = normalize(light_dir + view_dir);

  vec3 irradiance = u_light_intensity / r_squared;

  vec3 ambient = k_a * I_a;
  vec3 diffuse = k_d * irradiance * max(0.0, dot(normal, light_dir));
  vec3 specular = k_s * irradiance * pow(max(0.0, dot(normal, half_vector)), p);

  out_color = vec4(ambient + diffuse + specular, 1.0);
}

