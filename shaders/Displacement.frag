#version 330

uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

uniform vec4 u_color;

uniform sampler2D u_texture_2;
uniform vec2 u_texture_2_size;

uniform float u_normal_scaling;
uniform float u_height_scaling;

in vec4 v_position;
in vec4 v_normal;
in vec4 v_tangent;
in vec2 v_uv;

out vec4 out_color;

// Height of the height map at texture coordinate uv.
// We use the red channel of the texture as the height.
float h(vec2 uv) {
  return texture(u_texture_2, uv).r;
}

void main() {
  vec3 position = v_position.xyz;
  vec3 normal = normalize(v_normal.xyz);
  vec3 tangent = normalize(v_tangent.xyz);

  // ---- Compute the bumped normal ----

  // Tangent-space to world-space matrix. Columns are t, b, n.
  vec3 bitangent = cross(normal, tangent);
  mat3 TBN = mat3(tangent, bitangent, normal);

  // Size of one texel in uv coordinates
  float texture_width = u_texture_2_size.x;
  float texture_height = u_texture_2_size.y;
  float u = v_uv.x;
  float v = v_uv.y;

  // How much the height changes when we move one texel in u and in v
  float height_here = h(vec2(u, v));
  float dU = (h(vec2(u + 1.0 / texture_width, v)) - height_here) * u_height_scaling * u_normal_scaling;
  float dV = (h(vec2(u, v + 1.0 / texture_height)) - height_here) * u_height_scaling * u_normal_scaling;

  // Normal in tangent space, then transformed back to world space
  vec3 local_normal = vec3(-dU, -dV, 1.0);
  vec3 bumped_normal = normalize(TBN * local_normal);

  // ---- Blinn-Phong shading using the bumped normal ----
  float k_a = 0.1;
  vec3 I_a = vec3(1.0, 1.0, 1.0);
  float k_d = 1.0;
  float k_s = 0.5;
  float p = 100.0;

  vec3 to_light = u_light_pos - position;
  float r_squared = dot(to_light, to_light);
  vec3 light_dir = normalize(to_light);
  vec3 view_dir = normalize(u_cam_pos - position);
  vec3 half_vector = normalize(light_dir + view_dir);

  vec3 irradiance = u_light_intensity / r_squared;

  vec3 ambient = k_a * I_a;
  vec3 diffuse = k_d * irradiance * max(0.0, dot(bumped_normal, light_dir));
  vec3 specular = k_s * irradiance * pow(max(0.0, dot(bumped_normal, half_vector)), p);

  out_color = vec4(ambient + diffuse + specular, 1.0);
}

