#version 330

uniform mat4 u_view_projection;
uniform mat4 u_model;

uniform sampler2D u_texture_2;
uniform vec2 u_texture_2_size;

uniform float u_normal_scaling;
uniform float u_height_scaling;

in vec4 in_position;
in vec4 in_normal;
in vec4 in_tangent;
in vec2 in_uv;

out vec4 v_position;
out vec4 v_normal;
out vec2 v_uv;
out vec4 v_tangent;

// Height of the height map at texture coordinate uv.
// We use the red channel of the texture as the height.
float h(vec2 uv) {
  return texture(u_texture_2, uv).r;
}

void main() {
  // Move the vertex along its original (model-space) normal by the
  // height from the height map: p' = p + n * h(u, v) * k_h
  vec3 original_position = in_position.xyz;
  vec3 original_normal = normalize(in_normal.xyz);
  vec3 displaced_position = original_position + original_normal * h(in_uv) * u_height_scaling;
  vec4 displaced_position_4 = vec4(displaced_position, 1.0);

  v_position = u_model * displaced_position_4;
  v_normal = normalize(u_model * in_normal);
  v_uv = in_uv;
  v_tangent = normalize(u_model * in_tangent);
  gl_Position = u_view_projection * u_model * displaced_position_4;
}
