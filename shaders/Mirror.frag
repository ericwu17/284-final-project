#version 330


uniform vec3 u_cam_pos;

uniform samplerCube u_texture_cubemap;

in vec4 v_position;
in vec4 v_normal;
in vec4 v_tangent;

out vec4 out_color;

void main() {
  vec3 position = v_position.xyz;
  vec3 normal = normalize(v_normal.xyz);

  // Outgoing direction: from the surface point toward the camera
  vec3 w_o = normalize(u_cam_pos - position);

  // Reflect w_o about the normal to get the incoming direction w_i.
  // GLSL's reflect() expects the vector pointing INTO the surface, so we pass -w_o.
  vec3 w_i = reflect(-w_o, normal);

  out_color = texture(u_texture_cubemap, w_i);
  out_color.a = 1.0;
}
