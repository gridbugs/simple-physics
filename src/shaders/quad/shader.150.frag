#version 150 core

in vec3 v_Colour;
out vec4 Target;

void main() {
    Target = vec4(v_Colour, 1);
}
