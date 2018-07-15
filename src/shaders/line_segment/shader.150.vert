#version 150 core

in vec2 a_CornerZeroToOne;
in vec2 i_Start;
in vec2 i_End;
in vec3 i_Colour;

uniform WindowProperties {
    vec2 u_WindowSizeInPixels;
};

out vec3 v_Colour;

const float WIDTH = 2;

void main() {

    vec2 start_to_end = i_End - i_Start;
    vec2 width = normalize(vec2(-start_to_end.y, start_to_end.x)) * WIDTH;
    vec2 corner = i_Start - width / 2;
    vec2 pixel_coord = corner + (start_to_end * a_CornerZeroToOne.x) + (width * a_CornerZeroToOne.y) ;

    vec2 screen_coord = vec2(
        pixel_coord.x / u_WindowSizeInPixels.x * 2 - 1,
        1 - pixel_coord.y / u_WindowSizeInPixels.y * 2);

    v_Colour = i_Colour;

    gl_Position = vec4(screen_coord, 0, 1);
}
