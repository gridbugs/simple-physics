#version 150 core

in vec2 a_CornerZeroToOne;
in vec2 i_PositionOfTopLeftInPixels;
in vec2 i_DimensionsInPixels;
in vec3 i_Colour;

uniform WindowProperties {
    vec2 u_WindowSizeInPixels;
};

out vec3 v_Colour;

void main() {

    vec2 pixel_offset = a_CornerZeroToOne * i_DimensionsInPixels;
    vec2 pixel_coord = i_PositionOfTopLeftInPixels + pixel_offset;

    vec2 screen_coord = vec2(
        pixel_coord.x / u_WindowSizeInPixels.x * 2 - 1,
        1 - pixel_coord.y / u_WindowSizeInPixels.y * 2);

    v_Colour = i_Colour;

    gl_Position = vec4(screen_coord, 0, 1);
}
