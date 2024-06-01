//#pragma once
#ifndef __exp_shad_
#define __exp_shad_

//
// Shader used for planar expansion
//

// Shader set and parameters struct 
struct ExpansionShader
{
  // Vertex shader source program file name
  const char *vsrc;

  // Fragment shader source program file name
  const char *fsrc;

  // width and height
  const int width, height;

  // Radius and center position of image circle
  const float circle[4];
};

// Shader type
constexpr ExpansionShader shader_type[] =
{
  // 0: normal camera
  { "fixed.vert",     "normal.frag",    640,  480, 1.0f, 1.0f, 0.0f, 0.0f },

  // 1: Rotate the viewpoint with a regular camera
  { "rectangle.vert", "normal.frag",    640,  480, 1.0f, 1.0f, 0.0f, 0.0f },

  // 2: Equirectangular projection image (to remove vertical lines, set GL_CLAMP_TO_BORDER to GL_REPEAT)
  { "panorama.vert",  "panorama.frag", 1280,  720, 1.0f, 1.0f, 0.0f, 0.0f },

  // 3: 180° fisheye camera: 3.1415927 / 2 (≒ 180°/ 2))
  { "fisheye.vert",   "normal.frag",   1280,  720, 1.570796327f, 1.570796327f, 0.0f, 0.0f },

  // 4: 180° fisheye camera (FUJINON FE185C046HA-1 + SENTECH STC-MCE132U3V): 3.5779249 / 2 (≒ 205°/ 2)
  { "fisheye.vert",   "normal.frag",   1280, 1024, 1.797689129f, 1.797689129f, 0.0f, 0.0f },

  // 5: 206° fisheye camera (Kodak PIXPRO SP360 4K, with image stabilization): 3.5953783 / 2 (≒ 206°/ 2)
  { "fisheye.vert",   "normal.frag",   1440, 1440, 1.797689129f, 1.797689129f, 0.0f, 0.0f },

  // 6: 235° fisheye camera (Kodak PIXPRO SP360 4K, no image stabilization): 4.1015237 / 2 (≒ 235°/ 2)
  { "fisheye.vert",   "normal.frag",   1440, 1440, 2.050761871f, 2.050761871f, 0.0f, 0.0f },

  // 7: RICHO THETA USB live streaming video: (value determined by manual adjustment)
  { "theta.vert",     "theta.frag",    1280,  720, 1.003f, 1.003f, 0.0f, -0.002f },

  // 8: RICHO THETA's HDMI live streaming video: (value determined by manual adjustment)
  { "theta.vert",     "theta.frag",    1920,  1080, 1.003f, 1.003f, 0.0f, -0.002f }
};