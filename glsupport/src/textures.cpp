/********************************************************
  Stanford Driving Software
  Copyright (c) 2011 Stanford University
  All rights reserved.

  Redistribution and use in source and binary forms, with 
  or without modification, are permitted provided that the 
  following conditions are met:

* Redistributions of source code must retain the above 
  copyright notice, this list of conditions and the 
  following disclaimer.
* Redistributions in binary form must reproduce the above
  copyright notice, this list of conditions and the 
  following disclaimer in the documentation and/or other
  materials provided with the distribution.
* The names of the contributors may not be used to endorse
  or promote products derived from this software
  without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
  PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
  DAMAGE.
 ********************************************************/


#include <stdio.h>
#include <string>
#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif
#include <opencv2/highgui/highgui.hpp>
#include <glsupport/textures.h>

namespace glsupport
{

Texture::Texture(int32_t width, int32_t height,
                 int32_t max_texture_size, bool invert)
  : invert_(invert)
  , needs_tex2D_(true)
  , max_texture_size_(max_texture_size)
{

  texture_width_ = 0;  //KAI debug
  texture_height_ = 0; //KAI debug

  updateSize(width, height);

  glGenTextures(1, &texture_id_);
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  glBindTexture(GL_TEXTURE_2D, texture_id_);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
}

Texture::~Texture()
{
  glDeleteTextures(1, &texture_id_);
}

bool Texture::updateSize(int32_t image_width, int32_t image_height)
{
  int32_t prev_texture_width = texture_width_;
  int32_t prev_texture_height = texture_height_;

  image_width_ = image_width;
  image_height_ = image_height;

  texture_width_ = 32;
  texture_height_ = 32;
  while (texture_width_ < image_width_) {texture_width_ *= 2;}
  while (texture_height_ < image_height_) {texture_height_ *= 2;}

  if (texture_height_ > max_texture_size_) {texture_height_ = max_texture_size_;}
  if (texture_width_ > max_texture_size_) {texture_width_ = max_texture_size_;}

  max_u_ = image_width_ / (double) texture_width_;
  max_v_ = image_height_ / (double) texture_height_;
  if (max_u_ > 1) max_u_ = 1;
  if (max_v_ > 1) max_v_ = 1;

  return (texture_width_ != prev_texture_width || texture_height_ != prev_texture_height);
}

void Texture::loadFromRaw(const uint8_t* data,
                          int32_t image_width, int32_t image_height,
                          int32_t max_texture_size, bool invert)
{
  max_texture_size_ = max_texture_size;
  invert_ = invert;
  updateSize(image_width, image_height);
  updateFromRaw(data, image_width, image_height);
}

void Texture::loadFromImage(const cv::Mat& image, int32_t max_texture_size, bool invert)
{
  return loadFromRaw(image.data, image.cols, image.rows, max_texture_size, invert);
}

void Texture::loadFromFile(const std::string& filename, int32_t max_texture_size, bool invert)
{
  cv::Mat image = cv::imread(filename, CV_LOAD_IMAGE_GRAYSCALE);
  loadFromRaw(image.data, image.cols, image.rows, max_texture_size, invert);
}

void Texture::loadFromBytes(const std::vector<uint8_t>& bytes, int32_t max_texture_size, bool invert)
{
  cv::Mat image = cv::imdecode(cv::Mat(bytes, false), CV_LOAD_IMAGE_GRAYSCALE);
  loadFromRaw(image.data, image.cols, image.rows, max_texture_size, invert);
}



void Texture::loadRGBFromRaw(const uint8_t* data,
                             int32_t image_width, int32_t image_height,
                             int32_t max_texture_size, bool invert)
{
  max_texture_size_ = max_texture_size;
  invert_ = invert;
  updateSize(image_width, image_height);
  updateFromRaw(data, image_width, image_height);
}

void Texture::loadRGBFromImage(const cv::Mat& image, int32_t max_texture_size, bool invert)
{
  return loadRGBFromRaw(image.data, image.cols, image.rows, max_texture_size, invert);
}

void Texture::loadRGBFromFile(const std::string& filename, int32_t max_texture_size, bool invert)
{
  cv::Mat image = cv::imread(filename, CV_LOAD_IMAGE_COLOR);
  loadRGBFromRaw(image.data, image.cols, image.rows, max_texture_size, invert);
}

void Texture::loadRGBFromBytes(const std::vector<uint8_t>& bytes, int32_t max_texture_size, bool invert)
{
  cv::Mat image = cv::imdecode(cv::Mat(bytes, false), CV_LOAD_IMAGE_COLOR);
  loadRGBFromRaw(image.data, image.cols, image.rows, max_texture_size, invert);
}



void Texture::loadRGBAFromRaw(const uint8_t* data,
                              int32_t image_width, int32_t image_height,
                              int32_t max_texture_size,
                              uint8_t invisr, uint8_t invisg, uint8_t invisb,
                              bool invert)
{

  uint8_t* rgba = (uint8_t *) calloc(1, image_width * image_height * 4);

  int32_t start, end, inc;
  if(invert) {
    start = image_height-1;
    end = 0;
    inc = -1;
  }
  else {
    start = 0;
    end = image_height-1;
    inc = 1;
  }
  for (int32_t r = start; r != end; r+=inc) {
    for (int32_t c = 0; c < image_width; c++) {
      if (data[0] != invisb || data[1] != invisg || data[2] != invisr) {
        rgba[(r * image_width + c) * 4 + 0] = *data++;
        rgba[(r * image_width + c) * 4 + 1] = *data++;
        rgba[(r * image_width + c) * 4 + 2] = *data++;
        rgba[(r * image_width + c) * 4 + 3] = 255;
      }
      else {
        rgba[(r * image_width + c) * 4 + 0] = 0;
        rgba[(r * image_width + c) * 4 + 1] = 0;
        rgba[(r * image_width + c) * 4 + 2] = 0;
        rgba[(r * image_width + c) * 4 + 3] = 0;
        data+=3;
      }
    }
  }

  updateRGBAFromRaw(rgba, image_width, image_height);
  free(rgba);
}

void Texture::loadRGBAFromImage(const cv::Mat& image, int32_t max_texture_size,
                                uint8_t invisr, uint8_t invisg, uint8_t invisb,
                                bool invert)
{
  loadRGBAFromRaw(image.data, image.cols, image.rows, max_texture_size, invisr, invisg, invisb, invert);
}

void Texture::loadRGBAFromFile(const std::string& filename,
                               int32_t max_texture_size,
                               uint8_t invisr, uint8_t invisg, uint8_t invisb,
                               bool invert)
{

  cv::Mat image = cv::imread(filename, CV_LOAD_IMAGE_ANYCOLOR);
  loadRGBAFromRaw(image.data, image.cols, image.rows, max_texture_size, invisr, invisg, invisb, invert);
}

void Texture::loadRGBAFromBytes(const std::vector<uint8_t>& bytes,
                                int32_t max_texture_size,
                                uint8_t invisr, uint8_t invisg, uint8_t invisb,
                                bool invert)
{
  cv::Mat image = cv::imdecode(cv::Mat(bytes, false), CV_LOAD_IMAGE_ANYCOLOR);
  loadRGBAFromRaw(image.data, image.cols, image.rows, max_texture_size, invisr, invisg, invisb, invert);
}


void Texture::updateFromRaw(const uint8_t* data, int32_t image_width, int32_t image_height)
{
  // resize the texture, if necessary
  if (image_width != image_width_ || image_height != image_height_) {
    updateSize(image_width, image_height);
    needs_tex2D_ = true; // ?!?
  }

  glBindTexture(GL_TEXTURE_2D, texture_id_);

  if (needs_tex2D_) {
    std::vector<uint8_t> texture_data(3 * texture_width_ * texture_height_, 0);

    resizeImageData(data, image_width, image_height, 3, &(texture_data[0]), texture_width_, texture_height_, invert_);
    glTexImage2D(GL_TEXTURE_2D, 0, 3, texture_width_, texture_height_, 0, GL_BGR, GL_UNSIGNED_BYTE, &(texture_data[0]));
    needs_tex2D_ = false;
  }
  else if (!invert_ && image_width <= texture_width_ && image_height <= texture_height_) {
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, image_width, image_height, GL_BGR, GL_UNSIGNED_BYTE, data);
  }
  else {
    std::vector<uint8_t> texture_data(3 * texture_width_ * texture_height_, 0);
    resizeImageData(data, image_width, image_height, 3, &(texture_data[0]), texture_width_, texture_height_, invert_);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, texture_width_, texture_height_, GL_BGR, GL_UNSIGNED_BYTE, &(texture_data[0]));
  }
}

void Texture::updateFromImage(const cv::Mat& image)
{
  updateFromRaw(image.data, image.cols, image.rows);
}

void Texture::updateFromBytes(const std::vector<uint8_t>& bytes)
{
  cv::Mat image = cv::imdecode(cv::Mat(bytes, false), CV_LOAD_IMAGE_GRAYSCALE);
  updateFromRaw(image.data, image.cols, image.rows);
}

void Texture::updateRGBAFromRaw(const uint8_t* data, int32_t image_width, int32_t image_height)
{
  // resize the texture, if necessary
  if (image_width != image_width_ || image_height != image_height_) {
    updateSize(image_width, image_height);
    needs_tex2D_ = true;
  }

  glBindTexture(GL_TEXTURE_2D, texture_id_);

  if (needs_tex2D_) {
    std::vector<uint8_t> texture_data(4 * texture_width_ * texture_height_, 0);
    resizeImageData(data, image_width, image_height, 4, &(texture_data[0]), texture_width_, texture_height_, invert_);
    glTexImage2D(GL_TEXTURE_2D, 0, 4, texture_width_, texture_height_, 0, GL_BGRA, GL_UNSIGNED_BYTE, &(texture_data[0]));
    needs_tex2D_ = false;
  }
  else if (!invert_ && image_width <= texture_width_ && image_height <= texture_height_) {
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, image_width, image_height, GL_BGRA, GL_UNSIGNED_BYTE, data);
  }
  else {
    std::vector<uint8_t> texture_data(4 * texture_width_ * texture_height_, 0);
    resizeImageData(data, image_width, image_height, 4, &(texture_data[0]), texture_width_, texture_height_, invert_);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, texture_width_, texture_height_, GL_BGRA, GL_UNSIGNED_BYTE, &(texture_data[0]));
  }
}

void Texture::draw(double x1, double y1, double x2, double y2, bool smooth)
{
  double dx, dy;

  glEnable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, texture_id_);

  if (smooth) {
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    dx = 1 / (2 * texture_width_);
    dy = 1 / (2 * texture_height_);
  }
  else {
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    dx = 0;
    dy = 0;
  }

  glColor3f(1, 1, 1);
  glBegin(GL_POLYGON);
  glTexCoord2f(dx, dy);                     glVertex2f(x1, y1);
  glTexCoord2f(max_u_ - dx, dy);            glVertex2f(x2, y1);
  glTexCoord2f(max_u_ - dx, max_v_ - dy);   glVertex2f(x2, y2);
  glTexCoord2f(dx, max_v_ - dy);            glVertex2f(x1, y2);
  glEnd();
  glDisable(GL_TEXTURE_2D);
}

void Texture::resizeImageData(const uint8_t* input,
                              int32_t input_width, int32_t input_height,
                              int32_t bytespp,
                              uint8_t* output,
                              int32_t output_width, int32_t output_height,
                              bool invert)
{
  int x, y, x2, y2;

  if (invert) {
    if (output_width < input_width || output_height < input_height) {
      for (x = 0; x < output_width; x++)
        for (y = 0; y < output_height; y++) {
          x2 = x / (double) output_width * input_width;
          y2 = y / (double) output_height * input_height;
          if (x2 >= input_width) x2 = input_width - 1;
          if (y2 >= input_height) y2 = input_height - 1;
          memcpy(output + (y * output_width + x) * bytespp,
                 input + ((input_height - y2 - 1) * input_width + x2) * bytespp,
                 bytespp);
        }
    }
    else {
      for (x = 0; x < input_width; x++)
        for (y = 0; y < input_height; y++)
          memcpy(output + (y * output_width + x) * bytespp,
                 input + ((input_height - y - 1) * input_width + x) * bytespp,
                 bytespp);
    }
  }
  else {
    if (output_width < input_width || output_height < input_height) {
      for (x = 0; x < output_width; x++)
        for (y = 0; y < output_height; y++) {
          x2 = x / (double) output_width * input_width;
          y2 = y / (double) output_height * input_height;
          if (x2 >= input_width) x2 = input_width - 1;
          if (y2 >= input_height) y2 = input_height - 1;
          memcpy(output + (y * output_width + x) * bytespp,
                 input + (y2 * input_width + x2) * bytespp,
                 bytespp);
        }
    }
    else {
      for (x = 0; x < input_width; x++)
        for (y = 0; y < input_height; y++)
          memcpy(output + (y * output_width + x) * bytespp,
                 input + (y * input_width + x) * bytespp,
                 bytespp);
    }
  }
}

} // namespace glsupport
