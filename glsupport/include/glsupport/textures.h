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


#ifndef GLSUPPORT_TEXTURE_H_
#define GLSUPPORT_TEXTURE_H_

#include <string>
#include <vector>
#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif
#include <opencv2/core/core.hpp>

namespace glsupport
{

class Texture
{
public:
  Texture(int32_t width=32, int32_t height=32, int32_t max_texture_size=1024, bool invert=false);
  virtual ~Texture();

  void loadFromRaw(const uint8_t* data, int32_t image_width, int32_t image_height, int32_t max_texture_size, bool invert);
  void loadFromImage(const cv::Mat& image, int32_t max_texture_size, bool invert);
  void loadFromFile(const std::string& filename, int32_t max_texture_size, bool invert);
  void loadFromBytes(const std::vector<uint8_t>& bytes, int32_t max_texture_size, bool invert);

  void loadRGBFromRaw(const uint8_t* data, int32_t image_width, int32_t image_height, int32_t max_texture_size, bool invert);
  void loadRGBFromImage(const cv::Mat& image, int32_t max_texture_size, bool invert);
  void loadRGBFromFile(const std::string& filename, int32_t max_texture_size, bool invert);
  void loadRGBFromBytes(const std::vector<uint8_t>& bytes, int32_t max_texture_size, bool invert);

  void loadRGBAFromRaw(const uint8_t* data, int32_t image_width, int32_t image_height, int32_t max_texture_size,
                       uint8_t invisr, uint8_t invisg, uint8_t invisb, bool invert);
  void loadRGBAFromImage(const cv::Mat& image, int32_t max_texture_size, uint8_t invisr,
                       uint8_t invisg, uint8_t invisb, bool invert);
  void loadRGBAFromFile(const std::string& filename, int32_t max_texture_size, uint8_t invisr,
                        uint8_t invisg, uint8_t invisb, bool invert);
  void loadRGBAFromBytes(const std::vector<uint8_t>& bytes, int32_t max_texture_size,
                        uint8_t invisr, uint8_t invisg, uint8_t invisb, bool invert);

  void updateFromRaw(const uint8_t* data, int32_t image_width, int32_t image_height);
  void updateFromImage(const cv::Mat& image);
  void updateFromBytes(const std::vector<uint8_t>& bytes);
  void updateRGBAFromRaw(const uint8_t* data, int32_t image_width, int32_t image_height);

  void draw(double x1, double y1, double x2, double y2, bool smooth);

  float maxU() const {return max_u_;}
  float maxV() const {return max_v_;}
  int32_t imageWidth() const {return image_width_;}
  int32_t imageHeight() const {return image_height_;}

  int32_t textureWidth() const {return texture_width_;}
  int32_t textureHeight() const {return texture_height_;}
  GLuint glTextureId() const {return texture_id_;}

private:
  bool updateSize(int32_t image_width, int32_t image_height);

  static void resizeImageData(const uint8_t* input,
                              int32_t input_width, int32_t input_height,
                              int32_t bytespp,
                              uint8_t* output,
                              int32_t output_width, int32_t output_height,
                              bool invert);

private:
  bool invert_, needs_tex2D_;
  int32_t max_texture_size_;
  GLuint texture_id_;
  int32_t texture_width_, texture_height_;
  int32_t image_width_, image_height_;
  float max_u_, max_v_;
};

} // namespace glsupport

#endif //GLSUPPORT_TEXTURE_H_
