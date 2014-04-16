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

#ifndef GLSUPPORT_FONTRENDERER_H_
#define GLSUPPORT_FONTRENDERER_H_

#include <map>
#include <vector>
#include <string>

#include <FTGL/ftgl.h>
#include <FTGL/FTGLPolygonFont.h>

#include <stdr_lib/exception.h>

namespace glsupport
{

struct FontRendererException : virtual stdr::ex::ExceptionBase { };

class FontRenderer
{
public:
  FontRenderer();
  virtual ~FontRenderer();

  bool addFont(const std::string& file_name);
  bool addFont(const std::string& font_name, const uint8_t* mem, size_t mem_size);
  void removeFont(const std::string& font_name);
  void setFont(const std::string& font_name);

  int numFonts() const {return font_map_.size();}
  void setFontSize(uint32_t size);
  uint32_t fontSize() const {return size_;}

  float stringWidth(const std::string& text) const {
    return font_->Advance(text.c_str(), text.size());
  }

  float stringHeight(const std::string& text) const {
    float llx, lly, llz, urx, ury, urz;
    font_->BBox(text.c_str(), llx, lly, llz, urx, ury, urz);
    return ury-lly;
  }

  void stringBBox(const std::string& text, float& w, float& h) const {
    float llx, lly, llz, urx, ury, urz;
    font_->BBox(text.c_str(), llx, lly, llz, urx, ury, urz);
    w = urx-llx;
    h = ury-lly;
  }

  void drawString2D(const std::string& text);
  void drawString2D(const std::string& text, float x, float y);
  void drawPixmapString2D(const std::string& text, float x, float y);
  void drawString3D(const std::string&, float x, float y, float z);

private:
  std::map<std::string, uint32_t> font_map_;
  std::vector<FTFont*> fonts_;
  std::vector<FTFont*> pixmap_fonts_;
  FTFont* font_;
  FTFont* pixmap_font_;
  uint32_t size_;
};

} // namespace glsupport

#endif // GLSUPPORT_FONTRENDERER_H_
