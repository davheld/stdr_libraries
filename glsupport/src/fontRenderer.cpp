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

#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif
#include "helvetica.h"
#include <glsupport/fontRenderer.h>

namespace glsupport
{

FontRenderer::FontRenderer() : font_(NULL), pixmap_font_(NULL), size_(1)
{
  if (!addFont("helvetica", helvetica_resource_data, helvetica_resource_size)) {
    BOOST_THROW_EXCEPTION(FontRendererException() <<stdr::ex::MsgInfo("Cannot add default font resource."));
  }
  setFont("helvetica");
  font_->UseDisplayList(false);
  pixmap_font_->UseDisplayList(false);
}

FontRenderer::~FontRenderer()
{
  std::map<std::string, FTFont*>::const_iterator fit, fit_end;
  for(uint32_t i=0; i<fonts_.size(); i++) {
    delete fonts_[i];
    delete pixmap_fonts_[i];
  }
}

bool FontRenderer::addFont(const std::string& file_name)
{
  FTFont* font = NULL, *pixmap_font = NULL;

  try {
    font = new FTGLPolygonFont(file_name.c_str());
  }
  catch (...) {
    BOOST_THROW_EXCEPTION(FontRendererException() <<stdr::ex::MsgInfo("Cannot add font " + file_name));
  }

  if(font->Error()){
    BOOST_THROW_EXCEPTION(FontRendererException() <<stdr::ex::MsgInfo("Cannot add font " + file_name));
  }

  try {
    pixmap_font = new FTGLPixmapFont(file_name.c_str());
  }
  catch (...) {
    BOOST_THROW_EXCEPTION(FontRendererException() <<stdr::ex::MsgInfo("Cannot add pixmap font " + file_name));
  }

  if(pixmap_font->Error()){
    BOOST_THROW_EXCEPTION(FontRendererException() <<stdr::ex::MsgInfo("Cannot add pixmap font " + file_name));
  }

  font->FaceSize(size_);
  pixmap_font->FaceSize(size_);

  //TODO: Extract font name from file / file name
  std::pair<std::map<std::string, uint32_t>::iterator, bool> pair = font_map_.insert(std::make_pair(file_name.c_str(), sizeof(fonts_)));

  if (!pair.second) {
    delete font;
    delete pixmap_font;
    return false;
  }

  fonts_.push_back(font);
  pixmap_fonts_.push_back(pixmap_font);

  return true;
}

bool FontRenderer::addFont(const std::string& font_name, const uint8_t* mem, size_t mem_size)
{
  FTFont* font = NULL, *pixmap_font = NULL;

  if (!mem) {
    return false;
  }

  try {
    font = new FTGLPolygonFont(mem, mem_size);
  }
  catch (...) {
    BOOST_THROW_EXCEPTION(FontRendererException() <<stdr::ex::MsgInfo("Cannot add font " + font_name));
  }

  if(font->Error()){
    BOOST_THROW_EXCEPTION(FontRendererException() <<stdr::ex::MsgInfo("Cannot add font " + font_name));
  }

  try {
    pixmap_font = new FTGLPixmapFont(mem, mem_size);
  }
  catch (...) {
    BOOST_THROW_EXCEPTION(FontRendererException() <<stdr::ex::MsgInfo("Cannot add pixmap font " + font_name));
  }

  if(pixmap_font->Error()){
    BOOST_THROW_EXCEPTION(FontRendererException() <<stdr::ex::MsgInfo("Cannot add pixmap font " + font_name));
  }

  font->FaceSize(size_);
  pixmap_font->FaceSize(size_);

  std::pair<std::map<std::string, uint32_t>::iterator, bool> pair = font_map_.insert(std::make_pair(font_name, fonts_.size()));

  if (!pair.second) {
    delete font;
    delete pixmap_font;
    return false;
  }

  fonts_.push_back(font);
  pixmap_fonts_.push_back(pixmap_font);

  return true;
}

void FontRenderer::removeFont(const std::string& font_name)
{
  std::map<std::string, uint32_t>::iterator fit = font_map_.find(font_name);

  // TODO: buggy, reindex map...
  if (fit == font_map_.end()) {
    return;
  }

  uint32_t index = (*fit).second;

  font_map_.erase(fit);

  FTFont* font = fonts_[index];
  fonts_.erase(fonts_.begin() + index);
  delete font;

  font = pixmap_fonts_[index];
  pixmap_fonts_.erase(pixmap_fonts_.begin() + index);
  delete font;
}

void FontRenderer::setFont(const std::string& font_name)
{
  std::map<std::string, uint32_t>::iterator fit = font_map_.find(font_name);

  if (fit == font_map_.end()) {
    BOOST_THROW_EXCEPTION(FontRendererException() <<stdr::ex::MsgInfo("Font is unavailable. Please use addFont() first."));
  }

  font_ = fonts_[fit->second];
  pixmap_font_ = pixmap_fonts_[fit->second];
}


void FontRenderer::drawString2D(const std::string& text)
{
  glPushMatrix();
  font_->Render(text.c_str());
  glPopMatrix();
}

void FontRenderer::drawString2D(const std::string& text, float x, float y)
{
  glPushMatrix();
  glTranslatef(x, y, 0.);
  font_->Render(text.c_str());
  glPopMatrix();
}

void FontRenderer::drawPixmapString2D(const std::string& text, float x, float y)
{
  glPushMatrix();
  pixmap_font_->Render(text.c_str(), -1, FTPoint(x, y));
  glPopMatrix();
}

void FontRenderer::drawString3D(const std::string& text, float x, float y, float z)
{
  glPushMatrix();
  glTranslatef(x, y, z);
  font_->Render(text.c_str());
  glPopMatrix();
}

void FontRenderer::setFontSize(uint32_t size)
{
  font_->FaceSize(size_);
  pixmap_font_->FaceSize(size_);
  size_=size;
}

} // namespace glsupport
