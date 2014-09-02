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
#include <iostream>
#include <boost/format.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <bag_of_tricks/image_view_window.h>



ImageViewWindow::ImageViewWindow(const std::string &name, int delay)
  : win_name_(name), delay_(delay), pan_zoom_mode_(PZM_NONE)
{
  boost::lock_guard<boost::recursive_mutex> guard(mutex_);
  cv::namedWindow(win_name_, CV_WINDOW_NORMAL | CV_WINDOW_FREERATIO | CV_GUI_EXPANDED);
  cv::setMouseCallback(win_name_, &ImageViewWindow::onMouseStatic, this);
}

ImageViewWindow::~ImageViewWindow()
{
  cv::destroyWindow(win_name_);
}

void ImageViewWindow::updateImage(const cv::Mat & m)
{
  boost::lock_guard<boost::recursive_mutex> guard(mutex_);
  const bool first_time = source_img_.empty();
  m.copyTo(source_img_);
  full_roi_.width = m.cols;
  full_roi_.height = m.rows;
  if( first_time ) resetView();
  update();
}

void ImageViewWindow::resetView()
{
  boost::lock_guard<boost::recursive_mutex> guard(mutex_);
  if( source_img_.empty() )
    return;
  roi_ = full_roi_;
  cv::resizeWindow(win_name_, source_img_.cols / 4, source_img_.rows / 4);
  update();
}

void ImageViewWindow::pan(int x, int y)
{
  boost::lock_guard<boost::recursive_mutex> guard(mutex_);
  if( source_img_.empty() )
    return;
  cv::Rect proposed_roi = saved_roi_;
  const int dx = x-mouse_ev_orig_.x, dy = y-mouse_ev_orig_.y;

  proposed_roi.x -= dx;
  proposed_roi.y -= dy;

  if( proposed_roi.x < 0 ) {
    proposed_roi.x = 0;
    mouse_ev_orig_.x = x;
    saved_roi_.x = 0;
  }
  if( proposed_roi.y < 0 ) {
    proposed_roi.y = 0;
    mouse_ev_orig_.y = y;
    saved_roi_.y = 0;
  }
  if( proposed_roi.x + proposed_roi.width > full_roi_.width ) {
    proposed_roi.x = full_roi_.width - proposed_roi.width;
    mouse_ev_orig_.x = x;
    saved_roi_.x = full_roi_.width - proposed_roi.width;
  }
  if( proposed_roi.y + proposed_roi.height > full_roi_.height ) {
    proposed_roi.y = full_roi_.height - proposed_roi.height;
    mouse_ev_orig_.y = y;
    saved_roi_.y = full_roi_.height - proposed_roi.height;
  }

  roi_ = proposed_roi & full_roi_;

  update();
}

void ImageViewWindow::zoom(int y)
{
  boost::lock_guard<boost::recursive_mutex> guard(mutex_);

  const int dy = y - mouse_ev_orig_.y;
  // dy<0: mouse up: zoom in
  // dy>0: mouse down: zoom out

  const double zoom = static_cast<double>(full_roi_.width)
      / static_cast<double>(saved_roi_.width);
  const double zoom_f = 1 - static_cast<double>(dy)/1000;

  double proposed_zoom = zoom * zoom_f;

  /*
  std::cout <<"y=" <<y <<", orig=" <<mouse_ev_orig_.y <<", dy=" <<dy
           <<", saved width=" <<saved_roi_.width <<", zoom=" <<zoom
          <<", zoom_f=" <<zoom_f <<", proposed_zoom=" <<proposed_zoom <<std::endl;
  */

  if( proposed_zoom<1 )
    proposed_zoom = 1;

  cv::Rect proposed_roi;
  const cv::Point center(saved_roi_.x + saved_roi_.width/2, saved_roi_.y + saved_roi_.height/2);
  proposed_roi.width = full_roi_.width / proposed_zoom;
  proposed_roi.height = full_roi_.height / proposed_zoom;
  proposed_roi.x = center.x - proposed_roi.width/2;
  proposed_roi.y = center.y - proposed_roi.height/2;

  if( proposed_roi.width>50 && proposed_roi.height>50 && proposed_zoom<5 &&
      full_roi_.contains(proposed_roi.br()) && full_roi_.contains(proposed_roi.tl())) {
    roi_ = proposed_roi;
    update();
  }
}

cv::Point ImageViewWindow::winToImgCoords(int x, int y) const
{
  boost::lock_guard<boost::recursive_mutex> guard(mutex_);
  return cv::Point(x+roi_.x, y+roi_.y);
}

void ImageViewWindow::update()
{
  if( source_img_.empty() )
    return;
  disp_img_ = source_img_(roi_);
}

void ImageViewWindow::onMouseStatic(int event, int x, int y, int flags, void *param)
{
  ImageViewWindow * viewer = (ImageViewWindow *) param;
  viewer->onMouse(event, x, y, flags);
}

void ImageViewWindow::onMouse(int event, int x, int y, int flags)
{
  if( event==CV_EVENT_LBUTTONDOWN || event==CV_EVENT_RBUTTONDOWN ) {
    mouse_ev_orig_ = cv::Point(x,y);
    pan_zoom_mode_ = event==CV_EVENT_LBUTTONDOWN ? PZM_PAN : PZM_ZOOM;
    saved_roi_ = roi_;
  }
  if( event==CV_EVENT_LBUTTONUP || event==CV_EVENT_RBUTTONUP )
    pan_zoom_mode_ = PZM_NONE;
  if( pan_zoom_mode_==PZM_PAN )
    pan(x, y);
  if( pan_zoom_mode_==PZM_ZOOM )
    zoom(y);
}

void ImageViewWindow::printHelp()
{
  printf("Image view help:\n");
  printf("    r : reset the view (pan and zoom)\n");
  printf("    s : save the whole image\n");
  printf("    c : capture: save the image being displayed\n");
  printf("\n");
  printf("    right click and drag up/down : zoom\n");
  printf("    left click and drag          : pan\n");
}

char ImageViewWindow::spin()
{
  {
    boost::lock_guard<boost::recursive_mutex> guard(mutex_);
    if( ! disp_img_.empty() )
      cv::imshow(win_name_, disp_img_);
  }
  return waitKey();
}

char ImageViewWindow::waitKey()
{
  const char key = cv::waitKey(delay_);
  if( key=='r' ) {
    resetView();
  }
  else if( key=='c' || key=='s' ) {
    capture(key=='s');
  }
  else if( key=='h' ) {
    printHelp();
  }
  return key;
}

void ImageViewWindow::capture(bool wholeImage)
{
  static int n = 0;
  const std::string fn = (boost::format("img_view_%03d.png") % n++).str();
  cv::imwrite(fn, wholeImage ? source_img_ : disp_img_);
  std::cout <<(wholeImage ? "Saved whole img" : "Captured img") <<" to " <<fn <<std::endl;
}
