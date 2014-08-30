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
  zoom_ = 1;
  cv::resizeWindow(win_name_, source_img_.cols / 4, source_img_.rows / 4);
  update();
}

void ImageViewWindow::pan(int x, int y)
{
  boost::lock_guard<boost::recursive_mutex> guard(mutex_);
  if( source_img_.empty() )
    return;
  cv::Rect proposed_roi = saved_roi_;
  proposed_roi.x -= (x-mouse_ev_orig_x_) / zoom_;
  proposed_roi.y -= (y-mouse_ev_orig_y_) / zoom_;

  if( full_roi_.contains(proposed_roi.br()) && full_roi_.contains(proposed_roi.tl()))
    roi_ = proposed_roi;
  update();
}

void ImageViewWindow::zoom(int y)
{
  boost::lock_guard<boost::recursive_mutex> guard(mutex_);

  // dz<0: mouse up: zoom in
  // dz>0: mouse down: zoom out
  const double proposed_zoom = saved_zoom_ - ((y - mouse_ev_orig_y_) / 100.);
  if( proposed_zoom>=1 )
    zoom_ = proposed_zoom;
  std::cout <<zoom_ <<std::endl;
  update();
}

void ImageViewWindow::update()
{
  if( ! source_img_.empty() )
    cv::resize(source_img_(roi_), disp_img_, cv::Size(), zoom_, zoom_);
}

void ImageViewWindow::onMouseStatic(int event, int x, int y, int flags, void *param)
{
  ImageViewWindow * viewer = (ImageViewWindow *) param;
  viewer->onMouse(event, x, y, flags);
}

void ImageViewWindow::onMouse(int event, int x, int y, int flags)
{
  if( event==CV_EVENT_LBUTTONDOWN || event==CV_EVENT_RBUTTONDOWN ) {
    mouse_ev_orig_x_ = x;
    mouse_ev_orig_y_ = y;
    pan_zoom_mode_ = event==CV_EVENT_LBUTTONDOWN ? PZM_PAN : PZM_ZOOM;
    saved_roi_ = roi_;
    saved_zoom_ = zoom_;
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
