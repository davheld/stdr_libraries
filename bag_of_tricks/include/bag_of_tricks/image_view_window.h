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

#ifndef __BAG_OF_TRICKS__IMAGE_VIEW_WINDOW_H__
#define __BAG_OF_TRICKS__IMAGE_VIEW_WINDOW_H__


#include <boost/thread.hpp>
#include <opencv2/core/core.hpp>


/** \brief An image viewer that supports zooming and panning.
  *
  * Based on cv/highgui, with mouse event callbacks to handle zooming and
  * panning.
  *
  * ISSUES:
  * - automatic resizing does not work well, as highgui does not provide
  *   the size of the window.
  * - waitKey must be called from the same thread that created the ImageViewWindow.
  */
class ImageViewWindow
{
public:
  ImageViewWindow(const std::string& name, int delay=0);
  ~ImageViewWindow();

  inline const std::string& getWinName() const { return win_name_; }

  /// Updates the source image
  void updateImage(const cv::Mat &);

  /// Updates the GUI and calls waitKey. Must be called periodically.
  /// @return the result of waitKey
  char spin();

  /// computes the coordinates in the full image's frame, given coordinates
  /// in the displayed image's frame.
  cv::Point winToImgCoords(int x, int y) const;

protected:
  virtual void onMouse(int event, int x, int y, int flags);

  /** Wait for a key press and returns the key pressed if any.
    *
    * Keys 'r', 'c', 's' and 'h' are used for:
    *   - 'r': reset the view
    *   - 'c': capture the displayed image
    *   - 's': save the whole image
    *   - 'h': print the help message
    *
    * @param delay wait for an event for that many milliseconds.
    * @returns the key that was pressed.
    */
  virtual char waitKey();

  int delay_; ///< delay used in waitKey

  virtual void printHelp();

  /// Resets the view parameters: center point, zoom, etc.
  void resetView();

  void capture(bool wholeImage = true);

private:
  static void onMouseStatic(int event, int x, int y, int flags, void * param);

  /// Moves the center point (pan)
  void pan(int x, int y);

  /// Set the center point (pan)
  void center(int x, int y, bool update=true);

  /// Changes the zoom factor
  void zoom(int y);

  void update();

  const std::string win_name_;
  cv::Rect roi_, saved_roi_, full_roi_;

  enum PanZoomMode { PZM_NONE, PZM_PAN, PZM_ZOOM };
  PanZoomMode pan_zoom_mode_;
  cv::Point mouse_ev_orig_;

  cv::Mat source_img_, disp_img_;
  mutable boost::recursive_mutex mutex_;
};



#endif // __BAG_OF_TRICKS__IMAGE_VIEW_WINDOW_H__
