#ifndef CONNECTED_COMPONENTS_H
#define CONNECTED_COMPONENTS_H

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

//! img has 0 wherever there is nothing and nonzero where there is something to be clustered.
//! assignments will be filled with -1s for no object and with the object id otherwise.
void cluster(cv::Mat1b img, cv::Mat1i assignments);
  
cv::Mat3b colorAssignments(cv::Mat1i assignments);
  
cv::Vec3b mix(cv::Vec3b color0,
              cv::Vec3b color1,
              cv::Vec3b color2,
              double greyControl);

void flood(const cv::Point2i& seed, int id, cv::Mat1i* ass,
           cv::KeyPoint* center = NULL, int* num_pts = NULL);

#endif // CONNECTED_COMPONENTS_H

