#include <bag_of_tricks/connected_components.h>
#include <queue>
#include <ros/assert.h>
#include <timer/timer.h>

using namespace std;

cv::Vec3b mix(cv::Vec3b color0,
              cv::Vec3b color1,
              cv::Vec3b color2,
              double greyControl)
{
  int idx = rand() % 3;
  
  float mixRatio0 =
    (idx == 0) ? rand() / (double)RAND_MAX * greyControl : rand() / (double)RAND_MAX;

  float mixRatio1 =
    (idx == 1) ? rand() / (double)RAND_MAX * greyControl : rand() / (double)RAND_MAX;
  
  float mixRatio2 =
    (idx == 2) ? rand() / (double)RAND_MAX * greyControl : rand() / (double)RAND_MAX;
  
  
  float sum = mixRatio0 + mixRatio1 + mixRatio2;
  mixRatio0 /= sum;
  mixRatio1 /= sum;
  mixRatio2 /= sum;

  cv::Vec3b result;
  for(int i = 0; i < 3; ++i)
    result[i] = mixRatio0 * color0[i] + mixRatio1 * color1[i] + mixRatio2 * color2[i];

  return result;
}

cv::Mat3b colorAssignments(cv::Mat1i assignments)
{
  cv::Mat3b vis(assignments.size(), cv::Vec3b(0, 0, 0));

  cv::Vec3b color0(255, 0, 0);
  cv::Vec3b color1(0, 255, 0);
  cv::Vec3b color2(0, 0, 255);
  map<int, cv::Vec3b> colormap;
  
  for(int y = 0; y < assignments.rows; ++y) {
    for(int x = 0; x < assignments.cols; ++x) {
      int idx = assignments(y, x);
      if(idx < 0)
        continue;
      if(colormap.find(idx) == colormap.end())
        colormap[idx] = mix(color0, color1, color2, 0.2);
      vis(y, x) = colormap[idx];
    }
  }

  return vis;
}

void flood(const cv::Point2i& seed, int id, cv::Mat1i* ass, cv::KeyPoint* center, int* num_pts)
{
  double total_x = 0;
  double total_y = 0;
  int local_num_pts = 0;

  std::queue<cv::Point2i> que;
  que.push(seed);
  (*ass)(seed) = -2;
  while(!que.empty()) {
    cv::Point2i pt = que.front();
    que.pop();

    ROS_ASSERT((*ass)(pt) == -2);
    (*ass)(pt) = id;

    ++local_num_pts;
    total_x += pt.x;
    total_y += pt.y;

    cv::Point2i dpt;
    for(dpt.y = -1; dpt.y <= 1; ++dpt.y) {
      for(dpt.x = -1; dpt.x <= 1; ++dpt.x) {
        cv::Point2i pt2 = pt + dpt;
        //cout << pt << " " << dpt << " " << pt2 << endl;
        if(pt2.x < 0 || pt2.x >= ass->cols ||
           pt2.y < 0 || pt2.y >= ass->rows)
          continue;

        if((*ass)(pt2) == -3) {
          que.push(pt2);
          (*ass)(pt2) = -2;
        }
      }
    }
  }

  if(num_pts) {
    *num_pts = local_num_pts;
  }
  if(center) {
    center->pt.x = total_x / (double)local_num_pts;
    center->pt.y = total_y / (double)local_num_pts;
  }
}

//! img has 0 wherever there is nothing and nonzero where there is something to be clustered.
//! assignments will be filled with -1s for no object and with the object id otherwise.
void cluster(cv::Mat1b img, cv::Mat1i assignments)
{
  ScopedTimer st("detectBlobs");
  HighResTimer hrt;

  // -3    : unset
  // -2    : in queue
  // -1    : bg
  // >= 0    : cluster assignment
  hrt.reset("Initial assignment"); hrt.start();
  ROS_ASSERT(img.size() == assignments.size());
  assignments = -3;
  for(int y = 0; y < assignments.rows; ++y)
    for(int x = 0; x < assignments.cols; ++x)
      if(img(y, x) == 0)
        assignments(y, x) = -1;
  hrt.stop(); cout << hrt.reportMilliseconds() << endl;

  hrt.reset("Flooding"); hrt.start();
  int id = 0;
  for(int y = 0; y < assignments.rows; ++y) {
    for(int x = 0; x < assignments.cols; ++x) {
      if(assignments(y, x) == -3) {
        cv::KeyPoint center;
        int num_pts;
        cv::Point2i seed(x, y);
        flood(seed, id, &assignments, &center, &num_pts);
        ++id;
      }
    }
  }
  hrt.stop(); cout << hrt.reportMilliseconds() << endl;
}
