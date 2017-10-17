#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <my_vio/feature_tracker.hpp>

using namespace vio;
using namespace std;

class FeatureTrackerWrapper {
 public:
  FeatureTrackerWrapper() {
    FeatureMatcherOptions matcher_options;
    FeatureTrackerOptions tracker_options;
    // Use default FAST+FREAK.
    std::unique_ptr<FeatureMatcher> matcher =
        FeatureMatcher::CreateFeatureMatcher(matcher_options);
    tracker_ = FeatureTracker::CreateFeatureTracker(tracker_options,
                                                    std::move(matcher));
  }

  FeatureTracker *tracker() { return tracker_; }

 private:
  FeatureTracker *tracker_;
};

class ImageConverter {
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;

  FeatureTrackerWrapper tracker_wrapper_;
  std::unique_ptr<vio::ImageFrame> current_frame_;

 public:
  ImageConverter() : it_(nh_) {
    // Subscrive to input video feed and publish output video feed
    image_sub_ =
        it_.subscribe("/image_raw", 1, &ImageConverter::imageCb, this);
    cv::namedWindow("result", cv::WINDOW_AUTOSIZE);
  }

  void imageCb(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    std::unique_ptr<vio::ImageFrame> new_frame(
        new vio::ImageFrame(cv_ptr->image));
    // Init first frame.
    if (current_frame_ == nullptr) {
      current_frame_ = std::move(new_frame);
    } else {
      std::vector<cv::DMatch> matches;
      tracker_wrapper_.tracker()->TrackFrame(*current_frame_, *new_frame,
                                             matches);
      std::cout << "Feature number in new frame "
                << new_frame->keypoints().size() << std::endl;
      std::cout << "Found match " << matches.size() << std::endl;

      // Draw match.
      cv::Mat output_img = current_frame_->GetImage().clone();
      int thickness = 2;
      for (int i = 0; i < matches.size(); ++i) {
        line(output_img, new_frame->keypoints()[matches[i].trainIdx].pt,
             current_frame_->keypoints()[matches[i].queryIdx].pt,
             cv::Scalar(255, 0, 0), thickness);
      }
      current_frame_ = std::move(new_frame);
      cv::imshow("result", output_img);
      cv::waitKey(3);
    }
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;

  ros::spin();
  return 0;
}
