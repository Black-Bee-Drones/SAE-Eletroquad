#ifndef BOUNCING_CPP__YOLO_DETECTOR_HPP_
#define BOUNCING_CPP__YOLO_DETECTOR_HPP_

#include <opencv2/opencv.hpp>
#include <onnxruntime_cxx_api.h>
#include <vector>
#include <string>
#include <memory>

namespace bouncing_cpp
{

struct YoloDetection
{
  cv::Rect bbox;
  float score;
  int class_id;
  double inference_time_ms;

  YoloDetection(const cv::Rect& bbox, float score, int class_id, double inference_time_ms = 0.0)
    : bbox(bbox), score(score), class_id(class_id), inference_time_ms(inference_time_ms)
  {
  }
};

class YoloDetector
{
public:
  explicit YoloDetector(const std::string& model_path);
  ~YoloDetector();

  std::pair<std::vector<YoloDetection>, double> detect(const cv::Mat& image, float conf_threshold = 0.25f,
                                                       float iou_threshold = 0.5f);

  cv::Mat draw_detections(const cv::Mat& image, const std::vector<YoloDetection>& detections);

private:
  void preprocess(const cv::Mat& image, std::vector<float>& input_tensor);
  std::vector<YoloDetection> postprocess(const std::vector<float>& output_tensor, float conf_threshold,
                                         float iou_threshold, const cv::Size& original_size);

  std::unique_ptr<Ort::Session> session_;
  std::unique_ptr<Ort::Env> env_;
  Ort::SessionOptions session_options_;

  std::vector<std::string> input_names_;
  std::vector<std::string> output_names_;
  std::vector<std::vector<int64_t>> input_shapes_;
  std::vector<std::vector<int64_t>> output_shapes_;

  int input_width_;
  int input_height_;
  int num_classes_;

  std::vector<std::string> class_names_;
  std::vector<cv::Scalar> colors_;

  cv::Size original_image_size_;
};

}  // namespace bouncing_cpp

#endif  // BOUNCING_CPP__YOLO_DETECTOR_HPP_