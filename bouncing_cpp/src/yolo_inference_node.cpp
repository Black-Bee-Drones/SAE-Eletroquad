#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <opencv2/opencv.hpp>

#include "bouncing_cpp/yolo_detector.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("yolo_inference_node_cpp");

  // Declare and get parameters
  node->declare_parameter("model_path", "/home/samuel/ros2_ws/src/SAE-Eletroquad/bouncing/bouncing/ai/yolo/best.onnx");
  node->declare_parameter("conf_threshold", 0.5);
  node->declare_parameter("iou_threshold", 0.5);

  std::string model_path = node->get_parameter("model_path").as_string();
  double conf_threshold = node->get_parameter("conf_threshold").as_double();
  double iou_threshold = node->get_parameter("iou_threshold").as_double();

  RCLCPP_INFO(node->get_logger(), "Loading YOLO model from: %s", model_path.c_str());

  std::unique_ptr<bouncing_cpp::YoloDetector> yolo_detector;
  try
  {
    yolo_detector = std::make_unique<bouncing_cpp::YoloDetector>(model_path);
    RCLCPP_INFO(node->get_logger(), "YOLO model loaded successfully");
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to load YOLO model: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  // Publishers for detection results
  auto centers_pub = node->create_publisher<std_msgs::msg::Float32MultiArray>("/yolo_detections/centers", 10);
  auto ids_pub = node->create_publisher<std_msgs::msg::Int32MultiArray>("/yolo_detections/ids", 10);

  // OpenCV webcam capture
  cv::VideoCapture cap(0);  // 0 is usually the default webcam
  if (!cap.isOpened())
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to open webcam");
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Webcam opened successfully. Starting detection loop.");

  while (rclcpp::ok())
  {
    cv::Mat frame;
    cap >> frame;
    if (frame.empty())
    {
      RCLCPP_WARN(node->get_logger(), "Empty frame received from webcam. Exiting loop.");
      break;
    }

    // Run YOLO detection
    auto [detections, inference_time_ms] =
        yolo_detector->detect(frame, static_cast<float>(conf_threshold), static_cast<float>(iou_threshold));

    // Print inference time and FPS
    double fps = 1000.0 / inference_time_ms;
    RCLCPP_INFO(node->get_logger(), "Inference time: %.2f ms (%.1f FPS), Detections: %zu", inference_time_ms, fps,
                detections.size());

    // Prepare and publish detection centers and ids
    std_msgs::msg::Float32MultiArray centers_msg;
    std_msgs::msg::Int32MultiArray ids_msg;
    for (const auto& detection : detections)
    {
      float cx = detection.bbox.x + detection.bbox.width / 2.0f;
      float cy = detection.bbox.y + detection.bbox.height / 2.0f;
      centers_msg.data.push_back(cx);
      centers_msg.data.push_back(cy);
      ids_msg.data.push_back(detection.class_id);
    }
    centers_pub->publish(centers_msg);
    ids_pub->publish(ids_msg);

    // Draw detections on the frame
    cv::Mat result = yolo_detector->draw_detections(frame, detections);
    cv::imshow("Webcam", result);
    if (cv::waitKey(1) == 27)
    {  // Exit on ESC
      RCLCPP_INFO(node->get_logger(), "ESC pressed. Exiting detection loop.");
      break;
    }
    rclcpp::spin_some(node);
  }

  cap.release();
  cv::destroyAllWindows();
  rclcpp::shutdown();
  return 0;
}