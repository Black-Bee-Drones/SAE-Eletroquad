#include "bouncing_cpp/yolo_detector.hpp"
#include <chrono>
#include <algorithm>
#include <random>
#include <thread>
#include <iostream>

namespace bouncing_cpp
{

YoloDetector::YoloDetector(const std::string& model_path) : num_classes_(8)
{
  // Initialize class names (same as Python version)
  class_names_ = { "0", "1", "2", "3", "4", "5", "6", "7" };

  // Generate random colors for each class
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dis(0, 255);

  for (size_t i = 0; i < class_names_.size(); ++i)
  {
    colors_.push_back(cv::Scalar(dis(gen), dis(gen), dis(gen)));
  }

  // Initialize ONNX Runtime
  env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "YoloDetector");

  // Configure session options for maximum optimization
  session_options_.SetIntraOpNumThreads(4);  // Raspberry Pi 4 has 4 cores
  // session_options_.SetInterOpNumThreads(1);   // Sequential execution for single model
  // session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
  session_options_.SetExecutionMode(ExecutionMode::ORT_SEQUENTIAL);
  session_options_.EnableMemPattern();
  session_options_.EnableCpuMemArena();

  // Add ARM-specific optimizations
  session_options_.AddConfigEntry("session.disable_prepacking", "0");
  session_options_.AddConfigEntry("session.use_env_allocators", "1");
  session_options_.AddConfigEntry("session.enable_cpu_mem_arena", "1");

  // Try XNNPACK provider first (ARM-optimized)
  try
  {
    session_options_.AppendExecutionProvider("XNNPACK", {});
    std::cout << "Using XNNPACK Execution Provider (ARM-optimized)" << std::endl;
  }
  catch (const std::exception& e)
  {
    std::cout << "XNNPACK Execution Provider not available: " << e.what() << std::endl;

    // Try to use OpenVINO Execution Provider if available
    try
    {
      // Try to append OpenVINO execution provider
      session_options_.AppendExecutionProvider("OpenVINO", { { "device_type", "CPU_FP32" } });
      std::cout << "Using OpenVINO Execution Provider (CPU_FP32)" << std::endl;
    }
    catch (const std::exception& e)
    {
      std::cout << "OpenVINO Execution Provider not available, falling back to default CPU provider." << std::endl;
      std::cout << "Reason: " << e.what() << std::endl;
    }
  }

  // Create session
  session_ = std::make_unique<Ort::Session>(*env_, model_path.c_str(), session_options_);

  // Get input/output information
  Ort::AllocatorWithDefaultOptions allocator;

  // Input info
  size_t num_input_nodes = session_->GetInputCount();
  for (size_t i = 0; i < num_input_nodes; i++)
  {
    auto input_name = session_->GetInputNameAllocated(i, allocator);
    input_names_.push_back(std::string(input_name.get()));

    Ort::TypeInfo input_type_info = session_->GetInputTypeInfo(i);
    auto input_tensor_info = input_type_info.GetTensorTypeAndShapeInfo();
    auto input_dims = input_tensor_info.GetShape();
    input_shapes_.push_back(input_dims);

    // Set input dimensions from the model (assuming NCHW format: [batch, channels, height, width])
    if (input_dims.size() >= 4)
    {
      input_height_ = static_cast<int>(input_dims[2]);
      input_width_ = static_cast<int>(input_dims[3]);
      std::cout << "Detected model input size: " << input_width_ << "x" << input_height_ << std::endl;
    }
  }

  // Output info
  size_t num_output_nodes = session_->GetOutputCount();
  for (size_t i = 0; i < num_output_nodes; i++)
  {
    auto output_name = session_->GetOutputNameAllocated(i, allocator);
    output_names_.push_back(std::string(output_name.get()));

    Ort::TypeInfo output_type_info = session_->GetOutputTypeInfo(i);
    auto output_tensor_info = output_type_info.GetTensorTypeAndShapeInfo();
    auto output_dims = output_tensor_info.GetShape();
    output_shapes_.push_back(output_dims);
  }
}

YoloDetector::~YoloDetector() = default;

std::pair<std::vector<YoloDetection>, double> YoloDetector::detect(const cv::Mat& image, float conf_threshold,
                                                                   float iou_threshold)
{
  auto start_time = std::chrono::high_resolution_clock::now();

  original_image_size_ = image.size();

  // Preprocess
  std::vector<float> input_tensor;
  preprocess(image, input_tensor);

  // Create input tensor
  std::vector<int64_t> input_shape = { 1, 3, input_height_, input_width_ };
  Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
  Ort::Value input_tensor_ort = Ort::Value::CreateTensor<float>(memory_info, input_tensor.data(), input_tensor.size(),
                                                                input_shape.data(), input_shape.size());

  // Prepare input/output name arrays
  std::vector<const char*> input_names_cstr;
  for (const auto& name : input_names_)
    input_names_cstr.push_back(name.c_str());
  std::vector<const char*> output_names_cstr;
  for (const auto& name : output_names_)
    output_names_cstr.push_back(name.c_str());

  // Run inference
  auto output_tensors = session_->Run(Ort::RunOptions{ nullptr }, input_names_cstr.data(), &input_tensor_ort, 1,
                                      output_names_cstr.data(), 1);

  // Get output data
  float* output_data = output_tensors[0].GetTensorMutableData<float>();
  auto output_shape = output_tensors[0].GetTensorTypeAndShapeInfo().GetShape();

  size_t output_size = 1;
  for (auto dim : output_shape)
  {
    output_size *= dim;
  }

  std::vector<float> output_tensor(output_data, output_data + output_size);

  // Postprocess
  auto detections = postprocess(output_tensor, conf_threshold, iou_threshold, original_image_size_);

  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
  double inference_time_ms = duration.count() / 1000.0;

  // Add inference time to all detections
  for (auto& detection : detections)
  {
    detection.inference_time_ms = inference_time_ms;
  }

  return { detections, inference_time_ms };
}

void YoloDetector::preprocess(const cv::Mat& image, std::vector<float>& input_tensor)
{
  cv::Mat resized_image;
  cv::resize(image, resized_image, cv::Size(input_width_, input_height_));

  // Convert BGR to RGB and normalize
  cv::Mat rgb_image;
  cv::cvtColor(resized_image, rgb_image, cv::COLOR_BGR2RGB);

  rgb_image.convertTo(rgb_image, CV_32F, 1.0 / 255.0);

  // HWC to CHW format
  input_tensor.resize(3 * input_height_ * input_width_);

  std::vector<cv::Mat> channels(3);
  cv::split(rgb_image, channels);

  for (int c = 0; c < 3; ++c)
  {
    std::memcpy(input_tensor.data() + c * input_height_ * input_width_, channels[c].data,
                input_height_ * input_width_ * sizeof(float));
  }
}

std::vector<YoloDetection> YoloDetector::postprocess(const std::vector<float>& output_tensor, float conf_threshold,
                                                     float iou_threshold, const cv::Size& original_size)
{
  std::vector<YoloDetection> detections;
  std::vector<cv::Rect> boxes;
  std::vector<float> scores;
  std::vector<int> class_ids;

  // Calculate scaling factors
  float x_factor = static_cast<float>(original_size.width) / input_width_;
  float y_factor = static_cast<float>(original_size.height) / input_height_;

  // YOLO output format is typically [batch, 4 + num_classes, num_detections]
  // We need to transpose it to [num_detections, 4 + num_classes] like Python does
  int num_classes = num_classes_;                          // 8 classes
  int output_dim = 4 + num_classes;                        // 4 bbox coords + 8 class scores = 12
  int num_detections = output_tensor.size() / output_dim;  // Total elements / 12

  std::cout << "[DEBUG] Output tensor size: " << output_tensor.size() << ", num_detections: " << num_detections
            << ", output_dim: " << output_dim << std::endl;

  for (int i = 0; i < num_detections; ++i)
  {
    // Access transposed data: for detection i, element j is at position j * num_detections + i
    // Get bounding box (center format)
    float x_center = output_tensor[0 * num_detections + i];
    float y_center = output_tensor[1 * num_detections + i];
    float width = output_tensor[2 * num_detections + i];
    float height = output_tensor[3 * num_detections + i];

    // Get class scores (starting from index 4)
    float max_score = 0.0f;
    int max_class_id = 0;

    for (int j = 0; j < num_classes; ++j)
    {
      float score = output_tensor[(4 + j) * num_detections + i];
      if (score > max_score)
      {
        max_score = score;
        max_class_id = j;
      }
    }

    if (max_score >= conf_threshold)
    {
      // Convert to corner format and scale to original image size
      int x1 = static_cast<int>((x_center - width / 2.0f) * x_factor);
      int y1 = static_cast<int>((y_center - height / 2.0f) * y_factor);
      int w = static_cast<int>(width * x_factor);
      int h = static_cast<int>(height * y_factor);

      boxes.push_back(cv::Rect(x1, y1, w, h));
      scores.push_back(max_score);
      class_ids.push_back(max_class_id);

      std::cout << "[DEBUG] Detection " << i << ": class_id=" << max_class_id << ", score=" << max_score << ", bbox=("
                << x1 << "," << y1 << "," << w << "," << h << ")" << std::endl;
    }
  }

  // Apply Non-Maximum Suppression
  std::vector<int> indices;
  cv::dnn::NMSBoxes(boxes, scores, conf_threshold, iou_threshold, indices);

  for (int idx : indices)
  {
    detections.emplace_back(boxes[idx], scores[idx], class_ids[idx]);
  }

  return detections;
}

cv::Mat YoloDetector::draw_detections(const cv::Mat& image, const std::vector<YoloDetection>& detections)
{
  cv::Mat result = image.clone();

  for (const auto& detection : detections)
  {
    const auto& bbox = detection.bbox;
    int safe_class_id = detection.class_id;
    // Class ID bounds check
    if (safe_class_id < 0 || safe_class_id >= static_cast<int>(class_names_.size()))
    {
      std::cerr << "[WARN] Invalid class_id: " << safe_class_id << ". Skipping this detection.\n";
      continue;
    }
    // Bounding box validity check
    if (bbox.x < 0 || bbox.y < 0 || bbox.width <= 0 || bbox.height <= 0 || bbox.x + bbox.width > result.cols ||
        bbox.y + bbox.height > result.rows)
    {
      std::cerr << "[WARN] Invalid bbox: (" << bbox.x << "," << bbox.y << "," << bbox.width << "," << bbox.height
                << ") Skipping this detection.\n";
      continue;
    }

    const auto& color = colors_[safe_class_id % colors_.size()];
    std::string label =
        class_names_[safe_class_id] + ": " + std::to_string(static_cast<int>(detection.score * 100)) + "%";

    int baseline;
    cv::Size label_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);

    int label_x = bbox.x;
    int label_y = bbox.y - 10;
    if (label_y < label_size.height)
    {
      label_y = bbox.y + label_size.height + 10;
    }

    // Draw bounding box
    cv::rectangle(result, bbox, color, 2);
    // Draw label background
    cv::rectangle(result, cv::Point(label_x, label_y - label_size.height),
                  cv::Point(label_x + label_size.width, label_y + baseline), color, cv::FILLED);
    // Draw label text
    cv::putText(result, label, cv::Point(label_x, label_y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
  }

  return result;
}

}  // namespace bouncing_cpp