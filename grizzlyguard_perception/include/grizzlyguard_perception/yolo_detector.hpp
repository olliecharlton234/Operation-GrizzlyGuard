// YOLO object detector over ONNX Runtime. ROS-independent so it can be unit-tested
// and run from the command line without a simulator.
//
// Supported Ultralytics ONNX export layouts (do not assume every YOLO export is the same):
//   * Raw:        output [1, 4 + nc, N], rows = cx, cy, w, h, then one score per class.
//                 Needs NMS. Produced by YOLOv8/YOLO11/YOLO26 with nms=False/end2end=False.
//   * EndToEnd:   output [1, N, 6], each row = x1, y1, x2, y2, score, class_id.
//                 NMS-free (already applied in-model). Produced by YOLO26 end2end exports.
// All box coordinates come out in letterboxed-input pixels and are mapped back to the
// original image here.
//
// Why ONNX Runtime and not cv::dnn: OpenCV 4.5/4.6 (Ubuntu 22.04 / ROS 2 Humble) cannot
// import the YOLOv8-family detection head (dfl/Reshape shape assertion in net.forward()).

#ifndef GRIZZLYGUARD_PERCEPTION__YOLO_DETECTOR_HPP_
#define GRIZZLYGUARD_PERCEPTION__YOLO_DETECTOR_HPP_

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace grizzlyguard_perception
{

struct Detection
{
  cv::Rect2f box;      // pixels, (x, y, width, height)
  float score = 0.0f;
  int class_id = -1;
  std::string label;
};

struct LetterboxInfo
{
  float scale = 1.0f;  // original -> letterboxed
  int pad_x = 0;       // left padding in letterboxed pixels
  int pad_y = 0;       // top padding in letterboxed pixels
  int size = 0;        // square input size
};

enum class OutputFormat { Auto, Raw, EndToEnd };

// ---------- Pure helpers (no ONNX Runtime) ----------

// Ultralytics-compatible letterbox: keep aspect ratio, bilinear resize, centre pad with 114.
cv::Mat letterbox(const cv::Mat & bgr, int size, LetterboxInfo & info);

// Letterboxed BGR uint8 image -> RGB float32 NCHW in [0, 1].
void toInputTensor(const cv::Mat & letterboxed_bgr, std::vector<float> & out);

// Decode raw [rows = 4 + nc, cols = anchors] output (row-major, channel-major layout).
// Each anchor gets its arg-max class; class-aware NMS is applied.
std::vector<Detection> decodeRaw(
  const float * data, int rows, int anchors, float conf_threshold, float iou_threshold,
  int max_detections = 300);

// Decode end-to-end [N, 6] output (x1, y1, x2, y2, score, class). No NMS.
std::vector<Detection> decodeEndToEnd(const float * data, int n, float conf_threshold);

// Greedy class-aware NMS. Returns kept indices, highest score first.
std::vector<int> nms(const std::vector<Detection> & dets, float iou_threshold, int max_keep);

// Letterboxed coordinates -> original image pixels, clipped to the image.
void scaleToOriginal(std::vector<Detection> & dets, const LetterboxInfo & info, cv::Size original);

// Parse Ultralytics "names" metadata, e.g. "{0: 'person', 1: 'bicycle', 21: 'bear'}".
std::map<int, std::string> parseClassNames(const std::string & metadata);

// Exact (case-insensitive) name match; returns -1 if absent. "bear" != "teddy bear".
int resolveClassId(const std::map<int, std::string> & names, const std::string & target);

// Pick the layout from the output shape and the export's "end2end" metadata flag.
OutputFormat detectFormat(const std::vector<int64_t> & output_shape, const std::string & end2end_meta);

// Draw boxes in place. highlight_class is drawn red and thick, everything else grey.
void drawDetections(cv::Mat & bgr, const std::vector<Detection> & dets, int highlight_class);

OutputFormat parseOutputFormat(const std::string & s);  // "auto" | "raw" | "end2end"
std::string toString(OutputFormat f);

// ---------- Inference session ----------

class YoloDetector
{
public:
  struct Config
  {
    std::string model_path;
    int input_size = 640;            // must match a static model input; used if dynamic
    float conf_threshold = 0.25f;
    float iou_threshold = 0.45f;
    std::string execution_provider = "cpu";  // "cpu" or "cuda"
    int intra_op_threads = 2;
    OutputFormat output_format = OutputFormat::Auto;
    std::vector<std::string> class_names;   // optional override of model metadata
  };

  // Throws std::runtime_error with a readable reason if the model cannot be used.
  explicit YoloDetector(const Config & config);
  ~YoloDetector();
  YoloDetector(const YoloDetector &) = delete;
  YoloDetector & operator=(const YoloDetector &) = delete;

  // Runs inference on a BGR8 image. Returns all classes; callers filter.
  // Throws std::invalid_argument for an empty or non-BGR8 image.
  std::vector<Detection> detect(const cv::Mat & bgr);

  const std::map<int, std::string> & classNames() const {return names_;}
  int inputSize() const {return input_size_;}
  OutputFormat outputFormat() const {return format_;}
  const std::string & providerInUse() const {return provider_in_use_;}

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  Config config_;
  std::map<int, std::string> names_;
  int input_size_ = 640;
  OutputFormat format_ = OutputFormat::Raw;
  std::string provider_in_use_ = "cpu";
  std::vector<float> input_buffer_;
};

}  // namespace grizzlyguard_perception

#endif  // GRIZZLYGUARD_PERCEPTION__YOLO_DETECTOR_HPP_