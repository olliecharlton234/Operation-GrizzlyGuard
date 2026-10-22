#include "grizzlyguard_perception/yolo_detector.hpp"

#include <onnxruntime_cxx_api.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <numeric>
#include <regex>
#include <sstream>
#include <stdexcept>

#include <opencv2/imgproc.hpp>

namespace grizzlyguard_perception
{

// ---------------------------------------------------------------- pure helpers

cv::Mat letterbox(const cv::Mat & bgr, int size, LetterboxInfo & info)
{
  if (bgr.empty() || size <= 0) {
    throw std::invalid_argument("letterbox: empty image or non-positive size");
  }
  const float r = std::min(
    static_cast<float>(size) / static_cast<float>(bgr.rows),
    static_cast<float>(size) / static_cast<float>(bgr.cols));
  const int new_w = static_cast<int>(std::round(bgr.cols * r));
  const int new_h = static_cast<int>(std::round(bgr.rows * r));
  const float dw = (size - new_w) / 2.0f;
  const float dh = (size - new_h) / 2.0f;
  // Same rounding as Ultralytics LetterBox: round(d - 0.1) on the top/left side.
  const int left = static_cast<int>(std::round(dw - 0.1f));
  const int top = static_cast<int>(std::round(dh - 0.1f));

  cv::Mat out(size, size, CV_8UC3, cv::Scalar(114, 114, 114));
  cv::Mat resized;
  if (new_w != bgr.cols || new_h != bgr.rows) {
    cv::resize(bgr, resized, cv::Size(new_w, new_h), 0, 0, cv::INTER_LINEAR);
  } else {
    resized = bgr;
  }
  resized.copyTo(out(cv::Rect(left, top, new_w, new_h)));

  info.scale = r;
  info.pad_x = left;
  info.pad_y = top;
  info.size = size;
  return out;
}

void toInputTensor(const cv::Mat & img, std::vector<float> & out)
{
  if (img.empty() || img.type() != CV_8UC3 || img.rows != img.cols) {
    throw std::invalid_argument("toInputTensor: expected square CV_8UC3 image");
  }
  const int s = img.rows;
  const size_t plane = static_cast<size_t>(s) * s;
  out.resize(3 * plane);
  float * r = out.data();
  float * g = r + plane;
  float * b = g + plane;
  constexpr float k = 1.0f / 255.0f;
  size_t i = 0;
  for (int y = 0; y < s; ++y) {
    const uint8_t * p = img.ptr<uint8_t>(y);
    for (int x = 0; x < s; ++x, ++i, p += 3) {
      b[i] = p[0] * k;   // OpenCV stores BGR; the model expects RGB planes
      g[i] = p[1] * k;
      r[i] = p[2] * k;
    }
  }
}

static float iou(const cv::Rect2f & a, const cv::Rect2f & b)
{
  const float inter = (a & b).area();
  const float uni = a.area() + b.area() - inter;
  return uni > 0.0f ? inter / uni : 0.0f;
}

std::vector<int> nms(const std::vector<Detection> & dets, float iou_threshold, int max_keep)
{
  std::vector<int> order(dets.size());
  std::iota(order.begin(), order.end(), 0);
  std::stable_sort(order.begin(), order.end(), [&](int a, int b) {
      return dets[a].score > dets[b].score;
    });
  std::vector<int> keep;
  std::vector<bool> removed(dets.size(), false);
  for (size_t i = 0; i < order.size(); ++i) {
    const int a = order[i];
    if (removed[a]) {continue;}
    keep.push_back(a);
    if (static_cast<int>(keep.size()) >= max_keep) {break;}
    for (size_t j = i + 1; j < order.size(); ++j) {
      const int b = order[j];
      if (!removed[b] && dets[b].class_id == dets[a].class_id &&
        iou(dets[a].box, dets[b].box) > iou_threshold)
      {
        removed[b] = true;
      }
    }
  }
  return keep;
}

std::vector<Detection> decodeRaw(
  const float * data, int rows, int anchors, float conf_threshold, float iou_threshold,
  int max_detections)
{
  if (rows < 5 || anchors <= 0) {
    throw std::invalid_argument("decodeRaw: expected [4 + num_classes, anchors] output");
  }
  std::vector<Detection> candidates;
  for (int j = 0; j < anchors; ++j) {
    int best = -1;
    float best_score = -1.0f;
    for (int c = 4; c < rows; ++c) {
      const float s = data[static_cast<size_t>(c) * anchors + j];
      if (s > best_score) {best_score = s; best = c - 4;}
    }
    if (best_score < conf_threshold) {continue;}
    const float cx = data[j];
    const float cy = data[static_cast<size_t>(anchors) + j];
    const float w = data[2 * static_cast<size_t>(anchors) + j];
    const float h = data[3 * static_cast<size_t>(anchors) + j];
    Detection d;
    d.box = cv::Rect2f(cx - w / 2.0f, cy - h / 2.0f, w, h);
    d.score = best_score;
    d.class_id = best;
    candidates.push_back(d);
  }
  std::vector<Detection> out;
  for (int k : nms(candidates, iou_threshold, max_detections)) {
    out.push_back(candidates[k]);
  }
  return out;
}

std::vector<Detection> decodeEndToEnd(const float * data, int n, float conf_threshold)
{
  std::vector<Detection> out;
  for (int i = 0; i < n; ++i) {
    const float * r = data + static_cast<size_t>(i) * 6;
    if (r[4] < conf_threshold) {continue;}
    Detection d;
    d.box = cv::Rect2f(r[0], r[1], r[2] - r[0], r[3] - r[1]);
    d.score = r[4];
    d.class_id = static_cast<int>(std::lround(r[5]));
    out.push_back(d);
  }
  std::stable_sort(out.begin(), out.end(), [](const Detection & a, const Detection & b) {
      return a.score > b.score;
    });
  return out;
}

void scaleToOriginal(std::vector<Detection> & dets, const LetterboxInfo & info, cv::Size original)
{
  const float W = static_cast<float>(original.width);
  const float H = static_cast<float>(original.height);
  std::vector<Detection> kept;
  for (auto d : dets) {
    float x1 = (d.box.x - info.pad_x) / info.scale;
    float y1 = (d.box.y - info.pad_y) / info.scale;
    float x2 = (d.box.x + d.box.width - info.pad_x) / info.scale;
    float y2 = (d.box.y + d.box.height - info.pad_y) / info.scale;
    x1 = std::clamp(x1, 0.0f, W); x2 = std::clamp(x2, 0.0f, W);
    y1 = std::clamp(y1, 0.0f, H); y2 = std::clamp(y2, 0.0f, H);
    if (x2 <= x1 || y2 <= y1) {continue;}
    d.box = cv::Rect2f(x1, y1, x2 - x1, y2 - y1);
    kept.push_back(d);
  }
  dets.swap(kept);
}

std::map<int, std::string> parseClassNames(const std::string & metadata)
{
  std::map<int, std::string> names;
  static const std::regex re(R"((\d+)\s*:\s*(['"])(.*?)\2)");
  for (auto it = std::sregex_iterator(metadata.begin(), metadata.end(), re);
    it != std::sregex_iterator(); ++it)
  {
    names[std::stoi((*it)[1].str())] = (*it)[3].str();
  }
  return names;
}

static std::string lower(std::string s)
{
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {return std::tolower(c);});
  return s;
}

int resolveClassId(const std::map<int, std::string> & names, const std::string & target)
{
  const std::string t = lower(target);
  for (const auto & kv : names) {
    if (lower(kv.second) == t) {return kv.first;}
  }
  return -1;
}

OutputFormat detectFormat(const std::vector<int64_t> & shape, const std::string & end2end_meta)
{
  const std::string m = lower(end2end_meta);
  if (m == "true") {return OutputFormat::EndToEnd;}
  if (m == "false") {return OutputFormat::Raw;}
  if (shape.size() == 3 && shape[2] == 6 && shape[1] != 6) {return OutputFormat::EndToEnd;}
  return OutputFormat::Raw;
}

OutputFormat parseOutputFormat(const std::string & s)
{
  const std::string v = lower(s);
  if (v == "auto" || v.empty()) {return OutputFormat::Auto;}
  if (v == "raw") {return OutputFormat::Raw;}
  if (v == "end2end" || v == "endtoend") {return OutputFormat::EndToEnd;}
  throw std::invalid_argument("output_format must be 'auto', 'raw' or 'end2end', got '" + s + "'");
}

std::string toString(OutputFormat f)
{
  switch (f) {
    case OutputFormat::Raw: return "raw [1, 4+nc, N] + NMS";
    case OutputFormat::EndToEnd: return "end2end [1, N, 6], NMS-free";
    default: return "auto";
  }
}

void drawDetections(cv::Mat & bgr, const std::vector<Detection> & dets, int highlight_class)
{
  for (const auto & d : dets) {
    const bool hi = d.class_id == highlight_class;
    const cv::Scalar col = hi ? cv::Scalar(0, 0, 255) : cv::Scalar(160, 160, 160);
    cv::rectangle(bgr, d.box, col, hi ? 2 : 1);
    char text[96];
    std::snprintf(text, sizeof(text), "%s %.2f", d.label.c_str(), d.score);
    int base = 0;
    const cv::Size ts = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &base);
    const cv::Point org(static_cast<int>(d.box.x),
      std::max(ts.height + 2, static_cast<int>(d.box.y) - 3));
    cv::rectangle(bgr, cv::Rect(org.x, org.y - ts.height - 2, ts.width + 2, ts.height + base + 2),
      col, cv::FILLED);
    cv::putText(bgr, text, cv::Point(org.x + 1, org.y), cv::FONT_HERSHEY_SIMPLEX, 0.5,
      cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
  }
}

// ---------------------------------------------------------------- session

struct YoloDetector::Impl
{
  Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "grizzlyguard_yolo"};
  std::unique_ptr<Ort::Session> session;
  std::string input_name;
  std::string output_name;
  Ort::MemoryInfo mem = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
};

static std::string shapeStr(const std::vector<int64_t> & s)
{
  std::ostringstream o;
  o << "[";
  for (size_t i = 0; i < s.size(); ++i) {o << (i ? ", " : "") << s[i];}
  o << "]";
  return o.str();
}

YoloDetector::YoloDetector(const Config & config)
: impl_(std::make_unique<Impl>()), config_(config)
{
  if (config.model_path.empty()) {
    throw std::runtime_error("model_path is empty");
  }
  Ort::SessionOptions opts;
  opts.SetIntraOpNumThreads(std::max(1, config.intra_op_threads));
  opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

  const std::string provider = lower(config.execution_provider);
  if (provider == "cuda") {
    try {
      OrtCUDAProviderOptions cuda{};
      opts.AppendExecutionProvider_CUDA(cuda);
      provider_in_use_ = "cuda";
    } catch (const Ort::Exception & e) {
      provider_in_use_ = std::string("cpu (CUDA requested but unavailable: ") + e.what() + ")";
    }
  } else if (provider != "cpu") {
    throw std::runtime_error("execution_provider must be 'cpu' or 'cuda', got '" +
                             config.execution_provider + "'");
  }

  try {
    impl_->session = std::make_unique<Ort::Session>(impl_->env, config.model_path.c_str(), opts);
  } catch (const Ort::Exception & e) {
    throw std::runtime_error("failed to load ONNX model '" + config.model_path + "': " + e.what());
  }
  Ort::Session & s = *impl_->session;
  if (s.GetInputCount() != 1 || s.GetOutputCount() < 1) {
    throw std::runtime_error("expected a single-input YOLO detection model");
  }
  Ort::AllocatorWithDefaultOptions alloc;
  impl_->input_name = s.GetInputNameAllocated(0, alloc).get();
  impl_->output_name = s.GetOutputNameAllocated(0, alloc).get();

  // Input size: static models dictate it; a mismatching parameter is a configuration error.
  const auto in_shape = s.GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
  if (in_shape.size() != 4 || (in_shape[1] != 3 && in_shape[1] > 0)) {
    throw std::runtime_error("unexpected model input shape " + shapeStr(in_shape) +
                             ", expected [1, 3, H, W]");
  }
  if (in_shape[2] > 0 && in_shape[3] > 0) {
    if (in_shape[2] != in_shape[3]) {
      throw std::runtime_error("only square model inputs are supported, got " + shapeStr(in_shape));
    }
    if (config.input_size > 0 && config.input_size != in_shape[2]) {
      throw std::runtime_error(
              "inference_size=" + std::to_string(config.input_size) +
              " but the model was exported with a fixed input of " + std::to_string(in_shape[2]) +
              ". Re-export at the size you want, or set inference_size to match.");
    }
    input_size_ = static_cast<int>(in_shape[2]);
  } else {
    input_size_ = config.input_size > 0 ? config.input_size : 640;
  }

  // Metadata written by Ultralytics export.
  Ort::ModelMetadata md = s.GetModelMetadata();
  auto meta = [&](const char * key) -> std::string {
      auto v = md.LookupCustomMetadataMapAllocated(key, alloc);
      return v ? std::string(v.get()) : std::string();
    };

  if (!config.class_names.empty()) {
    for (size_t i = 0; i < config.class_names.size(); ++i) {
      names_[static_cast<int>(i)] = config.class_names[i];
    }
  } else {
    names_ = parseClassNames(meta("names"));
  }

  const auto out_shape = s.GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
  format_ = config.output_format == OutputFormat::Auto ?
    detectFormat(out_shape, meta("end2end")) : config.output_format;

  // Catch a class list that does not match the model, which would silently mislabel.
  if (format_ == OutputFormat::Raw && out_shape.size() == 3 && out_shape[1] > 0 &&
    !names_.empty())
  {
    const int64_t nc = out_shape[1] - 4;
    if (static_cast<int64_t>(names_.size()) != nc) {
      throw std::runtime_error(
              "model outputs " + std::to_string(nc) + " classes but " +
              std::to_string(names_.size()) + " class names are configured");
    }
  }
}

YoloDetector::~YoloDetector() = default;

std::vector<Detection> YoloDetector::detect(const cv::Mat & bgr)
{
  if (bgr.empty()) {throw std::invalid_argument("detect: empty image");}
  if (bgr.type() != CV_8UC3) {throw std::invalid_argument("detect: expected BGR8 (CV_8UC3)");}

  LetterboxInfo info;
  const cv::Mat lb = letterbox(bgr, input_size_, info);
  toInputTensor(lb, input_buffer_);

  const std::array<int64_t, 4> shape{1, 3, input_size_, input_size_};
  Ort::Value input = Ort::Value::CreateTensor<float>(
    impl_->mem, input_buffer_.data(), input_buffer_.size(), shape.data(), shape.size());
  const char * in_names[] = {impl_->input_name.c_str()};
  const char * out_names[] = {impl_->output_name.c_str()};
  auto outputs = impl_->session->Run(Ort::RunOptions{nullptr}, in_names, &input, 1, out_names, 1);

  const auto out_shape = outputs[0].GetTensorTypeAndShapeInfo().GetShape();
  const float * data = outputs[0].GetTensorData<float>();
  std::vector<Detection> dets;
  if (format_ == OutputFormat::EndToEnd) {
    if (out_shape.size() != 3 || out_shape[2] < 6) {
      throw std::runtime_error("end2end decoding expects [1, N, 6], model gave " +
                               shapeStr(out_shape));
    }
    dets = decodeEndToEnd(data, static_cast<int>(out_shape[1]), config_.conf_threshold);
  } else {
    if (out_shape.size() != 3 || out_shape[1] < 5) {
      throw std::runtime_error("raw decoding expects [1, 4+nc, N], model gave " +
                               shapeStr(out_shape));
    }
    dets = decodeRaw(data, static_cast<int>(out_shape[1]), static_cast<int>(out_shape[2]),
        config_.conf_threshold, config_.iou_threshold);
  }

  scaleToOriginal(dets, info, bgr.size());
  for (auto & d : dets) {
    auto it = names_.find(d.class_id);
    d.label = it != names_.end() ? it->second : "class_" + std::to_string(d.class_id);
  }
  return dets;
}

}  // namespace grizzlyguard_perception