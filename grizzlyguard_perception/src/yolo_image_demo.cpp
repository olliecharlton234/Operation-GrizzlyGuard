// Headless offline check: run the same C++ detector the ROS node uses on image files.
//
//   ros2 run grizzlyguard_perception yolo_image_demo --model <file.onnx> [options] img1.png ...
//     --out DIR        write annotated images to DIR (default: no images written)
//     --conf X         confidence threshold (default 0.25)
//     --target NAME    class to report (default "bear")
//     --all            also print every other class (unfiltered smoke test)
//     --provider cpu|cuda
//
// Exit codes: 0 = ran (bears found or not), 2 = bad arguments, 3 = model failed to load.

#include <chrono>
#include <cstdio>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>

#include "grizzlyguard_perception/yolo_detector.hpp"

namespace gp = grizzlyguard_perception;

int main(int argc, char ** argv)
{
  gp::YoloDetector::Config cfg;
  std::string out_dir, target = "bear";
  bool all = false;
  std::vector<std::string> images;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&]() -> std::string {
        if (i + 1 >= argc) {std::cerr << "missing value for " << a << "\n"; std::exit(2);}
        return argv[++i];
      };
    if (a == "--model") {cfg.model_path = next();}
    else if (a == "--out") {out_dir = next();}
    else if (a == "--conf") {cfg.conf_threshold = std::stof(next());}
    else if (a == "--target") {target = next();}
    else if (a == "--provider") {cfg.execution_provider = next();}
    else if (a == "--all") {all = true;}
    else if (!a.empty() && a[0] == '-') {std::cerr << "unknown option " << a << "\n"; return 2;}
    else {images.push_back(a);}
  }
  if (cfg.model_path.empty() || images.empty()) {
    std::cerr << "usage: yolo_image_demo --model M.onnx [--out DIR] [--conf X] [--target NAME]"
      " [--all] [--provider cpu|cuda] images...\n";
    return 2;
  }

  std::unique_ptr<gp::YoloDetector> det;
  try {
    det = std::make_unique<gp::YoloDetector>(cfg);
  } catch (const std::exception & e) {
    std::cerr << "MODEL LOAD FAILED: " << e.what() << "\n";
    return 3;
  }
  const int target_id = gp::resolveClassId(det->classNames(), target);
  std::cout << "model: " << cfg.model_path << "\n  format: " << gp::toString(det->outputFormat())
            << "\n  input: " << det->inputSize() << "x" << det->inputSize()
            << "\n  provider: " << det->providerInUse()
            << "\n  classes: " << det->classNames().size()
            << "\n  target '" << target << "' -> class id " << target_id << "\n";
  if (target_id < 0) {
    std::cerr << "target class '" << target << "' is not in the model's class list\n";
    return 2;
  }
  if (!out_dir.empty()) {std::filesystem::create_directories(out_dir);}

  for (const auto & path : images) {
    cv::Mat img = cv::imread(path, cv::IMREAD_COLOR);
    std::cout << "\n" << path;
    if (img.empty()) {std::cout << ": could not read image\n"; continue;}
    const auto t0 = std::chrono::steady_clock::now();
    const auto dets = det->detect(img);
    const double ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t0).count();
    std::printf(" (%dx%d, %.1f ms)\n", img.cols, img.rows, ms);
    int n_target = 0;
    std::vector<gp::Detection> shown;
    for (const auto & d : dets) {
      const bool is_target = d.class_id == target_id;
      n_target += is_target;
      if (!is_target && !all) {continue;}
      shown.push_back(d);
      std::printf("   %-3s %-14s %.3f  box x=%.0f y=%.0f w=%.0f h=%.0f\n",
        is_target ? "*" : "", d.label.c_str(), d.score, d.box.x, d.box.y, d.box.width,
        d.box.height);
    }
    std::printf("   => %s\n", n_target ? "BEAR DETECTED" : "no bear detected");
    if (!out_dir.empty()) {
      cv::Mat vis = img.clone();
      gp::drawDetections(vis, shown, target_id);
      cv::imwrite((std::filesystem::path(out_dir) /
        std::filesystem::path(path).filename()).string(), vis);
    }
  }
  return 0;
}