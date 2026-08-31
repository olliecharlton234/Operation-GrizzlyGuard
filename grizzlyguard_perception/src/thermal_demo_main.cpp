// Copyright 2026 Operation GrizzlyGuard (Group 15)
//
// Standalone test/demo tool for the thermal-perception colour-segmentation
// pipeline.
//
// This deliberately does NOT depend on ROS, cv_bridge, or the simulator: it
// links only against the pure `thermal_detector` library and plain OpenCV,
// so the perception logic can be demonstrated (or debugged) even before the
// simulated camera topic is available.
//
// It generates three synthetic forest-like test frames, runs each one
// through exactly the same detection functions used by the live ROS node
// (grizzlyguard_perception/thermal_detector.hpp), and saves the
// intermediate and final images so they can be inspected or included in a
// report/demo:
//
//   1. valid_warm_object   -- a large orange/red blob (a placeholder bear
//                              heat signature) on a dark green background.
//                              Expected: DETECTED.
//   2. no_warm_object      -- plain forest background only.
//                              Expected: NOT detected.
//   3. small_noise_rejected -- a small warm-coloured dot, below
//                              min_contour_area.
//                              Expected: NOT detected (rejected as noise).
//
// Usage:
//   thermal_demo [output_directory] [--show]
//
//   output_directory  Where to save PNGs (default: "thermal_demo_output" in
//                      the current working directory).
//   --show            Also try to open the images in OpenCV HighGUI windows
//                      (requires a display; safely skipped if unavailable).

#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "grizzlyguard_perception/thermal_detector.hpp"

namespace
{

using grizzlyguard_perception::DetectionResult;
using grizzlyguard_perception::ThermalDetectorParams;

struct DemoCase
{
  std::string name;
  cv::Mat image;
  bool expect_detected;
};

/// A dark-green background with a small amount of deterministic texture, so
/// it is not perfectly flat (closer to what a rendered forest floor might
/// look like) while still being fully reproducible between runs.
cv::Mat makeForestBackground(int width, int height)
{
  cv::Mat image(height, width, CV_8UC3, cv::Scalar(25, 70, 25));  // BGR: dark green

  cv::Mat noise(height, width, CV_8UC3);
  cv::RNG rng(12345);  // Fixed seed: deterministic, reproducible test frames.
  rng.fill(noise, cv::RNG::UNIFORM, cv::Scalar(0, 0, 0), cv::Scalar(14, 22, 14));
  image += noise;

  return image;
}

DemoCase makeValidWarmObjectCase()
{
  cv::Mat image = makeForestBackground(640, 480);
  // Orange/red filled ellipse: a placeholder "bear" heat signature.
  cv::ellipse(
    image, cv::Point(320, 260), cv::Size(70, 45), 0.0, 0.0, 360.0,
    cv::Scalar(25, 80, 210), cv::FILLED);
  return {"valid_warm_object", image, true};
}

DemoCase makeNoWarmObjectCase()
{
  return {"no_warm_object", makeForestBackground(640, 480), false};
}

DemoCase makeSmallNoiseCase()
{
  cv::Mat image = makeForestBackground(640, 480);
  // A small warm-coloured dot -- area well below the default
  // min_contour_area, so it should be rejected as noise.
  cv::circle(image, cv::Point(150, 120), 4, cv::Scalar(20, 90, 200), cv::FILLED);
  return {"small_noise_rejected", image, false};
}

bool hasFlag(int argc, char ** argv, const std::string & flag)
{
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == flag) {
      return true;
    }
  }
  return false;
}

std::string firstPositionalArg(int argc, char ** argv)
{
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if (arg.rfind("--", 0) != 0) {
      return arg;
    }
  }
  return "";
}

void trySave(const cv::Mat & image, const std::string & path)
{
  if (!cv::imwrite(path, image)) {
    std::cerr << "  [WARN] Failed to save " << path << std::endl;
  }
}

void tryShow(const std::string & window_name, const cv::Mat & image, bool show_enabled)
{
  if (!show_enabled) {
    return;
  }
  try {
    cv::imshow(window_name, image);
  } catch (const cv::Exception & e) {
    std::cerr << "  [WARN] Could not open a display window (" << e.what()
              << "). Continuing in headless (save-only) mode." << std::endl;
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  const bool show_windows = hasFlag(argc, argv, "--show");
  std::string output_dir = firstPositionalArg(argc, argv);
  if (output_dir.empty()) {
    output_dir = "thermal_demo_output";
  }

  const std::string mkdir_command = "mkdir -p \"" + output_dir + "\"";
  if (std::system(mkdir_command.c_str()) != 0) {
    std::cerr << "Failed to create output directory: " << output_dir << std::endl;
    return 1;
  }

  std::cout << "=====================================================" << std::endl;
  std::cout << " GrizzlyGuard thermal perception -- standalone demo" << std::endl;
  std::cout << " (colour-segmentation placeholder, NOT a real thermal sensor)" << std::endl;
  std::cout << " Saving outputs to: " << output_dir << std::endl;
  std::cout << "=====================================================" << std::endl;

  const std::vector<DemoCase> cases = {
    makeValidWarmObjectCase(),
    makeNoWarmObjectCase(),
    makeSmallNoiseCase(),
  };

  const ThermalDetectorParams params;  // Defaults, matching config/thermal_perception_params.yaml.
  bool all_passed = true;

  for (const DemoCase & test_case : cases) {
    std::cout << "\n--- Test case: " << test_case.name << " ---" << std::endl;

    const cv::Mat warm_mask = grizzlyguard_perception::computeWarmMask(test_case.image, params);
    const DetectionResult result = grizzlyguard_perception::detectWarmRegion(warm_mask, params);
    const cv::Mat overlay = grizzlyguard_perception::drawDetectionOverlay(test_case.image, result);
    const cv::Mat pseudo_thermal = grizzlyguard_perception::computePseudoThermal(
      test_case.image, result);

    if (result.detected) {
      std::cout << "  WARM OBJECT DETECTED"
                << "  centre=(" << static_cast<int>(result.centre.x) << ", "
                << static_cast<int>(result.centre.y) << ")"
                << "  bbox=[" << result.bounding_box.x << "," << result.bounding_box.y << ","
                << result.bounding_box.width << "x" << result.bounding_box.height << "]"
                << "  area=" << static_cast<int>(result.area) << "px^2"
                << "  confidence=" << std::fixed << std::setprecision(2) << result.confidence
                << std::endl;
    } else {
      std::cout << "  NO WARM OBJECT DETECTED" << std::endl;
    }

    const bool passed = (result.detected == test_case.expect_detected);
    std::cout << "  Expected detected=" << std::boolalpha << test_case.expect_detected
              << " -> " << (passed ? "PASS" : "FAIL") << std::endl;
    all_passed = all_passed && passed;

    const std::string prefix = output_dir + "/" + test_case.name;
    trySave(test_case.image, prefix + "_1_original.png");
    trySave(warm_mask, prefix + "_2_mask.png");
    trySave(pseudo_thermal, prefix + "_3_pseudo_thermal.png");
    trySave(overlay, prefix + "_4_annotated.png");

    tryShow(test_case.name + " - original", test_case.image, show_windows);
    tryShow(test_case.name + " - mask", warm_mask, show_windows);
    tryShow(test_case.name + " - pseudo-thermal", pseudo_thermal, show_windows);
    tryShow(test_case.name + " - annotated", overlay, show_windows);
  }

  if (show_windows) {
    std::cout << "\nPress any key in an image window to close the demo..." << std::endl;
    try {
      cv::waitKey(0);
    } catch (const cv::Exception &) {
      // No display available; nothing to wait on.
    }
  }

  std::cout << "\n=====================================================" << std::endl;
  std::cout << " Overall result: " << (all_passed ? "ALL TEST CASES PASSED" : "SOME TEST CASES FAILED")
            << std::endl;
  std::cout << "=====================================================" << std::endl;

  return all_passed ? 0 : 1;
}
