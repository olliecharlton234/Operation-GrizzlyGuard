// Copyright 2026 Operation GrizzlyGuard (Group 15)
//
// Pure, ROS-independent image-processing pipeline for the GrizzlyGuard
// perception prototype.
//
// IMPORTANT: this is a colour-segmentation *placeholder* for a real thermal
// camera. It treats red/orange/yellow pixels in an ordinary RGB image as
// "warm" and green/blue/dark pixels as "cool". It is a visual simulation
// only and does not measure real temperature. See the package README for
// details and for the recommended next step (a simulated thermal camera
// sensor / real thermal data).
//
// Every function here operates on plain cv::Mat / a small set of structs, so
// it can be exercised from:
//   - thermal_perception_node.cpp   (the live ROS 2 node)
//   - thermal_demo_main.cpp         (the standalone, ROS-free demo/test tool)
//   - test/test_thermal_detector.cpp (gtest unit tests, no ROS required)

#ifndef GRIZZLYGUARD_PERCEPTION__THERMAL_DETECTOR_HPP_
#define GRIZZLYGUARD_PERCEPTION__THERMAL_DETECTOR_HPP_

#include <cstddef>

#include <opencv2/opencv.hpp>

namespace grizzlyguard_perception
{

/// One inclusive HSV band using OpenCV's 8-bit HSV convention
/// (H: 0-179, S: 0-255, V: 0-255).
struct HsvRange
{
  int h_min = 0;
  int h_max = 179;
  int s_min = 0;
  int s_max = 255;
  int v_min = 0;
  int v_max = 255;
};

/// All tunable parameters for the colour-based "warm object" pipeline.
///
/// These map 1:1 onto the ROS 2 parameters declared by
/// ThermalPerceptionNode, so the exact same struct can be driven either from
/// live ROS parameters or from a plain C++ program (demo / gtest) with no
/// ROS dependency at all.
struct ThermalDetectorParams
{
  // Red wraps around hue 0/179 in OpenCV's HSV space, so it needs two bands.
  HsvRange red_low{0, 10, 90, 255, 90, 255};
  HsvRange red_high{170, 179, 90, 255, 90, 255};
  HsvRange orange{11, 25, 90, 255, 90, 255};
  HsvRange yellow{26, 35, 90, 255, 90, 255};

  int morph_kernel_size = 5;
  int morph_open_iterations = 1;
  int morph_close_iterations = 2;

  // Contours smaller than this (in pixels^2) are ignored as noise.
  double min_contour_area = 400.0;

  // Fraction of "warm" pixels required inside the candidate bounding box for
  // a detection to be accepted (see detectWarmRegion()).
  double confidence_threshold = 0.35;
};

/// Result of running the detector on a single frame.
struct DetectionResult
{
  bool detected = false;
  cv::Rect bounding_box;
  cv::Point2f centre{0.0f, 0.0f};
  double area = 0.0;         // Contour area of the selected warm region, in px^2.
  double confidence = 0.0;   // Proportion of warm pixels inside bounding_box, in [0, 1].
  std::size_t warm_pixel_count = 0;
};

/// Convert a BGR frame to a binary "warm object" mask.
///
/// Steps: BGR -> HSV, threshold the red/orange/yellow bands, OR them
/// together, then clean up the result with morphological opening (removes
/// small speckle noise) followed by closing (fills small holes).
///
/// Returns a single-channel CV_8UC1 mask (255 = warm, 0 = not warm), the
/// same size as bgr_image. Returns an empty Mat if bgr_image is empty.
cv::Mat computeWarmMask(const cv::Mat & bgr_image, const ThermalDetectorParams & params);

/// Find contours in a binary warm mask and select the largest one that meets
/// the minimum-area and confidence thresholds.
///
/// If no contour is large enough, or the best candidate's confidence is
/// below params.confidence_threshold, the returned DetectionResult has
/// detected == false (all other fields default-initialised / zero).
DetectionResult detectWarmRegion(const cv::Mat & warm_mask, const ThermalDetectorParams & params);

/// Draw the human-readable detection overlay onto a copy of bgr_image:
/// bounding box + centre marker + "WARM OBJECT DETECTED" and its stats, or
/// "NO WARM OBJECT DETECTED" when nothing valid was found.
cv::Mat drawDetectionOverlay(const cv::Mat & bgr_image, const DetectionResult & result);

/// Build a pseudo-thermal colour-mapped visualisation of a BGR frame, with
/// the detection bounding box / centre marker drawn on top when detected.
///
/// NOTE: this is a *visual simulation* based on how red-dominant each pixel
/// is (using OpenCV's COLORMAP_JET: cool -> dark blue/purple, mid -> green
/// /yellow, warm -> orange/red). It is NOT a calibrated temperature
/// measurement of any kind.
cv::Mat computePseudoThermal(const cv::Mat & bgr_image, const DetectionResult & result);

}  // namespace grizzlyguard_perception

#endif  // GRIZZLYGUARD_PERCEPTION__THERMAL_DETECTOR_HPP_
