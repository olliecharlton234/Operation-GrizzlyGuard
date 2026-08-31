// Copyright 2026 Operation GrizzlyGuard (Group 15)

#include "grizzlyguard_perception/thermal_detector.hpp"

#include <algorithm>
#include <iomanip>
#include <sstream>
#include <vector>

namespace grizzlyguard_perception
{

namespace
{

/// Threshold hsv_image against one HsvRange, writing a CV_8UC1 mask to out.
void inRangeHsv(const cv::Mat & hsv_image, const HsvRange & range, cv::Mat & out)
{
  cv::inRange(
    hsv_image,
    cv::Scalar(range.h_min, range.s_min, range.v_min),
    cv::Scalar(range.h_max, range.s_max, range.v_max),
    out);
}

}  // namespace

cv::Mat computeWarmMask(const cv::Mat & bgr_image, const ThermalDetectorParams & params)
{
  if (bgr_image.empty()) {
    return cv::Mat();
  }

  cv::Mat hsv_image;
  cv::cvtColor(bgr_image, hsv_image, cv::COLOR_BGR2HSV);

  // Red wraps around hue 0/179, so it is split into a low band and a high
  // band; orange and yellow are each a single contiguous band.
  cv::Mat mask_red_low, mask_red_high, mask_orange, mask_yellow;
  inRangeHsv(hsv_image, params.red_low, mask_red_low);
  inRangeHsv(hsv_image, params.red_high, mask_red_high);
  inRangeHsv(hsv_image, params.orange, mask_orange);
  inRangeHsv(hsv_image, params.yellow, mask_yellow);

  cv::Mat warm_mask;
  cv::bitwise_or(mask_red_low, mask_red_high, warm_mask);
  cv::bitwise_or(warm_mask, mask_orange, warm_mask);
  cv::bitwise_or(warm_mask, mask_yellow, warm_mask);

  // Morphological opening removes small speckle noise; closing then fills
  // small holes inside otherwise-solid warm regions.
  const int kernel_size = std::max(1, params.morph_kernel_size);
  const cv::Mat kernel = cv::getStructuringElement(
    cv::MORPH_ELLIPSE, cv::Size(kernel_size, kernel_size));

  if (params.morph_open_iterations > 0) {
    cv::morphologyEx(
      warm_mask, warm_mask, cv::MORPH_OPEN, kernel, cv::Point(-1, -1),
      params.morph_open_iterations);
  }
  if (params.morph_close_iterations > 0) {
    cv::morphologyEx(
      warm_mask, warm_mask, cv::MORPH_CLOSE, kernel, cv::Point(-1, -1),
      params.morph_close_iterations);
  }

  return warm_mask;
}

DetectionResult detectWarmRegion(const cv::Mat & warm_mask, const ThermalDetectorParams & params)
{
  DetectionResult result;

  if (warm_mask.empty()) {
    return result;
  }

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(warm_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  double best_area = 0.0;
  int best_index = -1;
  for (std::size_t i = 0; i < contours.size(); ++i) {
    const double area = cv::contourArea(contours[i]);
    if (area < params.min_contour_area) {
      // Too small to be a plausible heat signature; treat as noise.
      continue;
    }
    if (area > best_area) {
      best_area = area;
      best_index = static_cast<int>(i);
    }
  }

  if (best_index < 0) {
    return result;  // No contour met the minimum-area requirement.
  }

  const cv::Rect raw_bbox = cv::boundingRect(contours[static_cast<std::size_t>(best_index)]);
  // Clip to the mask bounds defensively; boundingRect() should already be
  // in-bounds, but this keeps the pixel-counting step below always safe.
  const cv::Rect bbox = raw_bbox & cv::Rect(0, 0, warm_mask.cols, warm_mask.rows);

  const double bbox_area = static_cast<double>(bbox.area());
  std::size_t warm_pixel_count = 0;
  double confidence = 0.0;
  if (bbox_area > 0.0) {
    warm_pixel_count = static_cast<std::size_t>(cv::countNonZero(warm_mask(bbox)));
    confidence = static_cast<double>(warm_pixel_count) / bbox_area;
  }

  result.bounding_box = bbox;
  result.area = best_area;
  result.warm_pixel_count = warm_pixel_count;
  result.confidence = confidence;
  result.centre = cv::Point2f(
    static_cast<float>(bbox.x) + static_cast<float>(bbox.width) / 2.0f,
    static_cast<float>(bbox.y) + static_cast<float>(bbox.height) / 2.0f);

  // Require both a large-enough area (already filtered above) and a
  // high-enough warm-pixel density inside the bounding box. This rejects
  // sparse/scattered warm pixels that happen to have a large bounding box
  // but are not a solid blob.
  result.detected = confidence >= params.confidence_threshold;

  return result;
}

cv::Mat drawDetectionOverlay(const cv::Mat & bgr_image, const DetectionResult & result)
{
  cv::Mat overlay = bgr_image.clone();

  const int font = cv::FONT_HERSHEY_SIMPLEX;
  const cv::Scalar white(255, 255, 255);
  const cv::Scalar red(0, 0, 255);
  const cv::Scalar orange(0, 165, 255);

  if (!result.detected) {
    cv::putText(overlay, "NO WARM OBJECT DETECTED", cv::Point(10, 25), font, 0.7, white, 2);
    return overlay;
  }

  cv::rectangle(overlay, result.bounding_box, orange, 2);
  cv::circle(overlay, result.centre, 5, red, cv::FILLED);
  cv::drawMarker(overlay, result.centre, red, cv::MARKER_CROSS, 14, 1);

  int line_y = 25;
  const int line_step = 20;

  cv::putText(overlay, "WARM OBJECT DETECTED", cv::Point(10, line_y), font, 0.7, red, 2);
  line_y += line_step + 5;

  std::ostringstream bbox_text;
  bbox_text << "BBox: (" << result.bounding_box.x << ", " << result.bounding_box.y << ") "
            << result.bounding_box.width << "x" << result.bounding_box.height << " px";
  cv::putText(overlay, bbox_text.str(), cv::Point(10, line_y), font, 0.5, white, 1);
  line_y += line_step;

  std::ostringstream centre_text;
  centre_text << "Centre: (" << static_cast<int>(result.centre.x) << ", "
              << static_cast<int>(result.centre.y) << ") px";
  cv::putText(overlay, centre_text.str(), cv::Point(10, line_y), font, 0.5, white, 1);
  line_y += line_step;

  std::ostringstream area_text;
  area_text << "Area: " << static_cast<int>(result.area) << " px^2";
  cv::putText(overlay, area_text.str(), cv::Point(10, line_y), font, 0.5, white, 1);
  line_y += line_step;

  std::ostringstream confidence_text;
  confidence_text << "Confidence: " << std::fixed << std::setprecision(2) << result.confidence;
  cv::putText(overlay, confidence_text.str(), cv::Point(10, line_y), font, 0.5, white, 1);

  return overlay;
}

cv::Mat computePseudoThermal(const cv::Mat & bgr_image, const DetectionResult & result)
{
  if (bgr_image.empty()) {
    return cv::Mat();
  }

  // Visual-only "warmth" score per pixel: how much the red channel
  // dominates over the average of green and blue. This is a simple stand-in
  // for temperature, not a physical measurement -- see the README.
  cv::Mat bgr_float;
  bgr_image.convertTo(bgr_float, CV_32FC3);

  std::vector<cv::Mat> channels(3);
  cv::split(bgr_float, channels);  // 0 = B, 1 = G, 2 = R
  const cv::Mat warmth = channels[2] - 0.5f * (channels[0] + channels[1]);

  cv::Mat warmth_u8;
  cv::normalize(warmth, warmth_u8, 0, 255, cv::NORM_MINMAX, CV_8UC1);

  cv::Mat pseudo_thermal;
  // COLORMAP_JET runs dark blue -> cyan -> green/yellow -> orange -> red,
  // which matches the requested cool/medium/warm visual convention.
  cv::applyColorMap(warmth_u8, pseudo_thermal, cv::COLORMAP_JET);

  if (result.detected) {
    cv::rectangle(pseudo_thermal, result.bounding_box, cv::Scalar(255, 255, 255), 2);
    cv::circle(pseudo_thermal, result.centre, 5, cv::Scalar(255, 255, 255), cv::FILLED);
  }

  return pseudo_thermal;
}

}  // namespace grizzlyguard_perception
