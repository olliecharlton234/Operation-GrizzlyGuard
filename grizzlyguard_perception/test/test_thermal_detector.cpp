#include <gtest/gtest.h>

#include <opencv2/opencv.hpp>

#include "grizzlyguard_perception/thermal_detector.hpp"

namespace
{

using grizzlyguard_perception::computeWarmMask;
using grizzlyguard_perception::detectWarmRegion;
using grizzlyguard_perception::DetectionResult;
using grizzlyguard_perception::ThermalDetectorParams;

cv::Mat makeForestBackground(int size = 200)
{
  return cv::Mat(size, size, CV_8UC3, cv::Scalar(25, 70, 25));  // BGR: dark green.
}

}  
TEST(ThermalDetector, DetectsLargeWarmRegion)
{
  cv::Mat image = makeForestBackground();
  cv::ellipse(
    image, cv::Point(100, 100), cv::Size(40, 30), 0.0, 0.0, 360.0,
    cv::Scalar(25, 80, 210), cv::FILLED);

  const ThermalDetectorParams params;
  const cv::Mat mask = computeWarmMask(image, params);
  const DetectionResult result = detectWarmRegion(mask, params);

  EXPECT_TRUE(result.detected);
  EXPECT_GT(result.area, params.min_contour_area);
  EXPECT_GE(result.confidence, params.confidence_threshold);
  EXPECT_NEAR(result.centre.x, 100.0, 15.0);
  EXPECT_NEAR(result.centre.y, 100.0, 15.0);
}

// Test case 2: no warm object -- a plain forest background must not trigger
// a detection.
TEST(ThermalDetector, NoDetectionOnPlainForestBackground)
{
  const cv::Mat image = makeForestBackground();
  const ThermalDetectorParams params;

  const cv::Mat mask = computeWarmMask(image, params);
  const DetectionResult result = detectWarmRegion(mask, params);

  EXPECT_FALSE(result.detected);
}

// Test case 3: small warm-coloured noise -- a dot far smaller than
// min_contour_area must be rejected, not reported as a detection.
TEST(ThermalDetector, RejectsSmallWarmNoise)
{
  cv::Mat image = makeForestBackground();
  cv::circle(image, cv::Point(50, 50), 3, cv::Scalar(20, 90, 200), cv::FILLED);

  const ThermalDetectorParams params;
  const cv::Mat mask = computeWarmMask(image, params);
  const DetectionResult result = detectWarmRegion(mask, params);

  EXPECT_FALSE(result.detected);
}

// Sanity check: an empty input image must not crash the pipeline.
TEST(ThermalDetector, HandlesEmptyImageSafely)
{
  const cv::Mat empty_image;
  const ThermalDetectorParams params;

  const cv::Mat mask = computeWarmMask(empty_image, params);
  EXPECT_TRUE(mask.empty());

  const DetectionResult result = detectWarmRegion(mask, params);
  EXPECT_FALSE(result.detected);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
