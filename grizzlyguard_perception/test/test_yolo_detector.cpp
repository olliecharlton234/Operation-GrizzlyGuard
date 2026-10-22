// Unit tests for the ROS-independent YOLO helpers (synthetic data; no model file needed).
// Optional real-model test: set GG_TEST_MODEL=/path/model.onnx to also load and run it.

#include <gtest/gtest.h>

#include <cstdlib>
#include <vector>

#include <opencv2/core.hpp>

#include "grizzlyguard_perception/yolo_detector.hpp"

using namespace grizzlyguard_perception;

TEST(Letterbox, CameraFrameGeometryAndPadding)
{
  cv::Mat img(480, 720, CV_8UC3, cv::Scalar(10, 20, 30));  // drone camera size
  LetterboxInfo info;
  cv::Mat lb = letterbox(img, 640, info);
  EXPECT_EQ(lb.rows, 640);
  EXPECT_EQ(lb.cols, 640);
  EXPECT_NEAR(info.scale, 640.0f / 720.0f, 1e-6);
  EXPECT_EQ(info.pad_x, 0);
  EXPECT_EQ(info.pad_y, 106);  // (640 - 427) / 2 = 106.5 -> round(106.5 - 0.1)
  EXPECT_EQ(lb.at<cv::Vec3b>(0, 0), cv::Vec3b(114, 114, 114));
  EXPECT_EQ(lb.at<cv::Vec3b>(320, 320), cv::Vec3b(10, 20, 30));
}

TEST(InputTensor, RgbOrderAndScaling)
{
  cv::Mat img(4, 4, CV_8UC3, cv::Scalar(51, 102, 255));  // B, G, R
  std::vector<float> t;
  toInputTensor(img, t);
  ASSERT_EQ(t.size(), 3u * 16u);
  EXPECT_FLOAT_EQ(t[0], 1.0f);        // R plane first
  EXPECT_FLOAT_EQ(t[16], 0.4f);       // G
  EXPECT_FLOAT_EQ(t[32], 0.2f);       // B
}

// Build a raw [4+nc, N] tensor with given anchors.
struct Anchor {float cx, cy, w, h; std::vector<float> scores;};
static std::vector<float> rawTensor(const std::vector<Anchor> & a, int nc)
{
  const int n = static_cast<int>(a.size());
  std::vector<float> t((4 + nc) * n, 0.0f);
  for (int j = 0; j < n; ++j) {
    t[0 * n + j] = a[j].cx; t[1 * n + j] = a[j].cy;
    t[2 * n + j] = a[j].w;  t[3 * n + j] = a[j].h;
    for (int c = 0; c < nc; ++c) {t[(4 + c) * n + j] = a[j].scores[c];}
  }
  return t;
}

TEST(DecodeRaw, NmsKeepsBestPerClassAndDropsLowScores)
{
  const auto t = rawTensor({
    {100, 100, 50, 50, {0.1f, 0.9f, 0.0f}},
    {102, 101, 50, 50, {0.1f, 0.8f, 0.0f}},   // overlaps, same class -> suppressed
    {101, 100, 50, 50, {0.0f, 0.1f, 0.7f}},   // overlaps, different class -> kept
    {300, 300, 20, 20, {0.0f, 0.1f, 0.2f}},   // below threshold
  }, 3);
  auto d = decodeRaw(t.data(), 7, 4, 0.25f, 0.45f);
  ASSERT_EQ(d.size(), 2u);
  EXPECT_EQ(d[0].class_id, 1);
  EXPECT_FLOAT_EQ(d[0].score, 0.9f);
  EXPECT_FLOAT_EQ(d[0].box.x, 75.0f);
  EXPECT_FLOAT_EQ(d[0].box.width, 50.0f);
  EXPECT_EQ(d[1].class_id, 2);
}

TEST(DecodeRaw, ArgmaxPreventsRelabellingAnotherAnimal)
{
  // Elephant (0) beats bear (1) at this anchor: it must come out as elephant, not bear.
  const auto t = rawTensor({{50, 50, 10, 10, {0.6f, 0.3f}}}, 2);
  auto d = decodeRaw(t.data(), 6, 1, 0.25f, 0.45f);
  ASSERT_EQ(d.size(), 1u);
  EXPECT_EQ(d[0].class_id, 0);
}

TEST(DecodeEndToEnd, ThresholdAndCorners)
{
  const std::vector<float> t = {
    10, 20, 110, 220, 0.8f, 21,
    0, 0, 5, 5, 0.1f, 0,
  };
  auto d = decodeEndToEnd(t.data(), 2, 0.25f);
  ASSERT_EQ(d.size(), 1u);
  EXPECT_EQ(d[0].class_id, 21);
  EXPECT_FLOAT_EQ(d[0].box.width, 100.0f);
  EXPECT_FLOAT_EQ(d[0].box.height, 200.0f);
}

TEST(ScaleToOriginal, RoundTripThroughLetterbox)
{
  cv::Mat img(480, 720, CV_8UC3);
  LetterboxInfo info;
  letterbox(img, 640, info);
  const cv::Rect2f orig(300, 200, 120, 90);
  Detection d;
  d.box = cv::Rect2f(orig.x * info.scale + info.pad_x, orig.y * info.scale + info.pad_y,
      orig.width * info.scale, orig.height * info.scale);
  std::vector<Detection> v{d};
  scaleToOriginal(v, info, img.size());
  ASSERT_EQ(v.size(), 1u);
  EXPECT_NEAR(v[0].box.x, orig.x, 1e-3);
  EXPECT_NEAR(v[0].box.y, orig.y, 1e-3);
  EXPECT_NEAR(v[0].box.width, orig.width, 1e-3);
  EXPECT_NEAR(v[0].box.height, orig.height, 1e-3);
}

TEST(ScaleToOriginal, ClipsAndDropsBoxesInPadding)
{
  cv::Mat img(480, 720, CV_8UC3);
  LetterboxInfo info;
  letterbox(img, 640, info);
  Detection in_pad;   // entirely inside the top grey padding
  in_pad.box = cv::Rect2f(10, 5, 50, 50);
  Detection overhang;  // runs off the right edge
  overhang.box = cv::Rect2f(600, 300, 100, 50);
  std::vector<Detection> v{in_pad, overhang};
  scaleToOriginal(v, info, img.size());
  ASSERT_EQ(v.size(), 1u);
  EXPECT_FLOAT_EQ(v[0].box.x + v[0].box.width, 720.0f);
}

TEST(ClassNames, ParseAndResolveExactly)
{
  const auto names = parseClassNames(
    "{0: 'person', 21: 'bear', 77: 'teddy bear', 78: \"hair drier\"}");
  ASSERT_EQ(names.size(), 4u);
  EXPECT_EQ(names.at(78), "hair drier");
  EXPECT_EQ(resolveClassId(names, "bear"), 21);   // not 77 ("teddy bear")
  EXPECT_EQ(resolveClassId(names, "Bear"), 21);
  EXPECT_EQ(resolveClassId(names, "moose"), -1);
  // A custom single-class model puts the bear at id 0.
  EXPECT_EQ(resolveClassId(parseClassNames("{0: 'bear'}"), "bear"), 0);
}

TEST(Format, DetectFromMetadataAndShape)
{
  EXPECT_EQ(detectFormat({1, 84, 8400}, "False"), OutputFormat::Raw);
  EXPECT_EQ(detectFormat({1, 300, 6}, "True"), OutputFormat::EndToEnd);
  EXPECT_EQ(detectFormat({1, 300, 6}, ""), OutputFormat::EndToEnd);
  EXPECT_EQ(detectFormat({1, 5, 8400}, ""), OutputFormat::Raw);   // single-class raw
  EXPECT_THROW(parseOutputFormat("onnx"), std::invalid_argument);
}

TEST(Detector, MissingModelFailsClearly)
{
  YoloDetector::Config cfg;
  cfg.model_path = "/nonexistent/model.onnx";
  EXPECT_THROW({YoloDetector d(cfg);}, std::runtime_error);
}

TEST(Detector, RealModelOptional)
{
  const char * path = std::getenv("GG_TEST_MODEL");
  if (!path) {GTEST_SKIP() << "set GG_TEST_MODEL to run against a real model";}
  YoloDetector::Config cfg;
  cfg.model_path = path;
  YoloDetector d(cfg);
  EXPECT_GE(resolveClassId(d.classNames(), "bear"), 0);
  cv::Mat blank(480, 720, CV_8UC3, cv::Scalar(90, 140, 60));
  EXPECT_NO_THROW(d.detect(blank));
  EXPECT_THROW(d.detect(cv::Mat()), std::invalid_argument);
  cv::Mat gray(480, 720, CV_8UC1);
  EXPECT_THROW(d.detect(gray), std::invalid_argument);
}