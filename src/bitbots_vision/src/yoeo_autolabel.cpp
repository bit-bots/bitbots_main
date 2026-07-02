#include <algorithm>
#include <bitbots_vision/candidate.hpp>
#include <bitbots_vision/model_config.hpp>
#include <bitbots_vision/yoeo_handler.hpp>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace {

struct Args {
  fs::path input_dir;
  fs::path output_dir;
  std::string model_path{"2022_10_07_flo_torso21_yoeox"};
  float conf_threshold{0.5f};
  float nms_threshold{0.4f};
  float ball_threshold{0.5f};
  int max_balls{1};
};

std::string json_escape(const std::string& input) {
  std::ostringstream out;
  for (const char c : input) {
    switch (c) {
      case '"':
        out << "\\\"";
        break;
      case '\\':
        out << "\\\\";
        break;
      case '\n':
        out << "\\n";
        break;
      case '\r':
        out << "\\r";
        break;
      case '\t':
        out << "\\t";
        break;
      default:
        out << c;
    }
  }
  return out.str();
}

bool has_image_extension(const fs::path& path) {
  const std::string ext = path.extension().string();
  return ext == ".png" || ext == ".jpg" || ext == ".jpeg" || ext == ".PNG" || ext == ".JPG" || ext == ".JPEG";
}

void render_progress(size_t processed, size_t total, size_t selected) {
  constexpr size_t width = 40;
  const double ratio = total == 0 ? 1.0 : static_cast<double>(processed) / static_cast<double>(total);
  const size_t filled = std::min(width, static_cast<size_t>(ratio * static_cast<double>(width)));
  std::cerr << "\rYOEO [";
  for (size_t i = 0; i < width; ++i) {
    std::cerr << (i < filled ? '#' : '-');
  }
  std::cerr << "] " << processed << "/" << total << " selected=" << selected << std::flush;
  if (processed == total) {
    std::cerr << "\n";
  }
}

std::string resolve_model_path(const std::string& path) {
  if (fs::exists(path)) {
    return path;
  }
  const char* prefix = std::getenv("CONDA_PREFIX");
  if (!prefix) {
    prefix = std::getenv("PIXI_PREFIX");
  }
  if (prefix) {
    const std::string resolved = std::string(prefix) + "/share/bitbots_model_" + path;
    if (fs::exists(resolved)) {
      return resolved;
    }
  }
  return path;
}

Args parse_args(int argc, char** argv) {
  Args args;
  for (int i = 1; i < argc; ++i) {
    const std::string key = argv[i];
    auto require_value = [&](const std::string& option) -> std::string {
      if (i + 1 >= argc) {
        throw std::runtime_error("Missing value for " + option);
      }
      return argv[++i];
    };

    if (key == "--input-dir") {
      args.input_dir = require_value(key);
    } else if (key == "--output-dir") {
      args.output_dir = require_value(key);
    } else if (key == "--model-path") {
      args.model_path = require_value(key);
    } else if (key == "--conf-threshold") {
      args.conf_threshold = std::stof(require_value(key));
    } else if (key == "--nms-threshold") {
      args.nms_threshold = std::stof(require_value(key));
    } else if (key == "--ball-threshold") {
      args.ball_threshold = std::stof(require_value(key));
    } else if (key == "--max-balls") {
      args.max_balls = std::stoi(require_value(key));
    } else if (key == "--help" || key == "-h") {
      std::cout << "Usage: bitbots_vision_yoeo_autolabel --input-dir DIR --output-dir DIR [options]\n"
                << "Options:\n"
                << "  --model-path PATH_OR_NAME\n"
                << "  --conf-threshold VALUE\n"
                << "  --nms-threshold VALUE\n"
                << "  --ball-threshold VALUE\n"
                << "  --max-balls COUNT\n";
      std::exit(0);
    } else {
      throw std::runtime_error("Unknown argument: " + key);
    }
  }

  if (args.input_dir.empty() || args.output_dir.empty()) {
    throw std::runtime_error("--input-dir and --output-dir are required");
  }
  return args;
}

void write_mask_if_present(const cv::Mat& mask, const fs::path& path) {
  if (mask.empty()) {
    return;
  }
  fs::create_directories(path.parent_path());
  cv::imwrite(path.string(), mask);
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Args args = parse_args(argc, argv);
    fs::create_directories(args.output_dir);
    const fs::path masks_dir = args.output_dir / "masks";
    fs::create_directories(masks_dir);

    const std::string model_path = resolve_model_path(args.model_path);
    const auto logger = rclcpp::get_logger("bitbots_vision_yoeo_autolabel");
    const bitbots_vision::ModelConfig model_config = bitbots_vision::ModelConfig::load_from(model_path);
    bitbots_vision::YoeoHandler::Config cfg;
    cfg.conf_threshold = args.conf_threshold;
    cfg.nms_threshold = args.nms_threshold;
    bitbots_vision::YoeoHandler handler(model_path, model_config, cfg, logger);

    std::ofstream jsonl(args.output_dir / "detections.jsonl");
    if (!jsonl.is_open()) {
      throw std::runtime_error("Could not open detections.jsonl for writing");
    }

    size_t processed = 0;
    size_t selected = 0;
    std::vector<fs::path> image_paths;
    for (const auto& entry : fs::recursive_directory_iterator(args.input_dir)) {
      if (entry.is_regular_file() && has_image_extension(entry.path())) {
        image_paths.push_back(entry.path());
      }
    }
    std::sort(image_paths.begin(), image_paths.end());
    render_progress(processed, image_paths.size(), selected);

    for (const auto& image_path : image_paths) {
      cv::Mat image = cv::imread(image_path.string(), cv::IMREAD_COLOR);
      if (image.empty()) {
        std::cerr << "\nCould not read image: " << image_path << "\n";
        ++processed;
        render_progress(processed, image_paths.size(), selected);
        continue;
      }

      handler.set_image(image);
      handler.predict();
      auto balls = bitbots_vision::Candidate::sort_by_rating(handler.get_detection_candidates_for("ball"));
      balls = bitbots_vision::Candidate::filter_by_rating(balls, args.ball_threshold);
      if (args.max_balls > 0 && static_cast<int>(balls.size()) > args.max_balls) {
        balls.resize(static_cast<size_t>(args.max_balls));
      }
      ++processed;
      if (balls.empty()) {
        render_progress(processed, image_paths.size(), selected);
        continue;
      }

      const fs::path rel_image = fs::relative(image_path, args.input_dir);
      const std::string stem = rel_image.parent_path().empty()
                                   ? rel_image.stem().string()
                                   : rel_image.parent_path().string() + "_" + rel_image.stem().string();
      const fs::path field_mask = masks_dir / (stem + "_field.png");
      const fs::path line_mask = masks_dir / (stem + "_lines.png");
      write_mask_if_present(handler.get_segmentation_mask_for("field"), field_mask);
      write_mask_if_present(handler.get_segmentation_mask_for("lines"), line_mask);

      jsonl << "{\"image\":\"" << json_escape(rel_image.generic_string()) << "\",";
      jsonl << "\"width\":" << image.cols << ",\"height\":" << image.rows << ",";
      jsonl << "\"backend\":\"yoeo\",";
      jsonl << "\"detections\":[";
      for (size_t i = 0; i < balls.size(); ++i) {
        const auto& ball = balls[i];
        if (i > 0) {
          jsonl << ",";
        }
        jsonl << "{\"class\":\"ball\",\"confidence\":" << ball.rating << ",\"bbox\":[" << ball.x1 << "," << ball.y1
              << "," << ball.width << "," << ball.height << "]}";
      }
      jsonl << "],\"masks\":{";
      bool wrote_mask = false;
      if (fs::exists(field_mask)) {
        jsonl << "\"field\":\"" << json_escape(fs::relative(field_mask, args.output_dir).generic_string()) << "\"";
        wrote_mask = true;
      }
      if (fs::exists(line_mask)) {
        if (wrote_mask) {
          jsonl << ",";
        }
        jsonl << "\"lines\":\"" << json_escape(fs::relative(line_mask, args.output_dir).generic_string()) << "\"";
      }
      jsonl << "}}\n";
      ++selected;
      render_progress(processed, image_paths.size(), selected);
    }

    jsonl.close();
    std::cerr << "Processed " << processed << " images, selected " << selected << " ball detections.\n" << std::flush;
    // ONNX Runtime provider teardown can block after all outputs have been flushed.
    std::_Exit(0);
  } catch (const std::exception& e) {
    std::cerr << "ERROR: " << e.what() << "\n";
    return 1;
  }
  return 0;
}
