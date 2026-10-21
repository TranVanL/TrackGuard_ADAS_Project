#include "trackguard/replay/kitti_loader.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>

namespace trackguard::replay {
namespace {

constexpr std::size_t kLabelFieldCount = 17U;
constexpr std::size_t kOxtsFieldCount = 30U;

std::vector<std::string> splitFields(const std::string& line) {
    std::istringstream stream(line);
    std::vector<std::string> fields;
    std::string field;
    while (stream >> field) {
        fields.push_back(field);
    }
    return fields;
}

int parseInt(const std::string& token, const std::string& field_name) {
    std::size_t parsed_length = 0U;
    int value = 0;
    try {
        value = std::stoi(token, &parsed_length);
    } catch (const std::exception& error) {
        throw std::runtime_error(
            "invalid integer for " + field_name + ": " + error.what());
    }
    if (parsed_length != token.size()) {
        throw std::runtime_error("invalid integer for " + field_name + ": '" + token + "'");
    }
    return value;
}

double parseDouble(const std::string& token, const std::string& field_name) {
    std::size_t parsed_length = 0U;
    double value = 0.0;
    try {
        value = std::stod(token, &parsed_length);
    } catch (const std::exception& error) {
        throw std::runtime_error(
            "invalid floating-point value for " + field_name + ": " + error.what());
    }
    if (parsed_length != token.size()) {
        throw std::runtime_error(
            "invalid floating-point value for " + field_name + ": '" + token + "'");
    }
    return value;
}

int parseIntegralDouble(const std::string& token, const std::string& field_name) {
    const double value = parseDouble(token, field_name);
    if (!std::isfinite(value) || std::trunc(value) != value ||
        value < static_cast<double>(std::numeric_limits<int>::min()) ||
        value > static_cast<double>(std::numeric_limits<int>::max())) {
        throw std::runtime_error("expected an integer-valued number for " + field_name);
    }
    return static_cast<int>(value);
}

void requireFinite(double value, const std::string& field_name) {
    if (!std::isfinite(value)) {
        throw std::runtime_error(field_name + " must be finite");
    }
}

KittiObjectLabel parseLabel(const std::vector<std::string>& fields) {
    if (fields.size() < kLabelFieldCount) {
        throw std::runtime_error(
            "expected at least 17 fields, got " + std::to_string(fields.size()));
    }
    if (fields.size() > kLabelFieldCount + 1U) {
        throw std::runtime_error("expected 17 fields or one optional score field");
    }

    KittiObjectLabel label;
    label.frame_index = parseInt(fields[0], "frame");
    label.ground_truth_track_id = parseInt(fields[1], "track_id");
    label.type = fields[2];
    label.truncated = parseDouble(fields[3], "truncated");
    label.occluded = parseInt(fields[4], "occluded");
    label.alpha = parseDouble(fields[5], "alpha");

    for (std::size_t index = 0U; index < label.bbox.size(); ++index) {
        label.bbox[index] = parseDouble(fields[6U + index], "bbox");
        requireFinite(label.bbox[index], "bbox");
    }
    for (std::size_t index = 0U; index < label.dimensions.size(); ++index) {
        label.dimensions[index] = parseDouble(fields[10U + index], "dimensions");
        requireFinite(label.dimensions[index], "dimensions");
    }
    for (std::size_t index = 0U; index < label.location.size(); ++index) {
        label.location[index] = parseDouble(fields[13U + index], "location");
        requireFinite(label.location[index], "location");
    }
    label.rotation_y = parseDouble(fields[16], "rotation_y");

    requireFinite(label.truncated, "truncated");
    requireFinite(label.alpha, "alpha");
    requireFinite(label.rotation_y, "rotation_y");
    if (label.frame_index < 0) {
        throw std::runtime_error("frame must be non-negative");
    }

    if (label.type != "DontCare") {
        if (label.occluded < 0 || label.occluded > 3) {
            throw std::runtime_error("occluded must be in [0, 3]");
        }
        for (const double dimension : label.dimensions) {
            if (dimension <= 0.0) {
                throw std::runtime_error("trackable object dimensions must be positive");
            }
        }
    }

    if (fields.size() > kLabelFieldCount) {
        label.score = parseDouble(fields[kLabelFieldCount], "score");
        requireFinite(*label.score, "score");
    }
    return label;
}

RawDetection toRawDetection(const KittiObjectLabel& label, Timestamp timestamp) {
    RawDetection detection;
    detection.t = timestamp;
    detection.sensor = SensorType::Camera;
    detection.pos = {label.location[0], label.location[2]};
    detection.vel = {0.0, 0.0};
    detection.velocity_valid = false;
    detection.confidence = 1.0;
    detection.frame_index = label.frame_index;
    detection.ground_truth_track_id = label.ground_truth_track_id;
    detection.object_type = label.type;
    detection.truncated = label.truncated;
    detection.occluded = label.occluded;
    detection.alpha = label.alpha;
    detection.bbox = label.bbox;
    detection.dimensions = label.dimensions;
    detection.camera_location = label.location;
    detection.rotation_y = label.rotation_y;
    return detection;
}

EgoMotionSample parseOxts(
    const std::vector<std::string>& fields,
    int frame_index,
    Timestamp timestamp) {
    if (fields.size() < kOxtsFieldCount) {
        throw std::runtime_error(
            "expected at least 30 fields, got " + std::to_string(fields.size()));
    }

    std::array<double, 25> values{};
    for (std::size_t index = 0U; index < values.size(); ++index) {
        values[index] = parseDouble(fields[index], "oxts field " + std::to_string(index));
    }

    EgoMotionSample sample;
    sample.frame_index = frame_index;
    sample.t = timestamp;
    sample.latitude_deg = values[0];
    sample.longitude_deg = values[1];
    sample.altitude_m = values[2];
    sample.roll_rad = values[3];
    sample.pitch_rad = values[4];
    sample.yaw_rad = values[5];
    sample.velocity_north_mps = values[6];
    sample.velocity_east_mps = values[7];
    sample.velocity_forward_mps = values[8];
    sample.velocity_left_mps = values[9];
    sample.velocity_up_mps = values[10];
    sample.acceleration_x_mps2 = values[11];
    sample.acceleration_y_mps2 = values[12];
    sample.acceleration_z_mps2 = values[13];
    sample.acceleration_forward_mps2 = values[14];
    sample.acceleration_left_mps2 = values[15];
    sample.acceleration_up_mps2 = values[16];
    sample.angular_rate_x_radps = values[17];
    sample.angular_rate_y_radps = values[18];
    sample.angular_rate_z_radps = values[19];
    sample.angular_rate_forward_radps = values[20];
    sample.angular_rate_left_radps = values[21];
    sample.angular_rate_up_radps = values[22];
    sample.position_accuracy_m = values[23];
    sample.velocity_accuracy_mps = values[24];
    sample.navstat = parseIntegralDouble(fields[25], "navstat");
    sample.numsats = parseIntegralDouble(fields[26], "numsats");
    sample.posmode = parseIntegralDouble(fields[27], "posmode");
    sample.velmode = parseIntegralDouble(fields[28], "velmode");
    sample.orimode = parseIntegralDouble(fields[29], "orimode");
    return sample;
}

void addIssue(
    KittiSequence& sequence,
    const std::string& source,
    std::size_t line_number,
    int frame_index,
    const std::exception& error) {
    sequence.issues.push_back({source, line_number, frame_index, error.what()});
}

}  // namespace

KittiTrackingLoader::KittiTrackingLoader(double frame_rate_hz)
    : frame_rate_hz_(frame_rate_hz) {
    if (!std::isfinite(frame_rate_hz_) || frame_rate_hz_ <= 0.0) {
        throw std::invalid_argument("KITTI frame rate must be finite and positive");
    }
}

Timestamp KittiTrackingLoader::timestampForFrame(int frame_index) const {
    const auto timestamp_us = static_cast<long long>(std::llround(
        static_cast<double>(frame_index) * 1'000'000.0 / frame_rate_hz_));
    return Timestamp(timestamp_us);
}

KittiSequence KittiTrackingLoader::load(
    const std::string& label_path,
    const std::string& oxts_path) const {
    std::ifstream label_stream(label_path);
    if (!label_stream) {
        throw std::runtime_error("Cannot open KITTI label file: " + label_path);
    }

    std::ifstream oxts_stream(oxts_path);
    if (!oxts_stream) {
        throw std::runtime_error("Cannot open KITTI OXTS file: " + oxts_path);
    }

    return load(label_stream, oxts_stream, label_path, oxts_path);
}

KittiSequence KittiTrackingLoader::load(
    std::istream& label_stream,
    std::istream& oxts_stream,
    const std::string& label_source,
    const std::string& oxts_source) const {
    std::map<int, std::vector<KittiObjectLabel>> labels_by_frame;
    std::map<int, EgoMotionSample> ego_by_frame;
    KittiSequence sequence;
    int highest_frame = -1;

    std::string line;
    std::size_t line_number = 0U;
    while (std::getline(label_stream, line)) {
        ++line_number;
        if (line.find_first_not_of(" \t\r\n") == std::string::npos) {
            continue;
        }

        const std::vector<std::string> fields = splitFields(line);
        int frame_index = -1;
        try {
            if (!fields.empty()) {
                frame_index = parseInt(fields[0], "frame");
            }
            const KittiObjectLabel label = parseLabel(fields);
            highest_frame = std::max(highest_frame, label.frame_index);
            labels_by_frame[label.frame_index].push_back(label);
        } catch (const std::exception& error) {
            addIssue(sequence, label_source, line_number, frame_index, error);
            if (frame_index >= 0) {
                highest_frame = std::max(highest_frame, frame_index);
            }
        }
    }

    line_number = 0U;
    int logical_frame_index = 0;
    while (std::getline(oxts_stream, line)) {
        ++line_number;
        if (line.find_first_not_of(" \t\r\n") == std::string::npos) {
            continue;
        }

        const int frame_index = logical_frame_index;
        ++logical_frame_index;
        highest_frame = std::max(highest_frame, frame_index);
        const std::vector<std::string> fields = splitFields(line);
        try {
            const EgoMotionSample sample =
                parseOxts(fields, frame_index, timestampForFrame(frame_index));
            ego_by_frame[frame_index] = sample;
        } catch (const std::exception& error) {
            addIssue(sequence, oxts_source, line_number, frame_index, error);
        }
    }

    if (highest_frame < 0) {
        return sequence;
    }

    sequence.frames.resize(static_cast<std::size_t>(highest_frame + 1));
    for (int frame_index = 0; frame_index <= highest_frame; ++frame_index) {
        KittiFrame& frame = sequence.frames[static_cast<std::size_t>(frame_index)];
        frame.frame_index = frame_index;
        frame.t = timestampForFrame(frame_index);

        const auto labels_iterator = labels_by_frame.find(frame_index);
        if (labels_iterator != labels_by_frame.end()) {
            frame.labels = labels_iterator->second;
            for (const KittiObjectLabel& label : frame.labels) {
                if (label.type != "DontCare") {
                    frame.detections.push_back(toRawDetection(label, frame.t));
                }
            }
        }

        const auto ego_iterator = ego_by_frame.find(frame_index);
        if (ego_iterator != ego_by_frame.end()) {
            frame.ego_motion = ego_iterator->second;
        }
    }

    return sequence;
}

}  // namespace trackguard::replay