#pragma once

#include <cstdint>
#include <stdexcept>
#include <string>
#include <string_view>

/// Minimal protobuf wire-format reader/writer helpers for rewriting
/// serialized ONNX models without an ONNX / protobuf library.
/// Internal to bitbots_vision (not installed).
namespace bitbots_vision::onnx_wire {

enum WireType : uint32_t { kVarint = 0, kFixed64 = 1, kLengthDelimited = 2, kFixed32 = 5 };

/// One field of a protobuf message. `payload` holds the decoded bytes for
/// length-delimited fields; `raw` always spans the complete encoded field
/// (tag included) so unmodified fields can be copied verbatim.
struct Field {
  uint32_t number{0};
  WireType wire_type{kVarint};
  uint64_t value{0};  // varint / fixed32 / fixed64 value
  std::string_view payload;
  std::string_view raw;
};

class Reader {
 public:
  explicit Reader(std::string_view data) : data_(data) {}

  bool done() const { return pos_ >= data_.size(); }

  Field next() {
    const size_t start = pos_;
    const uint64_t key = read_varint();
    Field field;
    field.number = static_cast<uint32_t>(key >> 3);
    field.wire_type = static_cast<WireType>(key & 0x7);
    switch (field.wire_type) {
      case kVarint:
        field.value = read_varint();
        break;
      case kFixed64:
        field.value = read_fixed(8);
        break;
      case kFixed32:
        field.value = read_fixed(4);
        break;
      case kLengthDelimited: {
        const uint64_t len = read_varint();
        require(len);
        field.payload = data_.substr(pos_, len);
        pos_ += len;
        break;
      }
      default:
        throw std::runtime_error("ONNX protobuf: unsupported wire type " + std::to_string(field.wire_type));
    }
    field.raw = data_.substr(start, pos_ - start);
    return field;
  }

  uint64_t read_varint() {
    uint64_t result = 0;
    for (int shift = 0; shift < 64; shift += 7) {
      require(1);
      const auto byte = static_cast<uint8_t>(data_[pos_++]);
      result |= static_cast<uint64_t>(byte & 0x7F) << shift;
      if ((byte & 0x80) == 0) {
        return result;
      }
    }
    throw std::runtime_error("ONNX protobuf: malformed varint");
  }

 private:
  void require(uint64_t n) const {
    if (n > data_.size() - pos_) {
      throw std::runtime_error("ONNX protobuf: unexpected end of data");
    }
  }

  uint64_t read_fixed(int bytes) {
    require(static_cast<uint64_t>(bytes));
    uint64_t result = 0;
    for (int i = 0; i < bytes; ++i) {
      result |= static_cast<uint64_t>(static_cast<uint8_t>(data_[pos_++])) << (8 * i);
    }
    return result;
  }

  std::string_view data_;
  size_t pos_{0};
};

inline void write_varint(std::string& out, uint64_t value) {
  while (value >= 0x80) {
    out.push_back(static_cast<char>((value & 0x7F) | 0x80));
    value >>= 7;
  }
  out.push_back(static_cast<char>(value));
}

inline void write_tag(std::string& out, uint32_t number, WireType wire_type) {
  write_varint(out, (static_cast<uint64_t>(number) << 3) | wire_type);
}

inline void write_varint_field(std::string& out, uint32_t number, uint64_t value) {
  write_tag(out, number, kVarint);
  write_varint(out, value);
}

inline void write_bytes_field(std::string& out, uint32_t number, std::string_view bytes) {
  write_tag(out, number, kLengthDelimited);
  write_varint(out, bytes.size());
  out.append(bytes.data(), bytes.size());
}

}  // namespace bitbots_vision::onnx_wire
