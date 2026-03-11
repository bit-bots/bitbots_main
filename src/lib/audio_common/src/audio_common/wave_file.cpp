// MIT License
//
// Copyright (c) 2024 Miguel Ángel González Santamarta
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <cstring>
#include <iostream>

#include "audio_common/wave_file.hpp"

using namespace audio_common;

WaveFile::WaveFile(const std::string &filepath)
    : filepath_(filepath), sample_rate_(0), channels_(0), bits_per_sample_(0) {}

WaveFile::~WaveFile() { this->file_.close(); }

bool WaveFile::open() {
  this->file_.open(this->filepath_, std::ios::binary);
  if (!this->file_.is_open()) {
    std::cerr << "Failed to open file: " << this->filepath_ << std::endl;
    return false;
  }
  // Read the WAV header
  char riff_header[4];
  this->file_.read(riff_header, 4);
  if (std::strncmp(riff_header, "RIFF", 4) != 0) {
    std::cerr << "Invalid WAV file" << std::endl;
    return false;
  }
  this->file_.seekg(24);
  this->file_.read(reinterpret_cast<char *>(&this->sample_rate_), 4);
  this->file_.seekg(22);
  this->file_.read(reinterpret_cast<char *>(&this->channels_), 2);
  this->file_.seekg(34);
  this->file_.read(reinterpret_cast<char *>(&this->bits_per_sample_), 2);
  this->file_.seekg(44); // Move to the data section start
  return true;
}

void WaveFile::rewind() {
  this->file_.close();
  this->open();
}

bool WaveFile::read(std::vector<float> &buffer, size_t size) {
  if (this->bits_per_sample_ != 16) {
    std::cerr << "Only 16-bit PCM WAV files are supported" << std::endl;
    return false;
  }

  // Allocate temporary buffer to read int16 data
  std::vector<int16_t> temp_buffer(size * this->channels_);

  // Read raw int16 samples from the file
  this->file_.read(reinterpret_cast<char *>(temp_buffer.data()),
                   size * this->channels_ * sizeof(int16_t));
  if (this->file_.gcount() !=
      static_cast<std::streamsize>(size * this->channels_ * sizeof(int16_t))) {
    return false; // End of file or read error
  }

  // Convert int16 samples to float and store in buffer
  buffer.resize(size * this->channels_);
  for (size_t i = 0; i < size * this->channels_; ++i) {
    buffer[i] = int16ToFloat(temp_buffer[i]);
  }

  return true;
}