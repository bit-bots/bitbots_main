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

#ifndef AUDIO_COMMON__WAVE_FILE_HPP
#define AUDIO_COMMON__WAVE_FILE_HPP

#include <fstream>
#include <string>
#include <vector>

namespace audio_common {

class WaveFile {
public:
  explicit WaveFile(const std::string &filepath);
  ~WaveFile();

  bool open();
  void rewind();
  bool read(std::vector<float> &buffer, size_t size);
  int get_sample_rate() const { return this->sample_rate_; }
  int get_num_channels() const { return this->channels_; }
  int get_bits_per_sample() const { return this->bits_per_sample_; }

private:
  std::string filepath_;
  std::ifstream file_;
  int sample_rate_;
  int channels_;
  int bits_per_sample_;

  // Convert int16_t sample to float [-1.0, 1.0]
  float int16ToFloat(int16_t sample) { return sample / 32768.0f; }
};

} // namespace audio_common

#endif // WAVE_FILE_HPP
