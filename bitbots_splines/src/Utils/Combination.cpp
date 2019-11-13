/*
This code is largely based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/
#include <limits>
#include <stdexcept>
#include "bitbots_splines/Combination.hpp"

namespace bitbots_splines {

unsigned long Combination::binomialCoefficient(
    size_t k, size_t n) {
  if (n == 0 || k == 0) {
    return 1;
  }
  if (k > n) {
    throw std::logic_error("Combination not valid k>n");
  }

  if (n - k < k) {
    return binomialCoefficient(n - k, n);
  }
  if (k == 1 || k == n) {
    return n;
  }

  Pair pair(k, n);
  if (pascal_triangle_.count(pair) == 0) {
    unsigned long val_1 = binomialCoefficient(k - 1, n - 1);
    unsigned long val_2 = binomialCoefficient(k, n - 1);
    unsigned long test =
        std::numeric_limits<unsigned long>::max()
            - val_1;
    if (val_2 < test) {
      pascal_triangle_[pair] = val_1 + val_2;
    } else {
      throw std::runtime_error("Combination overflow");
    }
  }

  return pascal_triangle_[pair];
}

void Combination::startCombination(size_t k, size_t n) {
  if (n == 0 || k == 0) {
    throw std::logic_error("Combination zero");
  }
  if (k > n) {
    throw std::logic_error("Combination not valid k>n");
  }

  indexes_ = std::vector<size_t>();
  k_ = k;
  n_ = n;
  for (size_t i = 0; i < k; i++) {
    indexes_.push_back(i);
  }
}

std::vector<size_t> Combination::nextCombination() {
  std::vector<size_t> result = indexes_;

  if (!indexes_.empty()) {
    bool is_end = incrIndexes(k_ - 1);
    if (is_end) {
      indexes_.clear();
    }
  }

  return result;
}

bool Combination::incrIndexes(size_t i) {
  if (indexes_[i] == n_ - (k_ - i)) {
    if (i == 0) {
      return true;
    } else {
      bool is_end = incrIndexes(i - 1);
      if (is_end) {
        return true;
      } else {
        indexes_[i] = indexes_[i - 1] + 1;
        return false;
      }
    }
  } else {
    indexes_[i]++;
    return false;
  }
}

}

