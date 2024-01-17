/*
This code is largely based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/
#ifndef BITBOTS_SPLINES_INCLUDE_BITBOTS_SPLINES_SPLINE_CONTAINER_H_
#define BITBOTS_SPLINES_INCLUDE_BITBOTS_SPLINES_SPLINE_CONTAINER_H_

#include <string>
#include <map>
#include <stdexcept>
#include <fstream>
#include "spline.hpp"
#include <set>
#include <algorithm>
#include <vector>
#include <bitbots_splines/smooth_spline.h>

namespace bitbots_splines {

/**
 * SplineContainer
 *
 * Wrapper for map of generic splines
 * types indexed by string name
 * Implementation of implort/export from files
 */
template<class T>
class SplineContainer {
 public:

  /**
   * Return the number of contained splines
   */
  inline size_t size() const {
    return container_.size();
  }

  /**
   * Add an empty spline with given name.
   * Variadic arguments allow to pass parameters to
   * spline constructor.
   */
  template<typename ... Args>
  inline void add(const std::string &name, Args... args) {
    if (container_.count(name) != 0) {
      throw std::logic_error(
          "SplineContainer spline already added");
    }
    container_[name] = T(args...);
  }

  /**
   * Return true if given spline
   * name is contained
   */
  inline bool exist(const std::string &name) const {
    return container_.count(name) > 0;
  }

  /**
   * Access to given named spline
   */
  inline const T &get(const std::string &name) const {
    if (container_.count(name) == 0) {
      throw std::logic_error(
          "SplineContainer invalid name: " + name);
    }
    return container_.at(name);
  }
  inline T &get(const std::string &name) {
    if (container_.count(name) == 0) {
      throw std::logic_error(
          "SplineContainer invalid name: " + name);
    }
    return container_.at(name);
  }

  /**
   * Access to internal map container
   */
  const std::map<std::string, T> &get() const {
    return container_;
  }
  std::map<std::string, T> &get() {
    return container_;
  }

  /**
   * Returns all time points where a point in any spline exists.
   */
  std::vector<double> getTimes() const {
    std::set<double> times;
    std::vector<double> times_sorted;
    // go trough all splines
    for (const auto &sp : container_) {
      // go trough all points of the spline
      for (const SmoothSpline::Point &point : sp.second.points()) {
        times.insert(point.time);
      }
    }
    //insert set into vector
    times_sorted.insert(times_sorted.end(), times.begin(), times.end());
    std::sort(times_sorted.begin(), times_sorted.end());
    return times_sorted;
  }

  /**
   * Return minimum and maximum abscisse values
   * of all registered splines parts
   */
  double min() const {
    if (container_.size() == 0) {
      return 0.0;
    }
    bool is_first = true;
    double m = 0.0;
    for (const auto &sp : container_) {
      if (is_first || m > sp.second.min()) {
        m = sp.second.min();
        is_first = false;
      }
    }
    return m;
  }
  double max() const {
    if (container_.size() == 0) {
      return 0.0;
    }
    bool is_first = true;
    double m = 0.0;
    for (const auto &sp : container_) {
      if (is_first || m < sp.second.max()) {
        m = sp.second.max();
        is_first = false;
      }
    }
    return m;
  }

  /**
   * Export to and Import from given file name
   * in "spline" CSV format prefixed with spline name
   */
  void exportData(const std::string &file_name) const {
    if (container_.size() == 0) {
      throw std::logic_error("SplineContainer empty");
    }

    std::ofstream file(file_name);
    if (!file.is_open()) {
      throw std::runtime_error(
          "SplineContainer unable to write file: "
              + file_name);
    }

    for (const auto &sp : container_) {
      file << "'" << sp.first << "' ";
      sp.second.exportData(file);
    }

    file.close();
  }
  void importData(const std::string &file_name) {
    std::ifstream file(file_name);
    if (!file.is_open()) {
      throw std::runtime_error(
          "SplineContainer unable to read file: "
              + file_name);
    }

    bool is_parse_error;
    while (file.good()) {
      is_parse_error = true;
      //Skip name delimitor
      if (file.peek() != '\'') break;
      file.ignore();
      if (!file.good()) break;
      //Parse spline name
      char name[256];
      file.getline(name, 256, '\'');
      //Import founded spline
      add(std::string(name));
      if (!file.good()) break;
      container_.at(std::string(name)).importData(file);
      is_parse_error = false;
      //Skip end line
      while (file.peek() == ' ' || file.peek() == '\n') {
        if (!file.good()) break;
        file.ignore();
      }
    }
    if (is_parse_error) {
      throw std::logic_error(
          "SplineContainer invalid input format");
    }

    file.close();
  }

 private:

  /**
   * Spline container indexed
   * by their name
   */
  std::map<std::string, T> container_;
};

}

#endif

