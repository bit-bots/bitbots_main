#include<algorithm>
#include<fstream>
#include<iostream>
#include<vector>
#include<unordered_set>
#include<yaml-cpp/yaml.h>
#include<cmath>

using namespace std;

class Point {
public:
    Point(int r, int g, int b):
        _r(r), _g(g), _b(b) {}
    int _r, _g, _b;
    double distance(const Point& p);
    bool comp_lg(const Point& p);
};

double Point::distance(const Point& p) {
    return sqrt(pow(p._r - _r, 2) + pow(p._g - _g, 2) + pow(p._b - _b, 2));
}

bool Point::comp_lg(const Point& p) {
    // Component wise larger, true if self > p
    return _r > p._r && _g > p._g && _b > p._g;
}

bool operator<(const Point& p1, const Point& p2) {
    if (p1._r != p2._r) return p1._r < p2._r;
    if (p1._g != p2._g) return p1._g < p2._g;
    return p1._b < p2._b;
}

ostream& operator<<(ostream& os, const Point& p) {
    os << "{" << p._r << " " << p._g << " " << p._b << "} ";
    return os;
}

bool operator==(const Point& p1, const Point& p2) {
    return p1._r == p2._r && p1._g == p2._g && p1._b == p2._b;
}

namespace std {
    template<> struct hash<Point> {
        typedef Point argument_type;
        typedef std::size_t result_type;
        result_type operator()(argument_type const& p) const noexcept {
            return p._r*256*256 + p._g*256 + p._b;
        }
    };
}

class PointDifference {
public:
    PointDifference(int r_, int g_, int b_):
        r(r_), g(g_), b(b_) {}
    int r,b,g;
};

PointDifference operator-(const Point& p1, const Point& p2) {
    PointDifference p(p1._r - p2._r, p1._g - p2._g, p1._b - p2._b);
    return p;
}

struct Cluster {
    list<int> point_indices;
    Point min;
    Point max;
};

class ColorspaceTool {
public:
    ColorspaceTool(double distance_threshold, int cluster_count, double interpolation_threshold);
    void load_colorspace_from_yaml(string filename);
    void interpolate();
    void save(string filename);
    void cluster();
private:
    vector<int> get_value_tuple(int index);
    vector<int> get_colorspace_points(vector<int>);
    void generate_interpolated_points(Point p1, Point p2);
    void all_distances();
    void build_main_cluster();
    void merge_clusters();
    double get_distance(int p_index1, int p_index2);

    vector<Point> _points;
    double _distance_threshold;
    int _cluster_count;
    double _interpolation_threshold;
    vector<vector<double>> _distances;
    vector<Cluster> _clusters;
    list<int> _main_cluster;
    unordered_set<Point> _interpolated_points;
};

ColorspaceTool::ColorspaceTool(double distance_threshold, 
                               int cluster_count,
                               double interpolation_threshold):
    _distance_threshold(distance_threshold),
    _cluster_count(cluster_count),
    _interpolation_threshold(interpolation_threshold) {}

void ColorspaceTool::load_colorspace_from_yaml(string filename) {
    cout << "Loading from " << filename << endl;
    ifstream file(filename);
    if (file.is_open()) {
        YAML::Node colorfile = YAML::Load(file);
        file.close();
        YAML::Node content = colorfile["color_values"]["greenField"];
        YAML::Node yred = content["red"];
        YAML::Node ygreen = content["green"];
        YAML::Node yblue = content["blue"];
        for (int i = 0; i < yred.size(); ++i) {
            int r = yred[i].as<int>();
            int g = ygreen[i].as<int>();
            int b = yblue[i].as<int>();
            Point p(r, g, b);
            _points.push_back(p);
        }
    }
    cout << _points.size() << endl;
}

double ColorspaceTool::get_distance(int p_index1, int p_index2) {
    return _distances[p_index2][p_index1];
}

void ColorspaceTool::all_distances() {
    _distances = vector<vector<double>>(_points.size());
    fill(_distances.begin(), _distances.end(), vector<double>(_points.size()));
    for (int i = 0; i < _points.size(); ++i) {
        for (int j = 0; j < i; ++j) {
            double d = _points[i].distance(_points[j]);
            _distances[i][j] = d;
            _distances[j][i] = d;
        }
    }
}

void ColorspaceTool::cluster() {
    cout << "Calculating distances" << endl;
    all_distances();
    cout << "Start making sets" << endl;
    for (int i = 0; i < _distances.size(); ++i) {
        list<int> threshold_distance_indices;
        int rmin = 255, gmin = 255, bmin = 255,
            rmax = 0, gmax = 0, bmax = 0;
        for (int j = 0; j < _distances.size(); ++j) {
            if (get_distance(i, j) < _distance_threshold) {
                threshold_distance_indices.push_back(j);
                if (_points[j]._r < rmin) rmin = _points[j]._r;
                if (_points[j]._r > rmax) rmax = _points[j]._r;
                if (_points[j]._g < gmin) gmin = _points[j]._g;
                if (_points[j]._g > gmax) gmax = _points[j]._g;
                if (_points[j]._b < bmin) bmin = _points[j]._b;
                if (_points[j]._b > bmax) bmax = _points[j]._b;
            }
        }
        Point min(rmin, gmin, bmin), max(rmax, gmax, bmax);
        Cluster c = {threshold_distance_indices, min, max};
        _clusters.push_back(c);
    }
    merge_clusters();
    build_main_cluster();
}

void ColorspaceTool::merge_clusters() {
    cout << "Merging sets" << endl;
    for (auto it1 = _clusters.begin(); it1 != _clusters.end(); ++it1) {
        for (auto it2 = _clusters.begin(); it2 != _clusters.end(); ++it2) {
            // Merging only necessary when cluster1.max > cluster2.min
            if (!it1->point_indices.empty() && !it2->point_indices.empty() && it1 != it2
                    && !(it1->min.comp_lg(it2->max) && it2->min.comp_lg(it1->max))) {
                for (auto& element: it2->point_indices) {
                    // Lists are sorted, binary search can be used
                    if (binary_search(it1->point_indices.begin(),
                                      it1->point_indices.end(),
                                      element)) {
                        it1->point_indices.merge(it2->point_indices);
                        it1->point_indices.unique();
                        break;
                    }
                }
            }
        }
    }
}

void ColorspaceTool::build_main_cluster() {
    // Sort clusters descending by size
    std::sort(_clusters.begin(), _clusters.end(), [&](Cluster c1, Cluster c2) { return c1.point_indices.size() > c2.point_indices.size(); });
    // Build main cluster
    int i = 0;
    auto it = _clusters.begin();
    while (i < _cluster_count && it != _clusters.end()) {
        _main_cluster.merge(it->point_indices);
        i++;
        it++;
    }
    _main_cluster.unique();
}

void ColorspaceTool::interpolate() {
    cout << "Interpolating points" << endl;
    for (auto outer_index = _main_cluster.begin(); outer_index != _main_cluster.end(); ++outer_index) {
        for (auto inner_index = next(outer_index); inner_index != _main_cluster.end(); ++inner_index) {
            if (get_distance(*outer_index, *inner_index) < _interpolation_threshold) {
                generate_interpolated_points(_points[*outer_index], _points[*inner_index]);
            }
        }
    }
}

void ColorspaceTool::generate_interpolated_points(Point p1, Point p2) {
    if (p1 < p2) std::swap(p1, p2);
    PointDifference difference = p1 - p2;
    int max = difference.r;
    if (difference.g > max) max = difference.g;
    if (difference.b > max) max = difference.b;
    for (int i = 0; i <= max; ++i) {
        int r = p2._r + int(float(difference.r)/max * i);
        int g = p2._g + int(float(difference.g)/max * i);
        int b = p2._b + int(float(difference.b)/max * i);
        Point p(r,g,b);
        _interpolated_points.insert(p);
    }
}

void ColorspaceTool::save(string filename) {
    cout << "Saving to " << filename << endl;
    ofstream file(filename);
    std::vector<int> red, blue, green;
    for (auto point: _interpolated_points) {
        red.push_back(point._r);
        green.push_back(point._g);
        blue.push_back(point._b);
    }
    YAML::Emitter out;
    out << YAML::BeginMap << YAML::Key << "color_values" << YAML::Value
        << YAML::BeginMap << YAML::Key << "greenField" << YAML::Value
        << YAML::BeginMap << YAML::Key << "red"
        << YAML::Value << YAML::Flow << YAML::BeginSeq;
    for (auto i: red) {
        out << i;
    }
    out << YAML::EndSeq << YAML::Block << YAML::Key << "green"
        << YAML::Value << YAML::Flow << YAML::BeginSeq;
    for (auto i: green) {
        out << i;
    }
    out << YAML::EndSeq << YAML::Block << YAML::Key << "blue"
        << YAML::Value << YAML::Flow << YAML::BeginSeq;
    for (auto i: blue) {
        out << i;
    }
    out << YAML::EndSeq << YAML::Flow << YAML::EndMap << YAML::EndMap << YAML::EndMap;
    file << out.c_str();
}

int main(int argc, char* argv[]) {
    if (argc == 1) {
        cerr << "Usage: " << argv[0] << " colorspace.yaml [-it interpolation-threshold] [-cc cluster-count] [-ct cluster-threshold] [-o out.yaml]" << endl;
        return 1;
    }
    string filename, output = "out.yaml";
    double interpolation_threshold = 10.0, cluster_threshold = 3.0;
    int cluster_count = 3;
    for (int i = 1; i < argc; ++i) {
        string arg = string(argv[i]);
        if (arg == "-o") {
            i++;
            output = argv[i];
        } else if (arg == "-it") {
            i++;
            interpolation_threshold = stod(argv[i]);
        } else if (arg == "-cc") {
            i++;
            cluster_count = stoi(argv[i]);
        } else if (arg == "-ct") {
            i++;
            cluster_threshold = stod(argv[i]);
        } else {
            filename = argv[i];
        }
    }
    if (filename.empty()) {
        cerr << "Please specify an input file" << endl;
        return 1;
    }
    ColorspaceTool tool(cluster_threshold, cluster_count, interpolation_threshold);
    tool.load_colorspace_from_yaml(filename);
    tool.cluster();
    tool.interpolate();
    tool.save(output);
}
