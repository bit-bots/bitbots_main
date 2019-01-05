#!/usr/bin/env python3

import os
import argparse
import yaml
import pickle
import numpy as np
import time
import math
from multiprocessing import Process, Manager


def connect(ends):             
    d = np.diff(ends, axis=0)[0]  
    j = np.argmax(np.abs(d))
    D = d[j]     
    aD = np.abs(D)
    return ends[0] + (np.outer(np.arange(aD + 1), d) + (aD>>1)) // aD

def generate_interpolated_points(point1, point2):
    points = connect(np.array([point2, point1]))
    return set(map(tuple, points))

def get_value_tuple_outer_function(index, tuple_input):
        return (tuple_input[0][index],
                tuple_input[1][index],
                tuple_input[2][index])

def worker(start, stop, result_map, index, distances, interpolation_threshold, main_cluster, colorspace_points):
    interpolated_points = set()
    main_cluster_set = set(main_cluster)
    for main_cluster_index in range(start, stop):
        point = main_cluster[main_cluster_index]
        points = set(np.where(distances[point] < interpolation_threshold)[0].tolist())
        points = list(points.intersection(main_cluster_set))
        for point2 in points:
            line = generate_interpolated_points(get_value_tuple_outer_function(point, colorspace_points), 
                                                get_value_tuple_outer_function(point2, colorspace_points))
            interpolated_points.update(line)

    result_map[index] = interpolated_points


class ColorspaceTool():
    def __init__(self):
        self.colorspace_points = (None,None,None)
        self.point_count = 0
        self.distances = None
        self.distance_threshold = 3
        self.cluster_count_threshold = 3
        self.interpolation_threshold = 8
        self.clusters = []
        self.interpolated_points = []
        self.main_cluster = set()
        self.to_dark = 0
        self.to_bright = 255

    def load_colorspace_from_yaml(self, color_path):
        with open(color_path, 'r') as stream:
            print("Loading file...")
            try:
                color_values = yaml.load(stream)
            except:
                print("Not able to read the File!!!")
            # Compatibility check
            if 'color_values' in color_values.keys():
                color_values = color_values['color_values']['greenField']
            self.point_count = len(color_values['red'])
            if self.point_count == len(color_values['green']) and self.point_count == len(color_values['blue']):
                x = np.array(color_values['red'])
                y = np.array(color_values['green'])
                z = np.array(color_values['blue'])
                self.colorspace_points = (x,y,z)

    def get_value_tuple(self, index):
        return (self.colorspace_points[0][index],
                self.colorspace_points[1][index],
                self.colorspace_points[2][index])

    def get_colorspace_points(self, pointlist):
        return ([self.colorspace_points[0][x] for x in pointlist],
                [self.colorspace_points[1][x] for x in pointlist],
                [self.colorspace_points[2][x] for x in pointlist])

    def all_distances(self):
        points = self.colorspace_points

        red = np.repeat(np.expand_dims(points[0], axis=0),points[0].size, axis=0)
        green = np.repeat(np.expand_dims(points[1], axis=0),points[1].size, axis=0)
        blue = np.repeat(np.expand_dims(points[2], axis=0),points[2].size, axis=0)

        self.distances = np.sqrt(np.square(red - red.transpose()) \
                        + np.square(green - green.transpose()) \
                        + np.square(blue - blue.transpose()))
    
    def cluster(self):
        print("Calculating distances")
        self.all_distances()

        print("Start making sets")
        clusters = self.clusters

        # Generates a set with neighbours for each point
        for row in self.distances:
                clusters.append(set(np.where(row < self.distance_threshold)[0].tolist()))

        print("Merging sets")
        for cluster1 in range(self.point_count):
            for cluster2 in range(self.point_count):
                if clusters[cluster2] is not None and clusters[cluster1] is not None:
                    if not clusters[cluster1].isdisjoint(clusters[cluster2]) and cluster1 != cluster2:
                        clusters[cluster1].update(clusters[cluster2])
                        clusters[cluster2] = None
        # Deletes empty clusters
        clusters = [points for points in clusters if points is not None]
        # Sorts clusters by their size
        clusters.sort(key=len, reverse=True)
        # Builds main set
        for point_set in clusters[0:self.cluster_count_threshold]:
            self.main_cluster.update(point_set)
    
        self.main_cluster = list(self.main_cluster)
        self.clusters = clusters


    def interpolate(self):
        print("Interpolating points...")
        interpolated_points = set()
        if os.cpu_count():
            processes = os.cpu_count()
            print("Running on all {} cores.".format(processes))
        else:
            processes = 1
        length = len(self.main_cluster)
        delta = math.ceil(length/processes)
        manager = Manager()
        result_map = manager.dict()
        jobs = []
        for index in range(processes):
            start = index * delta
            stop = (index + 1) * delta
            if stop > length:
                stop = length
            p = Process(target=worker, args=(start, stop, 
                                            result_map, index,
                                            self.distances, 
                                            self.interpolation_threshold, 
                                            self.main_cluster, 
                                            self.colorspace_points))
            jobs.append(p)
            p.start()

        for proc in jobs:
            proc.join()

        for index in result_map.keys():
            print(index)
            interpolated_points.update(result_map[index])

        main_points = [self.get_value_tuple(index) for index in self.main_cluster]
        interpolated_points.update(main_points)

        print("Finished interpolation!")

        self.interpolated_points = list(interpolated_points)

    def lightness_correction(self):
        points = self.colorspace_points
        lightness_max_value = math.sqrt(3*(255**2))
        deadpool = list()
        for index, point in enumerate(points[0]):
            point = self.get_value_tuple(index)
            lightness = int(math.sqrt(point[0]**2 + point[1]**2 + point[2]**2) * 255 / lightness_max_value)
            if not self.to_dark < lightness < self.to_bright:
                deadpool.append(index)
        self.colorspace_points = (np.delete(points[0], deadpool),
                                  np.delete(points[1], deadpool),
                                  np.delete(points[2], deadpool))
        self.point_count = len(self.colorspace_points[0])

    def plot_values(self, filename):
        point_colors = ['rgb({}, {}, {})'.format(*self.get_value_tuple(index)) for index in range(self.point_count)]
        
        if len(self.interpolated_points) > 0:
            visible = 'legendonly'
        else:
            visible = True

        trace1 = go.Scatter3d(
            x=self.colorspace_points[0],
            y=self.colorspace_points[1],
            z=self.colorspace_points[2],
            name='Color points',
            mode='markers',
            visible=visible,
            marker=dict(
                size=2,
                symbol='circle',
                color=point_colors,
            )
        )
        data = [trace1,]
        if len(self.clusters) > 0:
            points = self.get_colorspace_points(self.main_cluster)
            trace2 = go.Scatter3d(
                x=points[0],
                y=points[1],
                z=points[2],
                name='Cluster',
                mode='markers',
                visible= 'legendonly',
                opacity=0.1,
                marker=dict(
                    size=4,
                    symbol='circle',
                    color='rgb(0,0,0)',
                )
            )
            data.append(trace2)
        if len(self.interpolated_points) > 0:
            point_colors = ['rgb({}, {}, {})'.format(*point) for point in self.interpolated_points]
            x, y, z = zip(*self.interpolated_points)
            trace3 = go.Scatter3d(
                x=x,
                y=y,
                z=z,
                name='Interpolated Points',
                mode='markers',
                marker=dict(
                    size=2,
                    symbol='circle',
                    color=point_colors,
                )
            )
            data.append(trace3)
        layout = go.Layout(
            title='Colorspace {}'.format(filename[7:-5]),
            scene = dict(
                xaxis=dict(
                    range=[0, 255],
                    dtick=20,
                    title='Red',
                    titlefont=dict(
                        family='Courier New, monospace',
                        size=18,
                        color='#e03933'
                    )
                ),
                yaxis=dict(
                    range=[0, 255],
                    dtick=20,
                    title='Green',
                    titlefont=dict(
                        family='Courier New, monospace',
                        size=18,
                        color='#44f441'
                    )
                ),
                zaxis=dict(
                    range=[0, 255],
                    dtick=20,
                    title='Blue',
                    titlefont=dict(
                        family='Courier New, monospace',
                        size=18,
                        color='#4277f4'
                    )
                )
            )
        )
        fig = go.Figure(data=data, layout=layout)
        py.plot(fig, filename=filename, auto_open=True)
    
    def save(self, filename):
        if len(self.interpolated_points) > 0:
            red, green, blue = zip(*self.interpolated_points)
            red = [np.asscalar(x) for x in red]
            green = [np.asscalar(x) for x in green]
            blue = [np.asscalar(x) for x in blue]
            output_type = "interpolated"
            print("Exporting interpolated points")            
        else:
            red, green, blue = self.get_colorspace_points(self.main_cluster)
            output_type = "clustered"
            print("Exporting cluster points")

        data = dict(
            red = red,
            green = green,
            blue = blue
        )

        filename = '{}_{}.txt'.format(filename, output_type)
        with open(filename, 'wb') as outfile:
            pickle.dump(data, outfile, protocol=2)
            # stores data of colorspace in file as pickle for efficient loading (yaml is too slow)
        
        print("Output saved to '{}'.".format(filename))

if __name__ == "__main__":
    tool = ColorspaceTool()
    parser = argparse.ArgumentParser()
    parser.add_argument("-y", "--yaml", help="Input YAML file.")
    parser.add_argument("-o", "--output", help="Saves the output in a YAML file.", action='store_true')
    parser.add_argument("-v", "--visual-output", help="Show graph.", action='store_true')
    parser.add_argument("-nc", "--no-cluster", help="Disables clustering.", action='store_true')
    parser.add_argument("-ni", "--no-interpolation", help="Disables interpolation.", action='store_true')
    parser.add_argument("-dt", "--distance-threshold", help="Point to point distance threshold. Default={}".format(tool.distance_threshold), type=float)
    parser.add_argument("-cc", "--cluster-count", help="Number of used clusters. Default={}".format(tool.cluster_count_threshold), type=int)
    parser.add_argument("-it", "--interpolation-threshold", help="Interpolates a new point to points near other points in this threshold. Default={}".format(tool.interpolation_threshold), type=float)
    parser.add_argument("-mib", "--min-brightness", help="Defines a mimimum brightness for each point. Value is between 0 and 255.", type=int)
    parser.add_argument("-mab", "--max-brightness", help="Defines a maximum brightness for each point. Value is between 0 and 255.", type=int)
    args = parser.parse_args()
    np.warnings.filterwarnings('ignore')

    if args.yaml and os.path.exists(args.yaml):
        tool.load_colorspace_from_yaml(args.yaml)
        name = os.path.basename(args.yaml).split('.')[0]
        if args.min_brightness:
            tool.to_dark = args.min_brightness
        if args.max_brightness:
            tool.to_bright = args.max_brightness
        if args.min_brightness or args.max_brightness:
            tool.lightness_correction()
        if args.distance_threshold:
            tool.distance_threshold = args.distance_threshold
        if args.cluster_count:
            tool.cluster_count_threshold = args.cluster_count
        if not args.no_cluster:
            tool.cluster()
        if args.interpolation_threshold:
            tool.interpolation_threshold = args.interpolation_threshold
        if not args.no_interpolation:
            tool.interpolate()
        if args.visual_output:
            import plotly.offline as py
            import plotly.graph_objs as go
            html_filename = "plotly_{}.html".format(name)
            tool.plot_values(html_filename)
            print("Open '{}' to see the graph".format(html_filename))
        if args.output:
            tool.save(name)
    else:
        print("Path incorrect!")
