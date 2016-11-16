import math


class KMeans():

    def __init__(self, points, centers):
        self.points = points
        self.centers = centers
        self.old_centers = None

    def compute(self):
        self.compute_one_step()
        steps = 1

        while self.centers_still_move():
            steps +=1
            self.compute_one_step()

        return self.centers

    def compute_one_step(self):
        clusters = self.compute_cluster_assignments()
        avrg = self.compute_average_in_cluster(clusters)
        new_centers = []
        for i in range(len(self.centers)):
            new_centers.append((avrg[i][0], avrg[i][1]))
        self.old_centers = self.centers
        self.centers = new_centers

    def compute_cluster_assignments(self):
        clusters = {i: [] for i in range(len(self.centers))}

        for i in range(len(self.points)):
            p = self.points[i]
            distances = [self.distance(e, p) for e in self.centers]
            minimum_index = distances.index(min(distances))
            clusters[minimum_index].append(p)
        return clusters

    def distance(self, e, p):
        return math.sqrt((e[0] - p[0])**2 + (e[1] - p[1])**2)

    def compute_average_in_cluster(self, clusters):
        avrgs = {}
        for i in clusters.keys():
            c = clusters[i]
            l = len(c)
            if l != 0:
                x_av = sum([e[0] for e in c]) / float(l)
                y_av = sum([e[1] for e in c]) / float(l)
                avrgs[i] = [x_av, y_av]
            else:
                avrgs[i] = (self.centers[i][0], self.centers[i][1])
        return avrgs

    def centers_still_move(self):
        for i in range(len(self.centers)):
            old = self.old_centers[i]
            current = self.centers[i]

            if abs(old[0] - current[0]) > 1E-5 and abs(old[1] - current[1]) > 1E-5:
                return True