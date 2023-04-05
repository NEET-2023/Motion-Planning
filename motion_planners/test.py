import numpy as np

def distance(point1, point2):
        return np.linalg.norm(point2 - point1)

def minimum_distance(point):
        def helper(segment):
            start, end = np.array([segment[0], segment[1]]), np.array([segment[2], segment[3]])

            # Return minimum distance between line segment (start-end) and point p
            l2 = distance(start, end)**2
            if (l2 == 0.0):
                return distance(point, start)


            # v = start, w = end
            # Consider the line extending the segment, parameterized as v + t (w - v).
            # We find projection of point p onto the line.
            # It falls where t = [(p-v) . (w-v)] / |w-v|^2
            # We clamp t from [0,1] to handle points outside the segment vw.

            t = max(0, min(1, np.dot(point - start, end - start) / l2))
            projection = start + t * (end - start)  # Projection falls on the segment
            return distance(point, projection)
        return helper

if __name__ == '__main__':
    point = np.array([5, 5])
    path = np.array([(i, 2*i) for i in range(10)])
    path_segments = np.array([[path[i][0], path[i][1], path[i+1][0], path[i+1][1]] for i in range(len(path) - 1)])
    min_distances = np.apply_along_axis(minimum_distance(point), 1, path_segments)
    min_index = np.argmin(min_distances)
    forward_path = path_segments[min_index:][::-1]
    desired_point = path_segments[min_index][2:]
    print(point)
    print(path)
    print(min_distances)
    print(min_index)
    print(desired_point)