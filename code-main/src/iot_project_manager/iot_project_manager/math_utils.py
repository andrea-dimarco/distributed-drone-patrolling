import math

# Local version of the math functions used by the tester.
# Modifications to the project_solution won't affect this copy

def point_distance(p0, p1):

    '''
    Computes the distance between the two given points.
    p0: the coordinates of the first point (x0, y0)
    p1: the coordinates of the second point (x1, y1)
    '''

    vb = (p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2])
    return math.sqrt(vb[0]**2 + vb[1]**2 + vb[2]**2)
