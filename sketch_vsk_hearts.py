import vsketch
from shapely.geometry import Point, LinearRing
import vpype as vp
import numpy as np


class PolarPoint:

    def __init__(self, theta, r):
        self.theta = theta
        self.r = r

    def to_scaled_cartesian(self, r_scalar):
        new_r = self.r * r_scalar
        return Point(new_r * np.cos(self.theta), new_r * np.sin(self.theta))


def heart_f(t):
    return np.sin(t) * np.sqrt(np.abs(
        np.cos(t))) / (np.sin(t) + 7 / 5) - 2 * np.sin(t) + 2


def heart_pts(num_points):
    thetas = [i * 2 * np.pi / num_points for i in range(num_points)]
    return [PolarPoint(theta, heart_f(theta)) for theta in thetas]


class BoundingCircle:

    def __init__(self, p, r, rotation):
        self.p = p
        self.r = r
        self.rotation = rotation

    def draw(self, vsk: vsketch.SketchClass):
        vsk.circle(self.p.x, self.p.y, self.r, mode="radius")

    # rotation is wrong
    def draw_heart(self, vsk: vsketch.SketchClass, heart_points):
        heart_r = self.r / 2.5
        heart_p = Point(self.p.x, self.p.y + 1.42 * heart_r)
        pts = [
            Point(heart_r * pp.r * np.cos(pp.theta + self.rotation) + heart_p.x,
                  heart_r * pp.r * np.sin(pp.theta + self.rotation) + heart_p.y)
            for pp in heart_points
        ]
        vsk.geometry(LinearRing(pts))


class VskHeartsSketch(vsketch.SketchClass):
    # Sketch parameters:
    num_shapes = vsketch.Param(5)
    min_radius_ratio = vsketch.Param(0.05)
    max_radius_ratio = vsketch.Param(0.2)
    max_attempts = vsketch.Param(1000)
    min_rotation = vsketch.Param(np.pi - np.pi/3)
    max_rotation = vsketch.Param(np.pi + np.pi/3)
    num_points = vsketch.Param(500)

    def max_radius_at_p(self, vsk: vsketch.SketchClass,
                        circles: list[BoundingCircle], point: Point):
        max_r = min([
            self.max_radius, point.x, self.width - point.x, point.y,
            self.height - point.y
        ])

        # opportunity to improve performance by sorting circles by radius
        for c in circles:
            d = point.distance(c.p) - c.r
            max_r = min(d, max_r)
            if max_r < self.min_radius:
                break
        if max_r > self.min_radius:
            return max_r
        else:
            return None

    def random_point(self, vsk: vsketch.Vsketch):
        return Point(vsk.random(0, self.width), vsk.random(0, self.height))

    def draw(self, vsk: vsketch.Vsketch) -> None:
        vsk.size("a6", landscape=True)
        scale = "mm"
        vsk.scale(scale)
        factor = 1 / vp.convert_length(scale)
        self.width, self.height = factor * vsk.width, factor * vsk.height

        self.max_radius = min(self.width,
                              self.height) * self.max_radius_ratio / 2
        self.min_radius = min(self.width,
                              self.height) * self.min_radius_ratio / 2

        # implement your sketch here

        circles = []
        attempts = 0

        ## create circles
        while len(circles) < self.num_shapes and attempts < self.max_attempts:
            p = self.random_point(vsk)
            maybe_r = self.max_radius_at_p(vsk, circles, p)
            if maybe_r is not None:
                c = BoundingCircle(p, maybe_r, vsk.random(self.min_rotation,self.max_rotation))
                circles.append(c)
                attempts = 0
            else:
                attempts += 1

        heart_points = heart_pts(self.num_points)
        ## draw shapes
        for c in circles:
            c.draw_heart(vsk, heart_points)

    def finalize(self, vsk: vsketch.Vsketch) -> None:
        vsk.vpype("linemerge linesimplify reloop linesort")


if __name__ == "__main__":
    VskHeartsSketch.display()
