import vsketch
from shapely.geometry import Point, LinearRing
import vpype as vp
import numpy as np
from point2d import Point2D
import bisect


def heart_f(t):
    return np.sin(t) * np.sqrt(np.abs(
        np.cos(t))) / (np.sin(t) + 7 / 5) - 2 * np.sin(t) + 2


def heart_pts(num_points):
    thetas = [i * 2 * np.pi / num_points for i in range(num_points)]
    return [Point2D(a=theta, r=heart_f(theta)) for theta in thetas]


def star_pts(n):
    return [
        Point2D(a=(i * np.pi / n), r=(1 if i % 2 == 0 else 0.5))
        for i in range(n * 2)
    ]


class BoundingCircle:

    def __init__(self, p, r, layer, rotation):
        self.p = p
        self.r = r
        self.rotation = rotation
        self.layer = layer

    def draw(self, vsk: vsketch.SketchClass):
        vsk.circle(self.p.x, self.p.y, self.r, mode="radius")

    def draw_heart(self, vsk: vsketch.SketchClass, heart_points):
        heart_r = self.r / 2.5
        heart_center = Point2D(self.p.x, self.p.y + 1.42 * heart_r)
        bounding_center = Point2D(self.p.x, self.p.y)
        diff = heart_center - bounding_center
        diff.a += self.rotation
        points = [point * heart_r for point in heart_points]
        for p in points:
            p.a += self.rotation
        points = [
            Point((p + bounding_center + diff).cartesian()) for p in points
        ]
        vsk.stroke(self.layer)
        vsk.geometry(LinearRing(points))

    def draw_star(self, vsk: vsketch.SketchClass, star_points):
        pts = [p * self.r for p in star_points]
        for p in pts:
            p.a += self.rotation
        points = [Point((p + self.p).cartesian()) for p in pts]
        vsk.stroke(self.layer)
        vsk.geometry(LinearRing(points))


class VskHeartsSketch(vsketch.SketchClass):
    # Sketch parameters:
    num_shapes = vsketch.Param(5)
    min_radius_ratio = vsketch.Param(0.05)
    max_radius_ratio = vsketch.Param(0.2)
    max_attempts = vsketch.Param(1000)
    min_rotation = vsketch.Param(np.pi - np.pi / 3)
    max_rotation = vsketch.Param(np.pi + np.pi / 3)
    num_points = vsketch.Param(500)
    num_layers = vsketch.Param(1)
    min_star_points = vsketch.Param(8)
    max_star_points = vsketch.Param(8)

    def max_radius_at_p(self, vsk: vsketch.SketchClass,
                        circles: list[BoundingCircle], point: Point):
        max_r = min([
            self.max_radius, point.x, self.width - point.x, point.y,
            self.height - point.y
        ])

        for c in circles:
            d = (point - c.p).r - c.r
            max_r = min(d, max_r)
            if max_r < self.min_radius:
                break
        if max_r > self.min_radius:
            return max_r
        else:
            return None

    def random_point(self, vsk: vsketch.Vsketch):
        return Point2D(vsk.random(0, self.width), vsk.random(0, self.height))

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

        layers = range(1, self.num_layers + 1)
        circles = []
        attempts = 0

        ## create circles
        while len(circles) < self.num_shapes and attempts < self.max_attempts:
            p = self.random_point(vsk)
            maybe_r = self.max_radius_at_p(vsk, circles, p)
            if maybe_r is not None:
                layer = layers[int(vsk.random(0, 1) * len(layers))]
                c = BoundingCircle(
                    p, maybe_r, layer,
                    vsk.random(self.min_rotation, self.max_rotation))

                # keep list of circles sorted by decreasing radius for circle packing performace
                bisect.insort(circles,
                              c,
                              lo=len(circles),
                              hi=0,
                              key=lambda c: c.r)
                attempts = 0
            else:
                attempts += 1

        print("num circles:", len(circles))
        heart_points = heart_pts(self.num_points)
        ## draw shapes
        for c in circles:
            # c.draw(vsk)
            # c.draw_heart(vsk, heart_points)
            num_star_points = int(
                vsk.random(self.min_star_points, self.max_star_points + 1))
            star_points = star_pts(num_star_points)
            c.draw_star(vsk, star_points)

    def finalize(self, vsk: vsketch.Vsketch) -> None:
        vsk.vpype("linemerge linesimplify reloop linesort")


if __name__ == "__main__":
    VskHeartsSketch.display()
