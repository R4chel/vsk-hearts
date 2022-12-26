import vsketch
from shapely.geometry import Point, LinearRing, LineString
import vpype as vp
import numpy as np
from point2d import Point2D
import bisect


class BoundingCircle:

    def __init__(self, p, r, layer, rotation):
        self.p = p
        self.r = r
        self.rotation = rotation
        self.layer = layer

    def draw(self, vsk: vsketch.SketchClass):
        vsk.circle(self.p.x, self.p.y, self.r, mode="radius")

    def draw_spiral(self, vsk: vsketch.SketchClass, a, b, f, g, num_points,
                    increments):
        thetas = [i * 2 * np.pi / increments for i in range(num_points)]
        pts = [
            Point2D(a=theta,
                    r=((a + b * theta) * (1 + f * np.sin(g * theta) * self.r)))
            for theta in thetas
        ]
        points = [Point((p + self.p).cartesian()) for p in pts]
        vsk.stroke(self.layer)
        vsk.geometry(LineString(points))


class VskHeartsSketch(vsketch.SketchClass):
    # Sketch parameters:
    num_shapes = vsketch.Param(5)
    a = vsketch.Param(1.)
    b = vsketch.Param(2.)
    f = vsketch.Param(1.)
    g = vsketch.Param(2.)
    num_points = vsketch.Param(200)
    increments = vsketch.Param(10)
    min_radius_ratio = vsketch.Param(0.05)
    max_radius_ratio = vsketch.Param(0.2)
    max_attempts = vsketch.Param(1000)
    min_rotation = vsketch.Param(np.pi - np.pi / 3)
    max_rotation = vsketch.Param(np.pi + np.pi / 3)
    num_layers = vsketch.Param(1)

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

        ## draw shapes
        for c in circles:
            c.draw_spiral(vsk, self.a, self.b, self.f, self.g, self.num_points,
                          self.increments)

    def finalize(self, vsk: vsketch.Vsketch) -> None:
        vsk.vpype("linemerge linesimplify reloop linesort")


if __name__ == "__main__":
    VskHeartsSketch.display()
