import vsketch
from shapely.geometry import Point
import vpype as vp


class BoundingCircle:
    def __init__(self, p, r):
        self.p = p
        self.r = r

    def draw(self, vsk: vsketch.SketchClass):
        vsk.circle(self.p.x, self.p.y, self.r, mode="radius")


class VskHeartsSketch(vsketch.SketchClass):
    # Sketch parameters:
    num_shapes = vsketch.Param(5)
    min_radius_ratio = vsketch.Param(0.05)
    max_radius_ratio = vsketch.Param(0.2)
    max_attempts = vsketch.Param(1000)

    def max_radius_at_p(self, vsk: vsketch.SketchClass,
                   circles: list[BoundingCircle], point: Point):
        max_r = min([
            self.max_radius, point.x, vsk.width -
            point.x, point.y, vsk.height - point.y
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

        self.max_radius = min(self.width, self.height) * self.max_radius_ratio / 2
        self.min_radius = min(self.width, self.height) * self.min_radius_ratio / 2


        # implement your sketch here

        circles = []
        attempts = 0

        ## create circles
        while len(circles) < self.num_shapes and attempts < self.max_attempts:
            p = self.random_point(vsk)
            maybe_r = self.max_radius_at_p(vsk, circles, p)
            if maybe_r is not None:
                c = BoundingCircle(p, maybe_r)
                circles.append(c)
                attempts = 0
            else:
                attempts += 1

        ## draw shapes
        for c in circles:
            c.draw(vsk)

    def finalize(self, vsk: vsketch.Vsketch) -> None:
        vsk.vpype("linemerge linesimplify reloop linesort")


if __name__ == "__main__":
    VskHeartsSketch.display()
