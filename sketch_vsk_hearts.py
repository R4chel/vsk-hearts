import vsketch
from shapely.geometry import Point
import vpype as vp

class Circle:
    def __init__(self, x,y,r):
        self.x = x
        self.y = y
        self.r = r

class CirlcePacking:
    def __init__(self):
        self.circles = []

    def new_circle(self, vsk):
        pass

class VskHeartsSketch(vsketch.SketchClass):
    # Sketch parameters:
    num_shapes = vsketch.Param(5)

    def random_point(self, vsk:vsketch.Vsketch):
        return Point(vsk.random(0, self.width), vsk.random(0, self.height))
    
    def draw(self, vsk: vsketch.Vsketch) -> None:
        vsk.size("a6", landscape=True)
        vsk.scale("px")
        scale = "mm"
        vsk.scale(scale)
        factor = 1 / vp.convert_length(scale)
        self.width, self.height = factor * vsk.width, factor * vsk.height

        # implement your sketch here
        for i in range(self.num_shapes):
            p = self.random_point(vsk)
            vsk.circle(p.x,p.y, i)

    def finalize(self, vsk: vsketch.Vsketch) -> None:
        vsk.vpype("linemerge linesimplify reloop linesort")


if __name__ == "__main__":
    VskHeartsSketch.display()
