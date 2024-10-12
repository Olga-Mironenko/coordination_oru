"""
E.g.::

    (  # the main horizontal line (branch)
        ()*6  # simple vertical lines (rays)
        (     # complex vertical line (branch)
            ()*2  # simple horizontal lines (rays)
            (
                ()*4  # simple vertical lines (rays)
            )
        )
    )
"""
import math
import pathlib
import random
import tempfile
import turtle
from typing import Optional

from PIL import Image

LENGTH_STEP = 50
WIDTH_PEN = LENGTH_STEP // 2
assert WIDTH_PEN < LENGTH_STEP  # for a gap between rays

I_RAY_BRANCH_ZERO_OP = 2
LENGTH_OP = LENGTH_STEP
LENGTH_RAY_MIN = 1  # or LENGTH_STEP

# Note: GIF can have only 256 colors, so they may be distorted (not pure black in particular;
# see https://docs.gimp.org/2.10/en/gimp-stuck-export-gif-colors-changed.html).
FILENAME_OBSTACLES_PNG = 'obstacles.png'
WIDTH_GAP_IMAGE_CANVAS = 100
assert WIDTH_GAP_IMAGE_CANVAS % 2 == 0
WIDTH_GAP_RAY_OBSTACLE = WIDTH_PEN // 2 + 10

X_WINDOW_START = 0
Y_WINDOW_START = 0

PROBABILITY_BRIDGE_PRESENCE = 0.5
PROBABILITY_BRIDGE_SINGLE = 0.5


class Tree:
    @classmethod
    def generate(cls, *, is_last):
        branches = [0] * random.randint(2, 4)
        for i in range(0, len(branches), 1 if is_last else 2):
            branches[i] = random.randint(0, 5)

        heading_horizontal = random.randint(150, 180)
        heading_vertical = 90

        return Tree(branches, heading_horizontal, heading_vertical)

    def __init__(self, branches, heading_horizontal=180, heading_vertical=90):
        self.branches = branches
        self.heading_horizontal = heading_horizontal
        self.heading_vertical = heading_vertical

    def __repr__(self):
        return f'Tree({self.branches}, {self.heading_horizontal}, {self.heading_vertical})'

    def check(self, is_last):
        if not is_last:
            for i_branch, branch in enumerate(self.branches):
                # Horizontal branches must have no rays:
                if i_branch % 2 == 1:
                    assert branch == 0

    def compute_height_before_op(self):
        num_steps_before_op = I_RAY_BRANCH_ZERO_OP + 1
        length_before_op = LENGTH_STEP * num_steps_before_op
        alpha = math.radians(self.heading_horizontal - 180)
        height_before_op = length_before_op * math.sin(alpha)
        return height_before_op


class Drawer:
    def __init__(self, t, occupied_pixels, canvas_width, canvas_height):
        self.turtle: turtle.Turtle = t
        self.occupied_pixels: set[tuple[int, int]] = occupied_pixels
        self.canvas_width = canvas_width
        self.canvas_height = canvas_height

    def get_turtle_position_on_canvas(self):
        x, y = self.turtle.position()
        x = int(x)
        y = int(y)

        xc = x + self.canvas_width // 2
        assert 0 <= xc < self.canvas_width

        yc = self.canvas_height // 2 - y
        assert 0 <= yc < self.canvas_height

        return xc, yc

    def get_starting_theta(self):
        return int(self.turtle.heading() - 180) % 360

    def compute_ray_length(self, length_limit=float('inf')):
        current_x, current_y = self.get_turtle_position_on_canvas()
        heading_rad = math.radians(self.turtle.heading())
        ray_width = self.turtle.pensize()

        # Define the change in x and y for the given heading
        delta_x = math.cos(heading_rad)
        delta_y = -math.sin(heading_rad)  # Negate to make positive angles go upwards

        length = 0

        while length < length_limit:
            # Calculate the new coordinates (move by small increments)
            next_x = current_x + delta_x
            next_y = current_y + delta_y

            # Check for collisions with canvas borders
            if next_x < 0 or next_x >= self.canvas_width or next_y < 0 or next_y >= self.canvas_height:
                return length

            x_min = math.floor(next_x) - ray_width // 2
            x_max = math.ceil(next_x) + ray_width // 2

            y_min = math.floor(next_y) - ray_width // 2
            y_max = math.ceil(next_y) + ray_width // 2

            if length == 0:
                new_pixels = ((x, y)
                              for x in range(x_min, x_max + 1)
                              for y in range(y_min, y_max + 1))
            else:
                new_pixels = []

                # noinspection Duplicates
                if math.floor(next_x) < math.floor(current_x):  # left pixels
                    new_pixels += ((x_min, y) for y in range(y_min, y_max + 1))
                elif math.ceil(next_x) > math.ceil(current_x):  # right pixels
                    new_pixels += ((x_max, y) for y in range(y_min, y_max + 1))

                # noinspection Duplicates
                if math.floor(next_y) < math.floor(current_y):  # upper pixels
                    new_pixels += ((x, y_min) for x in range(x_min, x_max + 1))
                elif math.ceil(next_y) > math.ceil(current_y):  # lower pixels
                    new_pixels += ((x, y_max) for x in range(x_min, x_max + 1))

            # Check for collision with occupied pixels
            if any(pixel in self.occupied_pixels for pixel in new_pixels):
                return length

            # Update the current position and increase the length
            current_x, current_y = next_x, next_y
            length += 1

        return length

    def forward_with_check(self, distance):
        ray_length = self.compute_ray_length(distance)
        if ray_length < distance:
            return False

        self.turtle.forward(distance)
        return True

    def get_pose(self):
        return *self.get_turtle_position_on_canvas(), self.get_starting_theta()

    def draw_bridge(self, altitude, spans, ray_length, heading_branch, heading_ray):
        """
        E.g., for spans (0, 233) and (15, 506):

        - intersection: (15, 233)
        - go backwards by: (506 - 233) + (233 - 15) * ratio
        """
        position_start = self.turtle.position()

        ratio = random.random()
        intersection_min = max(spans[-2][0], spans[-1][0])
        intersection_max = min(spans[-2][1], spans[-1][1])
        length_before_bridge = int(altitude + ray_length - intersection_max
                                   + (intersection_max - intersection_min) * ratio)
        self.turtle.backward(length_before_bridge)

        self.turtle.setheading(heading_branch + 180)
        if not self.forward_with_check(LENGTH_STEP):
            return False
        self.turtle.backward(LENGTH_STEP)

        self.turtle.setheading(heading_ray)
        self.turtle.forward(length_before_bridge)  # the original position

        position_finish = self.turtle.position()
        assert math.isclose(position_finish[0], position_start[0])
        assert math.isclose(position_finish[1], position_start[1])

    def draw_branch(self, tree, num_rays, i_branch):
        is_horizontal = i_branch % 2 == 0
        heading_branch = tree.heading_horizontal if is_horizontal else tree.heading_vertical
        heading_ray = tree.heading_vertical if is_horizontal else tree.heading_horizontal

        altitude: Optional[float] = None
        spans: list[tuple[float, float]] = []   # each pair: (altitude of the ray, altitude + length of the ray)

        for i_ray in range(num_rays):
            # Move to the beginning of the ray:
            self.turtle.setheading(heading_branch)
            if not self.forward_with_check(LENGTH_STEP):
                return False
            if i_ray == 0:
                altitude = 0.0
            else:
                altitude += math.sin(math.radians(heading_branch)) * LENGTH_STEP

            # Draw the ray:
            self.turtle.setheading(heading_ray)
            ray_length = self.compute_ray_length() - WIDTH_GAP_RAY_OBSTACLE
            if ray_length < LENGTH_RAY_MIN:
                return False
            self.turtle.forward(ray_length)
            print(f'Ray end: {self.get_pose()}')
            spans.append((altitude, altitude + ray_length))
            if i_ray % 2 == 1 and random.random() < PROBABILITY_BRIDGE_PRESENCE:
                for i_bridge in range(1 if random.random() < PROBABILITY_BRIDGE_SINGLE else 2):
                    self.draw_bridge(altitude, spans, ray_length, heading_branch, heading_ray)
            self.turtle.backward(ray_length)

            # Draw the OP if needed:
            if i_branch == 0 and i_ray == I_RAY_BRANCH_ZERO_OP:
                self.turtle.left(180)
                if not self.forward_with_check(LENGTH_OP):
                    return False
                print(f'OP: {self.get_pose()}')
                self.turtle.backward(LENGTH_OP)
                self.turtle.right(180)

        self.turtle.setheading(heading_branch)
        if not self.forward_with_check(LENGTH_STEP):
            return False

        return True

    def draw_tree(self, tree):
        for i_branch, num_rays in enumerate(tree.branches):
            if not self.draw_branch(tree, num_rays, i_branch):
                return False

        return True

    def home(self, trees, width_image, height_image):
        x = width_image
        y = height_image

        y -= trees[0].compute_height_before_op() + LENGTH_OP
        y = min(height_image - 1, y)

        x -= WIDTH_PEN // 2
        y -= WIDTH_PEN // 2

        self.turtle.penup()
        self.turtle.goto(int(x) - width_image // 2,
                         height_image // 2 - int(y))
        self.turtle.pendown()

    def draw_trees(self, trees, width_image, height_image):
        assert trees
        self.home(trees, width_image, height_image)

        for tree in trees:
            if not self.draw_tree(tree):
                return False

        return True


def image_to_occupied_pixels(image):
    assert image.mode == 'RGB'
    pixels = image.load()
    occupied_pixels = set()

    for y in range(image.height):
        for x in range(image.width):
            rgb = pixels[x, y]
            if rgb != (0, 0, 0):
                occupied_pixels.add((x, y))
                #print(f"Pixel at ({x}, {y}) is occupied.")

    return occupied_pixels


def export_to_eps(t: turtle.Turtle, filename_eps: str, width: int, height: int) -> None:
    t.hideturtle()
    t.getscreen().getcanvas().postscript(file=filename_eps,
                                         width=width, height=height,
                                         pagewidth=width, pageheight=height)
    t.showturtle()


def convert_eps_to_png(filename_eps, filename_png, width, height):
    pic = Image.open(filename_eps)
    pic.load()

    x_min = WIDTH_GAP_IMAGE_CANVAS // 2 + 1
    y_min = WIDTH_GAP_IMAGE_CANVAS // 2 + 2

    x_max = x_min + width
    y_max = y_min + height

    assert pic.getpixel((x_min - 1, y_min - 1)) == (255, 255, 255)
    assert pic.getpixel((x_max + 1, y_max + 1)) == (255, 255, 255)

    pic = pic.crop((x_min, y_min, x_max, y_max))

    pic.save(filename_png)


def generate_map(filename_map_png):
    print(f'=== GENERATING {filename_map_png} ===')

    image = Image.open(FILENAME_OBSTACLES_PNG)
    occupied_pixels = image_to_occupied_pixels(image)

    screen = turtle.getscreen()
    width_screen = image.width + WIDTH_GAP_IMAGE_CANVAS
    height_screen = image.height + WIDTH_GAP_IMAGE_CANVAS

    turtle.mode('standard')
    turtle.setup(width=width_screen,
                 height=height_screen,
                 startx=X_WINDOW_START,
                 starty=Y_WINDOW_START)
    screen.bgcolor('gray')
    screen.bgpic(FILENAME_OBSTACLES_PNG)

    t = turtle.Turtle()
    t.pencolor('white')
    t.width(WIDTH_PEN)
    t.speed('fastest')

    drawer = Drawer(t, occupied_pixels, image.width, image.height)

    num_trees = 2
    trees = [Tree.generate(is_last=i == num_trees - 1) for i in range(num_trees)]

    for i_tree, tree in enumerate(trees):
        tree.check(i_tree == len(trees) - 1)

    if not drawer.draw_trees(trees, image.width, image.height):
        print('Not all trees are drawn')
    else:
        print('All trees are drawn')

    with tempfile.NamedTemporaryFile() as fp:
        export_to_eps(t, fp.name, width_screen, height_screen)
        convert_eps_to_png(fp.name, filename_map_png, image.width, image.height)

    #screen.mainloop()


def main():
    random.seed(1)

    path_maps = pathlib.Path('generated-maps')
    if not path_maps.exists():
        path_maps.mkdir()
    else:
        for path in path_maps.iterdir():
            path.unlink()

    num_maps = 5
    for i in range(1, num_maps + 1):
        generate_map(str(path_maps / f'map{i}.png'))


if __name__ == '__main__':
    main()