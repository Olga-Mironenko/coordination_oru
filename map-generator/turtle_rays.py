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
import turtle

from PIL import Image

LENGTH_STEP = 50
WIDTH_PEN = LENGTH_STEP // 2
assert WIDTH_PEN < LENGTH_STEP  # for a gap between rays

I_RAY_BRANCH_ZERO_OP = 2
LENGTH_OP = LENGTH_STEP
LENGTH_RAY_MIN = 1  # or LENGTH_STEP

# Note: GIF can have only 256 colors, so they may be distorted (not pure black in particular;
# see https://docs.gimp.org/2.10/en/gimp-stuck-export-gif-colors-changed.html).
FILENAME_BG_PNG = 'obstacles.png'
WIDTH_GAP_IMAGE_CANVAS = 100
WIDTH_GAP_RAY_OBSTACLE = WIDTH_PEN // 2 + 10


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
        return int(self.turtle.heading() - 180)

    def compute_ray_length(self):
        current_x, current_y = self.get_turtle_position_on_canvas()
        heading_rad = math.radians(self.turtle.heading())
        ray_width = self.turtle.pensize()

        # Define the change in x and y for the given heading
        delta_x = math.cos(heading_rad)
        delta_y = -math.sin(heading_rad)  # Negate to make positive angles go upwards

        length = 0

        while True:
            # Calculate the new coordinates (move by small increments)
            next_x = current_x + delta_x
            next_y = current_y + delta_y

            # Check for collisions with canvas borders
            if next_x < 0 or next_x >= self.canvas_width or next_y < 0 or next_y >= self.canvas_height:
                return length

            next_x_int = int(next_x)
            next_y_int = int(next_y)

            # Check for collision with occupied pixels
            for i in range(-ray_width // 2, ray_width // 2 + 1):
                for j in range(-ray_width // 2, ray_width // 2 + 1):
                    if (next_x_int + i, next_y_int + j) in self.occupied_pixels:
                        return length

            # Update the current position and increase the length
            current_x, current_y = next_x, next_y
            length += 1

    def forward_with_check(self, distance):
        ray_length = self.compute_ray_length()
        if ray_length < distance:
            return False

        self.turtle.forward(distance)
        return True

    def draw_branch(self, num_rays, i_branch):
        is_horizontal = i_branch % 2 == 0

        for i_ray in range(num_rays):
            if not self.forward_with_check(LENGTH_STEP):
                return False
            if is_horizontal:
                self.turtle.right(90)
            else:
                self.turtle.left(90)

            ray_length = self.compute_ray_length() - WIDTH_GAP_RAY_OBSTACLE
            if ray_length < LENGTH_RAY_MIN:
                return False
            self.turtle.forward(ray_length)
            print(f'Ray end: {(*self.get_turtle_position_on_canvas(), self.get_starting_theta())}')
            self.turtle.backward(ray_length)

            if i_branch == 0 and i_ray == I_RAY_BRANCH_ZERO_OP:
                self.turtle.left(180)
                if not self.forward_with_check(LENGTH_OP):
                    return False
                self.turtle.backward(LENGTH_OP)
                self.turtle.right(180)

            if is_horizontal:
                self.turtle.left(90)
            else:
                self.turtle.right(90)

        if not self.forward_with_check(LENGTH_STEP):
            return False
        if is_horizontal:
            self.turtle.right(90)
        else:
            self.turtle.left(90)

        return True

    def draw_tree(self, tree):
        self.turtle.setheading(180)  # west

        for i_branch, num_rays in enumerate(tree):
            if not self.draw_branch(num_rays, i_branch):
                return False

        return True

    def draw_trees(self, trees):
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


def main():
    image = Image.open(FILENAME_BG_PNG)
    occupied_pixels = image_to_occupied_pixels(image)

    turtle.mode('standard')
    turtle.setup(image.width + WIDTH_GAP_IMAGE_CANVAS, image.height + WIDTH_GAP_IMAGE_CANVAS)
    turtle.getscreen().bgcolor('gray')
    turtle.getscreen().bgpic(FILENAME_BG_PNG)

    t = turtle.Turtle()
    t.pencolor('white')
    t.width(WIDTH_PEN)
    t.speed('fastest')

    t.penup()
    t.goto(image.width // 2 - WIDTH_PEN // 2,
           -image.height // 2 + LENGTH_OP + WIDTH_PEN // 2)
    t.pendown()

    drawer = Drawer(t, occupied_pixels, image.width, image.height)

    trees = [
        [6, 0, 4],
        [3, 0, 5],
        [5, 4, 3, 2],
    ]

    for i_tree in range(len(trees) - 1):
        tree = trees[i_tree]
        for i_branch in range(len(tree)):
            # Horizontal branches must have no rays:
            if i_branch % 2 == 1:
                assert tree[i_branch] == 0

    if not drawer.draw_trees(trees):
        print('Not all trees are drawn')
    else:
        print('All trees are drawn')

    t.screen.mainloop()


if __name__ == '__main__':
    main()