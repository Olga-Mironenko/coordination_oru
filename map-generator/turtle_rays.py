"""
The general idea::

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
import collections
import datetime
import hashlib
import json
import math
import pathlib
import random
import subprocess
import sys
import textwrap
import turtle
from typing import Dict, List, Optional

from PIL import Image
from loguru import logger

import background_generator

NUM_MAPS = 1
NUM_AUTS = 2
IS_HUMAN_STAYING = False
IS_OP_TREE0_BRANCH0_ALLOWED = True

IMAGE_WIDTH = 400
IMAGE_HEIGHT = 300
NUM_ELLIPSES = 0

LENGTH_STEP = 50
WIDTH_PEN = LENGTH_STEP // 2
assert WIDTH_PEN < LENGTH_STEP  # for a gap between rays

I_RAY_OP = 2
LENGTH_OP = LENGTH_STEP
LENGTH_RAY_MIN = 1  # or LENGTH_STEP

WIDTH_GAP_IMAGE_CANVAS = 100
assert WIDTH_GAP_IMAGE_CANVAS % 2 == 0
WIDTH_GAP_RAY_OBSTACLE = WIDTH_PEN * 2

X_WINDOW_START = 0
Y_WINDOW_START = 0

PROBABILITY_BRIDGE_PRESENCE = 1.0
PROBABILITY_BRIDGE_SINGLE = 0.5

MAP_RESOLUTION_COORDINATION_ORU = 0.1  # meters per pixel

KIND_POSE_TO_COLOR_LABEL = {'D': 'gray', 'OP': 'yellow', 'start': None, 'finish': None}
FONT_LABEL = ('Arial', 16, 'normal')
HEIGHT_TEXT = 6 + FONT_LABEL[1]


class Pose:
    def __init__(self, x, y, heading):
        self.x = x
        self.y = y
        self.heading = heading

    def __repr__(self):
        return f"Pose({self.x}, {self.y}, {self.heading})"

    def to_coordination_oru_format(self, image_height):
        return tuple(round(value, 3)
                     for value in (
                         self.x * MAP_RESOLUTION_COORDINATION_ORU,
                         (image_height - self.y) * MAP_RESOLUTION_COORDINATION_ORU,
                         math.radians(self.heading),
                     ))

    def distance_to(self, other: 'Pose') -> float:
        return math.sqrt((self.x - other.x) ** 2 + (self.y - other.y) ** 2)


class Tree:
    @classmethod
    def generate(cls, *, is_last):
        branches = [0] * random.randint(2, 4)
        for i in range(0, len(branches), 1 if is_last else 2):
            branches[i] = random.randint(0, 5)

        heading_horizontal = random.randint(165, 180)
        heading_vertical = 90

        return Tree(branches, heading_horizontal, heading_vertical)

    def __init__(self, branches, heading_horizontal=180, heading_vertical=90):
        self.branches = branches
        self.heading_horizontal = heading_horizontal
        self.heading_vertical = heading_vertical

    def __repr__(self):
        return f'Tree({self.branches}, {self.heading_horizontal}, {self.heading_vertical})'

    def __getitem__(self, item):
        return self.branches[item]

    def check(self, is_last):
        if not is_last:
            for i_branch, branch in enumerate(self.branches):
                # Horizontal branches must have no rays:
                if i_branch % 2 == 1:
                    assert branch == 0


class Drawer:
    def __init__(self, t, occupied_pixels, canvas_width, canvas_height):
        self.turtle: turtle.Turtle = t
        self.occupied_pixels: set[tuple[int, int]] = occupied_pixels
        self.canvas_width = canvas_width
        self.canvas_height = canvas_height
        self.trees = None

        self.name2pose: dict[str, Pose] = {}
        self.kind_pose_to_num: dict[str, int] = {}
        self.num_ops = 0

    def add_pose(self, kind, *, is_facing_dead_end=True):
        self.kind_pose_to_num[kind] = self.kind_pose_to_num.get(kind, 0) + 1
        if kind in ('start', 'finish'):
            assert self.kind_pose_to_num[kind] == 1
            name = kind
        elif kind in ('D', 'OP'):
            name = f'{kind}{self.kind_pose_to_num[kind]}'
        else:
            raise ValueError(f'Unknown kind {kind}')

        delta = WIDTH_PEN // 2

        self.turtle.penup()
        if is_facing_dead_end:
            self.turtle.backward(delta)
        else:
            self.turtle.forward(delta)
        #self.turtle.dot(WIDTH_PEN // 2, "lightgray")
        pose = self.get_pose()
        if is_facing_dead_end:
            self.turtle.forward(delta)
        else:
            self.turtle.backward(delta)
        self.turtle.pendown()

        self.name2pose[name] = pose
        logger.info(f'Pose: {name}: {pose}')

        color = KIND_POSE_TO_COLOR_LABEL[kind]
        if color is not None:
            self.draw_label(name, color)

    def draw_label(self, label, color):
        position_orig = self.turtle.position()
        x, y = position_orig

        width_text = FONT_LABEL[1] * len(label)

        heading = self.turtle.heading()
        if 90 - 45 <= heading <= 90 + 45:  # up
            y += WIDTH_PEN // 2
        elif 270 - 45 <= heading <= 270 + 45:  # down
            y -= WIDTH_PEN // 2 + HEIGHT_TEXT + 10
        elif 90 + 45 <= heading <= 270 - 45:  # left
            x -= WIDTH_PEN // 2 + width_text // 2
            y -= WIDTH_PEN // 2
        else:  # right
            x += WIDTH_PEN // 2 + width_text // 2
            y -= WIDTH_PEN // 2

        self.turtle.penup()
        self.turtle.goto(x, y)

        pencolor_orig = self.turtle.pencolor()
        self.turtle.pencolor(color)
        self.turtle.write(label, align='center', font=FONT_LABEL)
        self.turtle.pencolor(pencolor_orig)

        self.turtle.goto(*position_orig)
        self.turtle.pendown()

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
        return Pose(*self.get_turtle_position_on_canvas(), self.get_starting_theta())

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

    def is_op_drawing_needed(self, i_tree, i_branch, i_ray) -> bool:
        if i_ray == I_RAY_OP:
            if i_branch % 2 == 0 and (IS_OP_TREE0_BRANCH0_ALLOWED or not i_tree == i_branch == 0):
                if i_branch > 0:
                    branch_prev = self.trees[i_tree][i_branch - 1]
                elif i_tree > 0:
                    branch_prev = self.trees[i_tree - 1][-1]
                else:
                    branch_prev = 0

                if branch_prev == 0:
                    return True

        return False

    def draw_branch(self, i_tree: int, tree: Tree, i_branch: int, num_rays: int) -> bool:
        is_horizontal = i_branch % 2 == 0
        heading_branch = tree.heading_horizontal if is_horizontal else tree.heading_vertical
        heading_ray = tree.heading_vertical if is_horizontal else tree.heading_horizontal
        heading_original = self.turtle.heading()

        altitude: Optional[float] = None
        spans: list[tuple[float, float]] = []   # each pair: (altitude of the ray, altitude + length of the ray)

        for i_ray in range(num_rays):
            # Move to the beginning of the ray:
            self.turtle.setheading(heading_branch)
            if not self.forward_with_check(LENGTH_STEP):
                if i_ray == 0:
                    self.turtle.setheading(heading_original)
                return False
            if i_ray == 0:
                altitude = 0.0
            else:
                altitude += math.sin(math.radians(heading_branch)) * LENGTH_STEP

            # Draw the ray:

            self.turtle.setheading(heading_ray)
            ray_length = self.compute_ray_length() - WIDTH_GAP_RAY_OBSTACLE
            if ray_length < LENGTH_RAY_MIN:
                self.turtle.setheading(heading_branch)
                return False
            self.turtle.forward(ray_length)

            self.add_pose('D')  # draw point

            spans.append((altitude, altitude + ray_length))
            if i_ray % 2 == 1 and random.random() < PROBABILITY_BRIDGE_PRESENCE:
                for i_bridge in range(1 if random.random() < PROBABILITY_BRIDGE_SINGLE else 2):
                    self.draw_bridge(altitude, spans, ray_length, heading_branch, heading_ray)

            self.turtle.backward(ray_length)

            # Draw the OP of the branch if needed:
            if self.is_op_drawing_needed(i_tree, i_branch, i_ray):
                self.turtle.left(180)
                if not self.forward_with_check(LENGTH_OP):
                    self.turtle.setheading(heading_branch)
                    return False
                self.add_pose('OP')  # ore point
                self.turtle.backward(LENGTH_OP)
                self.turtle.right(180)
                self.num_ops += 1

        self.turtle.setheading(heading_branch)
        if not self.forward_with_check(LENGTH_STEP):
            return False

        return True

    def draw_tree(self, i_tree, tree):
        for i_branch, num_rays in enumerate(tree.branches):
            if not self.draw_branch(i_tree, tree, i_branch, num_rays):
                return False

        return True

    def home(self, width_image, height_image, heading):
        x = width_image - 1
        y = height_image - 1

        x -= WIDTH_PEN // 2
        y -= WIDTH_PEN // 2 + HEIGHT_TEXT

        self.turtle.penup()
        self.turtle.goto(int(x) - width_image // 2,
                         height_image // 2 - int(y))
        self.turtle.pendown()

        self.turtle.setheading(heading)

    def draw_trees(self, trees: List[Tree], width_image: int, height_image: int) -> bool:
        assert trees
        assert self.trees is None
        self.trees = trees

        self.home(width_image, height_image, trees[0].heading_horizontal)
        self.add_pose('start', is_facing_dead_end=False)

        is_ok = True
        for i_tree, tree in enumerate(trees):
            if not self.draw_tree(i_tree, tree):
                is_ok = False
                break

        self.add_pose('finish')

        self.trees = None
        return is_ok


def add_robots_to_name2pose(num_auts: int, name2pose: Dict[str, Pose]) -> Dict[str, Pose]:
    # Make a copy of the original dictionary to avoid modifying the input
    new_name2pose = {}

    # Get all D and OP poses
    d_poses = {key: pose for key, pose in name2pose.items() if key.startswith('D')}
    op_poses = {key: pose for key, pose in name2pose.items() if key.startswith('OP')}

    if len(d_poses) < num_auts:
        raise ValueError("Not enough D poses to assign unique start positions for all automated vehicles.")
    if not op_poses:
        raise ValueError("No OP poses to assign finish positions for automated vehicles.")

    # Select unique D poses for each automated vehicle start
    available_d_keys = list(d_poses)
    random.shuffle(available_d_keys)
    selected_d_keys = available_d_keys[:num_auts]

    # Assign start and nearest finish for each automated vehicle
    key2pairs = collections.defaultdict(list)
    for i in range(num_auts):
        start_name = f'aut{i + 1}_start'
        start_pose = d_poses[selected_d_keys[i]]

        finish_name = f'aut{i + 1}_finish'
        # Find the nearest OP pose for the finish
        nearest_op_key = min(op_poses, key=lambda k: start_pose.distance_to(op_poses[k]))
        finish_pose = op_poses[nearest_op_key]

        key2pairs[selected_d_keys[i]].append((start_name, start_pose))
        key2pairs[nearest_op_key].append((finish_name, finish_pose))

    # Iterate through original name2pose dictionary and add start/finish pairs after corresponding items
    for key, value in name2pose.items():
        new_name2pose[key] = value

        # Insert 'hum1_start' after 'start'
        if key == 'start':
            new_name2pose['hum1_start'] = value
            if IS_HUMAN_STAYING:
                new_name2pose['hum1_finish'] = value

        # Insert 'hum1_finish' after 'finish'
        elif key == 'finish':
            if not IS_HUMAN_STAYING:
                new_name2pose['hum1_finish'] = value

        # Insert each automated vehicle's start and finish after the corresponding D pose and OP pose
        elif key in key2pairs:
            for name, pose in key2pairs[key]:
                new_name2pose[name] = pose

    return new_name2pose


def image_to_occupied_pixels(image):
    # Note: GIF can have only 256 colors, so they may be distorted (not pure black in particular;
    # see https://docs.gimp.org/2.10/en/gimp-stuck-export-gif-colors-changed.html).

    assert image.mode == 'RGB'
    pixels = image.load()
    occupied_pixels = set()

    for y in range(image.height):
        for x in range(image.width):
            rgb = pixels[x, y]
            if rgb != (0, 0, 0):
                occupied_pixels.add((x, y))
                logger.debug(f"Pixel at ({x}, {y}) is occupied.")

    return occupied_pixels


def make_screenshot(t: turtle.Turtle,
                    image_width: int, image_height: int,
                    filename_map_png: str) -> None:
    """
    Note: If there is a risk of the window on the main display being overlapped,
    run `Xvfb :99 -screen 0 1024x768x24` and pass `DISPLAY=:99` to the program.
    """
    screen = t.getscreen()

    bgcolor_orig = screen.bgcolor()
    bgpic_orig = screen.bgpic()

    screen.bgcolor('black')
    screen.bgpic('nopic')
    t.hideturtle()

    shift = WIDTH_GAP_IMAGE_CANVAS // 2 + 2
    subprocess.run(
        f'xwd -nobdrs -silent -id $(xdotool search --name "Python Turtle Graphics")'
        f' | convert xwd:- -crop {image_width}x{image_height}+{shift}+{shift} -strip {filename_map_png}',
        shell=True,
        check=True,
    )

    screen.bgcolor(bgcolor_orig)
    screen.bgpic(bgpic_orig)
    t.showturtle()


def save_locations(num_auts: int, filename_locations: str, drawer: Drawer, image_height: int) -> None:
    with open(filename_locations, 'w') as file:
        print('# Locations:', file=file)
        print('# name', 'x_meters', 'y_meters', 'theta', sep='\t', file=file)
        print(file=file)

        name2pose_with_robots = add_robots_to_name2pose(num_auts, drawer.name2pose)
        pose_prev = None
        for name, pose in name2pose_with_robots.items():
            if pose_prev is not None and pose != pose_prev:
                print(file=file)
            print(name, *pose.to_coordination_oru_format(image_height), sep='\t', file=file)
            pose_prev = pose


def compute_hash_bytes(data_bytes: bytes) -> str:
    return hashlib.sha1(data_bytes).hexdigest()[:8]


def compute_hash_filename(filename_map_png: str) -> str:
    with open(filename_map_png, 'rb') as file:
        return compute_hash_bytes(file.read())


def compute_hash_text(text_mapconf: str) -> str:
    return compute_hash_bytes(text_mapconf.encode())


def generate_scenario(path_maps: pathlib.Path, i_map: int, i_generation: int) -> bool:
    basename_map_png = f'map{i_map}.png'
    basename_locations = f'locations{i_map}.tsv'

    filename_scenario = str(path_maps / f'scenario{i_map}.json')
    filename_map_png = str(path_maps / basename_map_png)
    filename_locations = str(path_maps / basename_locations)
    # Note: `filename_background_png_to_generate` is made unique among all attempts to make `bgpic` work properly.
    filename_background_png_to_generate = str(path_maps / f'background{i_map}_g{i_generation}.png')
    filename_log = str(path_maps / f'log{i_map}_g{i_generation}.log')

    logger.remove()
    logger.add(
        sys.stdout,
        level='INFO',
        format='<green>{time:HH:mm:ss.SSS}:</green> <level>{level}:</level> {message}',
    )
    logger.add(
        filename_log,
        level='INFO',
        format='<level>{level}:</level> {message}',
    )

    logger.info(f'=== GENERATING {filename_scenario} (g{i_generation}) ===')

    random.seed(i_generation)

    dimensions_vehicle = (1.5, 1.0, 0, 0, 0, 0)  # see `VehicleSize.java`

    background_generator.generate_background(
        filename_background_png_to_generate,
        image_width=IMAGE_WIDTH,
        image_height=IMAGE_HEIGHT,
        num_ellipses=NUM_ELLIPSES,
    )
    image = Image.open(filename_background_png_to_generate)
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
    screen.bgpic(filename_background_png_to_generate)

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
        logger.warning('Not all trees are drawn')
    else:
        logger.info('All trees are drawn')

    if drawer.num_ops == 0:
        logger.warning('No OP is drawn')
        return False

    make_screenshot(t, image.width, image.height, filename_map_png)

    save_locations(NUM_AUTS, filename_locations, drawer, image.height)

    text_mapconf = textwrap.dedent(f"""
        image: {basename_map_png}
        resolution: 0.1
        origin: [0, 0, 0]
        occupied_thresh: 1.0
        """).lstrip()

    hexdigest_mapconf = compute_hash_text(
        '-'.join((
            compute_hash_filename(filename_map_png),
            compute_hash_text(text_mapconf),
            compute_hash_text(str(dimensions_vehicle))
        ))
    )
    basename_mapconf = f'mapconf{i_map}-{hexdigest_mapconf}.yaml'
    filename_mapconf = str(path_maps / basename_mapconf)

    with open(filename_mapconf, 'w') as file:
        file.write(text_mapconf)

    scenario = {
        'mapconf': basename_mapconf,
        'locations': basename_locations,
        'num_auts': NUM_AUTS,
        'dimensions_vehicle': list(dimensions_vehicle),
    }

    with open(filename_scenario, 'w') as file:
        json.dump(scenario, file, indent=4, ensure_ascii=False)

    #screen.mainloop()
    return True


def main():
    path_root = pathlib.Path('generated-maps')
    subdir = datetime.datetime.now().isoformat(sep='_', timespec='seconds')

    path_maps = path_root / subdir
    path_maps.mkdir(exist_ok=False, parents=True)

    path_current = path_root / 'current'
    path_current.unlink(missing_ok=True)
    path_current.symlink_to(subdir, target_is_directory=True)

    i_generation = 1
    for i_map in range(1, NUM_MAPS + 1):
        while True:
            is_ok = generate_scenario(path_maps, i_map, i_generation)
            i_generation += 1
            if is_ok:
                break


if __name__ == '__main__':
    main()