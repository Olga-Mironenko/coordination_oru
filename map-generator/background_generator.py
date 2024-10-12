from PIL import Image, ImageDraw
import random

def biased_random(max_value: int, bias_factor: float) -> int:
    """
    Generate a random number between 0 and max_value with a bias towards 0.
    """
    return int(max_value * (random.random() ** bias_factor))


def generate_background(
    filename: str = "test_background.png",
    *,
    image_width: int = 800,
    image_height: int = 600,
    background_color: tuple[int, int, int] = (0, 0, 0),  # Black
    ellipse_color: tuple[int, int, int] = (184, 103, 103),  # Red
    num_ellipses: int = 10,
    min_ellipse_width: int = 30,
    max_ellipse_width: int = 100,
    min_ellipse_height: int = 30,
    max_ellipse_height: int = 100,
    bias_factor: float = 3.0
) -> None:
    # Create a new black image
    image = Image.new("RGB", (image_width, image_height), background_color)
    draw = ImageDraw.Draw(image)

    # Draw random ellipses on the image
    for _ in range(num_ellipses):
        while True:
            # Generate random top-left position with bias towards top and left
            x1 = biased_random(image_width, bias_factor)
            y1 = biased_random(image_height, bias_factor)

            # Generate random size for the ellipse within specified limits
            ellipse_width = random.randint(min_ellipse_width, max_ellipse_width)
            ellipse_height = random.randint(min_ellipse_height, max_ellipse_height)

            # Calculate bottom-right position based on top-left and size
            x2 = x1 + ellipse_width
            y2 = y1 + ellipse_height

            # Ensure the ellipse does not touch the right-lower quarter of the image
            if not (x2 >= image_width * 0.5 and y2 >= image_height * 0.5):
                break

        # Draw the ellipse
        draw.ellipse([x1, y1, x2, y2], fill=ellipse_color)

    # Save the generated image
    image.save(filename)


def main() -> None:
    generate_background()


if __name__ == "__main__":
    main()