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
import turtle

M = 10

def draw_branch(t, num_rays, is_horizontal):
    for i in range(num_rays):
        t.forward(M)
        if is_horizontal:
            t.right(90)
        else:
            t.left(90)

        t.forward(10 * M)
        t.backward(10 * M)

        if is_horizontal:
            t.left(90)
        else:
            t.right(90)

    t.forward(M)
    if is_horizontal:
        t.right(90)
    else:
        t.left(90)


def draw_tree(t, rays):
    t.setheading(180)  # west
    for i_branch, num_rays in enumerate(rays):
        draw_branch(t, num_rays, i_branch % 2 == 0)


def main():
    turtle.mode('standard')

    t = turtle.Turtle()
    t.speed('fastest')
    t.width(3)

    draw_tree(t, [6, 2, 4])
    draw_tree(t, [5, 4])

    t.screen.mainloop()


if __name__ == '__main__':
    main()