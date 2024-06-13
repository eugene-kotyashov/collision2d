This is a 2D object collision physics simulation demo done in pure C language.

Currently only point to edge contact points are processed.
Only collisions with no zero normal projection of relative
    velocity on colliding edge's normal  are resolved.
Midpoint method of solving 2d differential equations of motions is used.
Demo is using bodies of rectangular shape.
Visualization is done via SDL library.