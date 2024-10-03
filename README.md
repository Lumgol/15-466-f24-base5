# neon labyrinth

Author: Sasha Mishkin

Design: Look, I just wanted to practice making fun shaders :) 

(This is a little maze but with weird colors/lighting! Objects a certain distance away become completely dark, which makes gameplay a little more challenging.)

I took inspiration from [REM: Dream Generator](https://virtuavirtues-backlog.itch.io/rem) on itch.io.

Screen Shot: [Screenshot](screenshot.png)

How To Play:

Just use arrow keys to move around on the walkmesh and the mouse to rotate the camera. Try to find the (downward-going) tunnel at the center of the maze, go through, end up at a second maze, find its center and go through another tunnel, and then chill at the end platform on the other side.

Sources: I used [https://mazegenerator.net/](https://mazegenerator.net/) as a preliminary reference for the mazes' shapes but modified my actual mazes from the generated one. I also used Shadertoy to write and test a [2D version](https://www.shadertoy.com/view/MXfBRn) of my shader, just because the quick feedback and isolation of the shader from all of the other game parts was helpful.

This game was built with [NEST](NEST.md).