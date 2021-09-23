# See the result of my work below (YouTube video):

# https://youtu.be/YtEP7Gh4IgI

# FlockSimulator
School OpenGL project. I used CUDA language to make all computations on GPU.<br />
At the beginnig i had 20 - 30FPS with 500 fishes before optimization. <br />
Solution optimized for parallelism gave me 20FPS for 40k boids and 60FPS for 15k boids. <br />

# About algorithm:
Basic model of flocking behavior is controlled by three simple rules:

Separation – avoid crowding neighbours <br />
Alignment – steer towards average heading of neighbours <br />
Cohesion – steer towards average position of neighbours <br />

User can manipulate the strenght of these 3 rules using keyboard buttons.
What is more, boids avoid mouse cursor if user press 'M' key.


# More information about flocking algorithm: 
https://en.wikipedia.org/wiki/Flocking_(behavior)


