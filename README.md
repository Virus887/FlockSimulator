# FlockSimulator
School OpenGL project. I used CUDA language to make all computations on GPU.
At the begginig i had 20 - 30FPS with 500 fishes before optimization.
Solution optimized for parallelism gave me 30FPS for 40k fishes.

#About algorithm:
Basic model of flocking behavior is controlled by three simple rules:

Separation – avoid crowding neighbours 
Alignment – steer towards average heading of neighbours
Cohesion – steer towards average position of neighbours

User can manipulatestrenght of these 3 rules using keyboard buttons.
What is more, boids avoid mouse cursor if user prass 'M' key.


#More information about flocking algorithm: 
https://en.wikipedia.org/wiki/Flocking_(behavior)


# See the result of my work below:
https://youtu.be/YtEP7Gh4IgI
