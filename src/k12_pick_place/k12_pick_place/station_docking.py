"""Bounded pose control at the measured skid-steer rotation centre."""
import math


def angle(value):
    return math.remainder(value, 2*math.pi)


def docking_command(base, goal, pivot_x):
    """Control the front-axle point, including the final station heading.

    Physical pure-turn recordings put the instantaneous centre 195 mm ahead
    of base_footprint, at this model's front axle. Controlling base_footprint as
    a unicycle makes rotation produce unmodelled lateral error and orbiting.
    Both current and target poses are shifted by the same robot-fixed offset.
    """
    x,y,yaw=base;gx,gy,gyaw=goal
    px,py=x+pivot_x*math.cos(yaw), y+pivot_x*math.sin(yaw)
    tx,ty=gx+pivot_x*math.cos(gyaw), gy+pivot_x*math.sin(gyaw)
    dx,dy=tx-px,ty-py
    distance=math.hypot(dx,dy)
    heading=angle(gyaw-yaw)
    if distance < .008:
        return 0., max(-.22,min(.22,1.1*heading)), distance, heading
    bearing=math.atan2(dy,dx)
    alpha=angle(bearing-yaw)
    beta=angle(gyaw-bearing)
    velocity=min(.05,.45*distance)*max(0.,math.cos(alpha)) if abs(alpha)<.6 else 0.
    omega=max(-.22,min(.22,1.2*alpha-.5*beta))
    return velocity,omega,distance,heading
