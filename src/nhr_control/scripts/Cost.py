import math

r = 0.038
L = 0.354
dt = 0.1

# Xi, Yi, Thetai: Input point's coordinates
# Xn, Yn, Thetan: End point coordintes
def cost(Xi,Yi,Thetai,UL,UR):
    t = 0
    Xn = Xi
    Yn = Yi
    Thetan = math.pi * Thetai / 180
    D = 0
    while t < 1:
        dd = 0.5 * r * (UL + UR) * dt
        dXn = dd * math.cos(Thetan)
        dYn = dd * math.sin(Thetan)
        Xn += dXn
        Yn += dYn
        Thetan += ((r / L) * (UR - UL) * dt)
        D += math.sqrt(dXn**2 + dYn**2)
        t += dt
    Thetan = 180 * Thetan / math.pi
    return Xn, Yn, Thetan, D
