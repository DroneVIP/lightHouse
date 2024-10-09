import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Slider, Button

# Constants
G = 6.67430e-11  # Gravitational constant (m^3 kg^-1 s^-2)
mu = 3.986e14    # Standard gravitational parameter for Earth (m^3 s^-2)
M = 5.97e24      # Mass of Earth (kg)
R = 6371000      # Radius of Earth (meters)
r_0 = 500000     # Perigee altitude (meters)
v_0 = 9000       # Initial Tangential Velocity of Spacecraft at Perigee (m/s)
#v_0 = np.sqrt(G * M / r)  # Initial Tangential Velocity of Spacecraft at Perigee (m/s)

def specific_orbital_energy():
    soe = ((v_0**2)/2)-(mu/(R+r_0))
    return soe  # Returns energy in J/kg (m^2/s^2)

def semi_major_axis():
    a = -(mu/(2*specific_orbital_energy()))
    return a  # Returns semi-major axis in meters

def specific_angular_momentum():
    h = (R+r_0) * v_0
    return h  # Returns angular momentum in m^2/s

def eccentricity():
    e = np.sqrt(1+(2*specific_orbital_energy()*specific_angular_momentum()**2)/(mu**2))
    return e  # Dimensionless

def keplers_first_law(theta):
    a = semi_major_axis()
    e = eccentricity()
    r = a * (1 - e**2) / (1 + e * np.cos(theta))
    return r  # Returns radial distance in meters

def mean_motion():
    a = semi_major_axis()
    n = np.sqrt(mu/a**3)
    return n  # Returns mean motion in radians/second

def mean_to_true(time_in_seconds):
    # time_in_seconds is in seconds
    # Returns true anomaly in radians
    n = mean_motion()
    e = eccentricity()
    m = n * np.asarray(time_in_seconds)  # Mean anomaly in radians
    
    # Ensure m is in the range [0, 2π]
    m = m % (2 * np.pi)
    
    # Initial guess for E (eccentric anomaly in radians)
    E = m + e * np.sin(m) / (1 - np.sin(m + e) + np.sin(m))
    
    # Newton-Raphson iteration
    for _ in range(10):  # Usually converges in a few iterations
        E = E - (E - e * np.sin(E) - m) / (1 - e * np.cos(E))
    
    # Convert eccentric anomaly to true anomaly
    f = 2 * np.arctan(np.sqrt((1 + e) / (1 - e)) * np.tan(E / 2))
    
    # Ensure f is in the range [0, 2π]
    f = f % (2 * np.pi)
    
    return f  # Returns true anomaly in radians


def plot_orbit(num_points=2000):
    total_time = 2 * np.pi * np.sqrt(semi_major_axis()**3 / mu)
    print(f"Total orbital period: {total_time} seconds")

    t = np.linspace(0, total_time, num_points)
    theta = mean_to_true(t)
    r = keplers_first_law(theta)

    h = specific_angular_momentum()
    v = h/r

    x=r*np.cos(theta)
    y=r*np.sin(theta)

    fig, ax = plt.subplots(figsize=(10, 10))
    ax.set_aspect('equal')

    earth = plt.Circle((0, 0), R, color='blue', label='Earth')
    ax.add_patch(earth)

    orbit, = ax.plot(x, y, 'r--', label='Orbit')
    spacecraft, = ax.plot([], [], 'go', markersize=10, label='Spacecraft')
    
    ax.set_title(f"Spacecraft Orbit at {(r_0)/1000:.0f} km altitude")
    ax.set_xlabel("Distance (m)")
    ax.set_ylabel("Distance (m)")
    ax.legend()
    ax.grid(True)

    def init():
        spacecraft.set_data([], [])
        return spacecraft,

    def animate(i):
        spacecraft.set_data([x[i]], [y[i]])
        print([x[i]], [y[i]])
        return spacecraft,

    anim = FuncAnimation(fig, animate, init_func=init, frames=num_points,
                         interval=1000, blit=True)

    plt.show()

if __name__ == "__main__":
    plot_orbit()