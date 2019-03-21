import matplotlib.pyplot as plt
from PIL import Image


def draw(ivp_solution, control, time):
    lambda0 = ivp_solution[:, 0]
    lambda1 = ivp_solution[:, 1]
    lambda2 = ivp_solution[:, 2]
    lambda3 = ivp_solution[:, 3]

    omega1 = control[0]
    omega2 = control[1]
    omega3 = control[2]

    fig = plt.figure()
    ax1 = fig.add_subplot(211)
    ax1.plot(time, lambda0, color='b', label='lambda0')
    ax1.plot(time, lambda1, color='r', label='lambda1')
    ax1.plot(time, lambda2, color='g', label='lambda2')
    ax1.plot(time, lambda3, color='y', label='lambda3')
    ax1.legend()
    ax1.grid()

    ax2 = fig.add_subplot(212)
    ax2.axhline(y=omega1, color='b', label='omega1')
    ax2.axhline(y=omega2, color='r', label='omega2')
    ax2.axhline(y=omega3, color='g', label='omega3')
    ax2.legend()
    ax2.grid()
    
    plt.savefig('resources/img.png')


def open(url):
    return Image.open(url)


def save(url):
    Image.open(url).show()
