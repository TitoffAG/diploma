import matplotlib.pyplot as plt
from PIL import Image


def draw_ivp_and_control(ivp_solution, control, time, alpha_interval):
    lambda0 = ivp_solution[:, 0]
    lambda1 = ivp_solution[:, 1]
    lambda2 = ivp_solution[:, 2]
    lambda3 = ivp_solution[:, 3]

    omega1 = [control[0][0], control[1][0], control[2][0], control[3][0], control[4][0],
              control[5][0], control[6][0], control[7][0], control[8][0], control[9][0], control[10][0]]
    omega2 = [control[0][1], control[1][1], control[2][1], control[3][1], control[4][1],
              control[5][1], control[6][1], control[7][1], control[8][1], control[9][1], control[10][1]]
    omega3 = [control[0][2], control[1][2], control[2][2], control[3][2], control[4][2],
              control[5][2], control[6][2], control[7][2], control[8][2], control[9][2], control[10][2]]

    fig = plt.figure()
    ax1 = fig.add_subplot(211)
    ax1.plot(time, lambda0, color='b', label='lambda0')
    ax1.plot(time, lambda1, color='r', label='lambda1')
    ax1.plot(time, lambda2, color='g', label='lambda2')
    ax1.plot(time, lambda3, color='y', label='lambda3')
    ax1.legend()
    ax1.grid()

    ax2 = fig.add_subplot(212)
    ax2.plot(alpha_interval, omega1, color='b', label='omega1')
    ax2.plot(alpha_interval, omega2, color='r', label='omega2')
    ax2.plot(alpha_interval, omega3, color='g', label='omega3')
    ax2.legend()
    ax2.grid()

    plt.savefig('resources/ivp_and_control.png')


def draw_fuctional(functional, alpha_interval):
    fig, ax = plt.subplots()

    ax.plot(alpha_interval, functional, color='b', label='functional')
    ax.legend()
    ax.grid()

    fig.savefig('resources/functional.png')


def open(url):
    return Image.open(url)


def save(url):
    Image.open(url).show()
