from mpl_toolkits.mplot3d import Axes3D
from Hexapod import *
import matplotlib.animation as animation
import matplotlib.pyplot as plt
COUNTER = 0
def animate(i):
    global COUNTER
    COUNTER += 1
    ax.clear()
    ax.set_xlim3d(-250, 250)
    ax.set_ylim3d(-250, 250)
    ax.set_zlim3d(-200, 100)
    #vHex.move_left_()
    #vHex.move_body_tripot(direct='backward')
    if COUNTER % 1 == 0:
        vHex.action(gen_forward_path_table_high(20, 4))
    #vHex.turn_body()
    #vHex.move_left_()
    #vHex.test_()
    vHex.draw_robo()

if __name__ == '__main__':
    fig = plt.figure()
    ax = Axes3D(fig)
    ax.clear()
    ax.set_aspect("auto")
    ax.set_xlim3d(-250, 250)
    ax.set_ylim3d(-250, 250)
    ax.set_zlim3d(-150, 150)

    vHex = VirtualHexapod(ax=ax)
    vHex.draw_robo()
    ani = animation.FuncAnimation(fig, animate, interval=1)
    # ani.save("pendulum.gif", writer = 'pillow', fps=30)
    plt.show()

