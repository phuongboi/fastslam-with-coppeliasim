import numpy as np
import matplotlib.pyplot as plt
import cv2
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 6))


# Bresenhams Line Generation Algorithm
# ref: https://www.geeksforgeeks.org/bresenhams-line-generation-algorithm/
def bresenham(x1, y1, x2, y2, w, h):
    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
    dx = abs(x2 - x1)
    dy = abs(y2 - y1)

    steep = 0
    if dx <= dy:
        steep = 1
        x1, y1 = y1, x1
        x2, y2 = y2, x2
        dx, dy = dy, dx

    pk = 2 * dy - dx

    loc = []
    for _ in range(0, dx + 1):
        if (x1 < 0 or y1 < 0) or (steep == 0 and (x1 >= h or y1 >= w)) or (steep == 1 and (x1 >= w or y1 >= h)):
            break

        if steep == 0:
            loc.append([x1, y1])
        else:
            loc.append([y1, x1])

        if x1 < x2:
            x1 = x1 + 1
        else:
            x1 = x1 - 1

        if (pk < 0):
            if steep == 0:
                pk = pk + 2 * dy
            else:
                pk = pk + 2 * dy
        else:
            if y1 < y2:
                y1 = y1 + 1
            else:
                y1 = y1 - 1

            pk = pk + 2 * dy - 2 * dx

    return loc


def wrapAngle(radian):
    radian = radian - 2 * np.pi * np.floor((radian + np.pi) / (2 * np.pi))
    return radian


def degree2radian(degree):
    return degree / 180 * np.pi


def prob2logodds(prob):
    return np.log(prob / (1 - prob + 1e-15))


def logodds2prob(logodds):
    return 1 - 1 / (1 + np.exp(logodds) + 1e-15)


def normalDistribution(mean, variance):
    return np.exp(-(np.power(mean, 2) / variance / 2.0) / np.sqrt(2.0 * np.pi * variance))


def create_rotation_matrix(theta):
    R = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta), np.cos(theta)]
    ])
    R_inv = np.linalg.inv(R)

    return R, R_inv


def absolute2relative(position, states):
    x, y, theta = states
    pose = np.array([x, y])

    R, R_inv = create_rotation_matrix(theta)
    position = position - pose
    position = np.array(position) @ R_inv.T

    return position


def relative2absolute(position, states):
    x, y, theta = states
    pose = np.array([x, y])

    R, R_inv = create_rotation_matrix(theta)
    position = np.array(position) @ R.T
    position = position + pose

    return position
# def compute_odometry(v_left, v_right)

def visualize(robot, particles, best_particle, radar_list, step, title, output_path, visualize=False):
    ax1.clear()
    ax2.clear()
    fig.suptitle("{}\n\n number of particles:{}, step:{}".format(title, len(particles), step + 1))
    ax1.set_title("Estimated by Particles")
    ax2.set_title("Ground Truth")
    ax1.axis("off")
    ax2.axis("off")

    grid_size = best_particle.grid_size
    ax1.set_xlim(0, grid_size[1])
    ax1.set_ylim(0, grid_size[0])

    grid_size = robot.grid_size
    ax2.set_xlim(0, grid_size[1])
    ax2.set_ylim(0, grid_size[0])

    # draw map
    world_map = 1 - best_particle.grid
    ax1.imshow(world_map, cmap='gray')
    world_map = 1 - robot.grid
    ax2.imshow(world_map, cmap='gray')

    # draw radar beams
    for (x, y) in radar_list:
        ax2.plot(x, y, "yo", markersize=1)

    # draw tragectory
    true_path = np.array(robot.trajectory)
    ax2.plot(true_path[:, 0], true_path[:, 1], "b")
    estimated_path = np.array(best_particle.trajectory)
    ax1.plot(estimated_path[:, 0], estimated_path[:, 1], "g")

    # draw particles position
    for p in particles:
        ax1.plot(p.x, p.y, "go", markersize=1)

    # draw robot position
    ax2.plot(robot.x, robot.y, "bo")

    if step % 10 == 0:
        plt.savefig('{}_{}.png'.format(output_path, step), bbox_inches='tight')

    if visualize:
        plt.draw()
        plt.pause(0.01)

def visualize_opencv(robot, particles, best_particle, radar_list, step, title, output_path, recorder):
    world_map = 1 - best_particle.grid
    empty_map = np.ones((150, 150))
    img = np.stack((world_map,)*3, axis=-1)
    img1 = np.stack((empty_map,)*3, axis=-1)

    # draw particle
    for p in particles:
        cv2.circle(img, (int(p.x), int(p.y)), 1, (0, 0, 128), 1)
    # draw robot position
    cv2.circle(img1, (int(robot.x), int(robot.y)), 3, (0, 0, 255), 1)
    # draw robot orientation
    x = robot.x + np.cos(robot.theta)*3
    y = robot.y + np.sin(robot.theta)*3
    cv2.line(img1, (int(robot.x), int(robot.y)),(int(x), int(y)), (0, 0, 255), 1)
    # draw center map
    cv2.circle(img, (75, 75), 1, (255, 0, 0), 1)
    cv2.circle(img1, (75, 75), 1, (255, 0, 0), 1)
    # draw 1 m2 square grid
    for i in (50, 100):
        cv2.line(img, (0, i), (150, i), (0, 255, 0), 1)
        cv2.line(img, (i, 0), (i, 150), (0, 255, 0), 1)
        cv2.line(img1, (0, i), (150, i), (0, 255, 0), 1)
        cv2.line(img1, (i, 0), (i, 150), (0, 255, 0), 1)

    for (x, y) in radar_list:
        cv2.circle(img1, (x, y), 1, (128, 128, 0), 1)

    img = cv2.resize(img, (300,300))
    img1 = cv2.resize(img1, (300, 300))

    concated_img = np.concatenate((img, img1), axis=1)
    print(concated_img.shape)
    cv2.imshow("slam", concated_img)
    recorder.write(cv2.cvtColor((concated_img*255).astype(np.uint8), cv2.COLOR_RGB2BGR))


    # if cv2.waitKey(25) & 0xFF == ord('q'):
    #     break
