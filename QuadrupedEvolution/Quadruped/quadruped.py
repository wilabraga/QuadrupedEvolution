import pybullet as p
import time
import random
import numpy as np
import figures
import math
import multiprocessing as mp

def polar_to_rect(r, theta):
    x = r * np.cos(theta)
    y = r * np.sin(theta)
    return x, y

def reward(xdist, ydist):
    return xdist ** 2 - ydist ** 2

def build_quadruped(torso_params, front_leg_params, hind_leg_params, init_pos = [0, 0, 0]):

    x, y, z = init_pos
    tx, ty, tz = torso_params
    tp = 0

    fig = figures.figure("quadruped", new_inertia=0, reset_file_count=1)

    fig.link("torso", "box", [tx, ty, tz], 0.3, [x, y, z], [0, tp, 0])

    fl1, fr1, fl2, fr2 = front_leg_params
    fdx1, fdz1 = polar_to_rect(tx / 2 - fr1, tp)

    front_right_leg_pos = [fdx1, -ty / 2 + fr1, -tz / 2 - fl1 / 2 - fr1 + fdz1 - 0.01]
    fig.link("frlp", "capsule", [fl1, fr1], 0.2, front_right_leg_pos, [0, 0, 0])
    fig.joint("tfrlp", "torso", "frlp", "continuous", [x, y, z], [0, 0, 0], [0, 1, 0])
    fig.link("frld", "capsule", [fl2, fr2], 0.2, [0, 0, -fl1 - fr1 - 0.01], [0, 0, 0])
    fig.joint("tfrld", "frlp", "frld", "continuous", front_right_leg_pos, [0, 0, 0], [0, 1, 0])

    front_left_leg_pos = [fdx1, ty / 2 - fr1, -tz / 2 - fl1 / 2 - fr1 + fdz1 - 0.01]
    fig.link("fllp", "capsule", [fl1, fr1], 0.2, front_left_leg_pos, [0, 0, 0])
    fig.joint("tfllp", "torso", "fllp", "continuous", [x, y, z], [0, 0, 0], [0, 1, 0])
    fig.link("flld", "capsule", [fl2, fr2], 0.2, [0, 0, -fl1 - fr1 - 0.01], [0, 0, 0])
    fig.joint("tflld", "fllp", "flld", "continuous", front_left_leg_pos, [0, 0, 0], [0, 1, 0])

    hl1, hr1, hl2, hr2 = hind_leg_params
    hdx1, hdz1 = polar_to_rect(-tx / 2 + hr1, tp)

    hind_right_leg_pos = [hdx1, -ty / 2 + hr1, -tz / 2 - hl1 / 2 - hr1 + hdz1 - 0.01]
    fig.link("hrlp", "capsule", [hl1, hr1], 0.2, hind_right_leg_pos, [0, 0, 0])
    fig.joint("thrlp", "torso", "hrlp", "continuous", [x, y, z], [0, 0, 0], [0, 1, 0])
    fig.link("hrld", "capsule", [hl2, hr2], 0.2, [0, 0, -hl1 - hr1 - 0.01], [0, 0, 0])
    fig.joint("thrld", "hrlp", "hrld", "continuous", hind_right_leg_pos, [0, 0, 0], [0, 1, 0])

    hind_left_leg_pos = [hdx1, ty / 2 - hr1, -tz / 2 - hl1 / 2 - hr1 + hdz1 - 0.01]
    fig.link("hllp", "capsule", [hl1, hr1], 0.2, hind_left_leg_pos, [0, 0, 0])
    fig.joint("thllp", "torso", "hllp", "continuous", [x, y, z], [0, 0, 0], [0, 1, 0])
    fig.link("hlld", "capsule", [hl2, hr2], 0.2, [0, 0, -hl1 - hr1 - 0.01], [0, 0, 0])
    fig.joint("thlld", "hllp", "hlld", "continuous", hind_left_leg_pos, [0, 0, 0], [0, 1, 0])

    return fig

def init(l):
    global lock
    lock = l

def run(params, visual=False, toLock=True):
    torso = [.3, .2, .1]
    front = [.05, .02, .02, .02]
    hind = [.05, .02, .02, .02]
    pos = [0, 0, 1]
    freq = params[0]
    force = params[1]
    angles = params[2:]
    creature = build_quadruped(torso, front, hind)
    d = test(creature, pos, freq, force, angles, visual=visual, toLock=toLock)
    return d


def test(fig, pos, freq, force, angles, visual=True, toLock=True):

    angles = [(t * math.pi / 180.0) for t in angles]

    if visual:
        physicsClient = p.connect(p.GUI)
        # get rid of side windows in viewer
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        # place camera
        p.resetDebugVisualizerCamera(cameraDistance=1, cameraYaw=45, cameraPitch=-30,
                                     cameraTargetPosition=[0, 0, -.15])
    else:
        physicsClient = p.connect(p.DIRECT)

    p.setGravity(0, 0, -10)
    # p.setGravity(0,0,0)  # turn off gravity while debugging robot body shape

    # create a ground plane
    plane_id = p.createCollisionShape(p.GEOM_PLANE);
    my_plane = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=plane_id, baseVisualShapeIndex=-1)

    # create the robot body
    if toLock:
        lock.acquire()
    quadruped = fig.create([pos[0], pos[1], 0.5], [0, 0, 0])
    if toLock:
        lock.release()

    time_steps = 2500

    # angle: max_angle_prox, off_prox, phase_prox, max_angle_dist, off_dist, phase_dist
    hll = angles[0:6]
    fll = angles[6:12]
    hlr = angles[12:18]
    flr = angles[18:24]
    ease_in = 150
    drop = 200

    if visual:
        time_steps = 4000
        drop = 1000

    # simulate some number of time steps
    for i in range(time_steps):


        if i >= drop:

            t = i - drop

            w = t * freq
            #hllp, hlld, fllp, flld, hlrp, hlrd, flrp, flrd
            legs = [math.sin(w + hll[2]) * hll[0] + hll[1], math.sin(w + hll[5]) * hll[3] + hll[4]]
            legs += [math.sin(w + fll[2]) * fll[0] + fll[1], math.sin(w + fll[5]) * fll[3] + fll[4]]
            legs += [math.sin(w + hlr[2]) * hlr[0] + hlr[1], math.sin(w + hlr[5]) * hlr[3] + hlr[4]]
            legs += [math.sin(w + flr[2]) * flr[0] + flr[1], math.sin(w + flr[5]) * flr[3] + flr[4]]



            # joint order: fr, fl, hr, hl | list order: hl, fl, hr, fr
            p.setJointMotorControl2(quadruped, 0, p.POSITION_CONTROL, targetPosition=legs[6], force=force)
            p.setJointMotorControl2(quadruped, 1, p.POSITION_CONTROL, targetPosition=legs[7], force=force)
            p.setJointMotorControl2(quadruped, 2, p.POSITION_CONTROL, targetPosition=legs[2], force=force)
            p.setJointMotorControl2(quadruped, 3, p.POSITION_CONTROL, targetPosition=legs[3], force=force)
            p.setJointMotorControl2(quadruped, 4, p.POSITION_CONTROL, targetPosition=legs[4], force=force)
            p.setJointMotorControl2(quadruped, 5, p.POSITION_CONTROL, targetPosition=legs[5], force=force)
            p.setJointMotorControl2(quadruped, 6, p.POSITION_CONTROL, targetPosition=legs[0], force=force)
            p.setJointMotorControl2(quadruped, 7, p.POSITION_CONTROL, targetPosition=legs[1], force=force)

        p.stepSimulation()
        if visual:
            time.sleep(1. / 240.)
            cam = p.getBasePositionAndOrientation(quadruped)[0]
            p.resetDebugVisualizerCamera(cameraDistance=1, cameraYaw=45, cameraPitch=-30,
                                         cameraTargetPosition=[cam[0], cam[1], -.15])

    # get position and orientation of robot, and find how far it travelled
    final_pos, orient = p.getBasePositionAndOrientation(quadruped)
    # reward = np.sqrt((final_pos[0] - pos[0]) ** 2 + (final_pos[1] - pos[1]) ** 2)
    reward = np.abs(final_pos[0] - pos[0]) - np.abs(final_pos[1] - pos[1])
    p.disconnect()  # disconnect from simulator

    return reward

#hll, fll, hlr, flr

# run([0.7553493568615226, 4.4317203522537865, 6.650985954054075, 125.63980925168937, 137.16641071169053, 28.388782198910523, 3.87626521687439, 166.07773266479458, 9.1642959319731, 49.95365766379568, 297.88139809238027, 42.931574036388014, 29.548079933409735, 297.0364474004982, 2.6796444713829297, 65.41842995842076, 155.97289315823326, 0.6183292889093717, 34.64524802413289, 343.3638591837004, 16.411672621816006, 44.74883760013701, 148.8767922688975, 41.749309233544714, 37.05913239263939, 122.60213244128437], True, False) #speed boat
# run([0.04, 4, 20, 0, 0, 20, 0, 0, 20, 0, 0, 20, 0, 0, 20, 0, 0, 20, 0, 0, 20, 0, 0, 20, 0, 0], True, False) #Boogie
# run([0.08, 2.562999800699517, 0.5308129986372451, 3.2431625437780083, 214.05414553120707, 15.933606471620857, 6.994922158266398, 220.5544946229809, 4.171939715583372, 11.394771959348843, 62.7997769570504, 22.34321469821636, 12.069140676862784, 350.13394797416174, 21.404825320453114, 4.707714314370831, 65.40435715845962, 19.953340697296927, 17.76259279611986, 146.89038414384, 11.937954425048297, 11.394959673383532, 91.66787018668316, 23.331331491306546, 18.284449551864878, 122.09086417913777], True, False) #Pacing
# run([0.12, 3.1140169672312923, 13.938035377834682, 17.072529214585845, 230.67155500539806, 24.99728146621527, 4.586990023896753, 235.9572385914968, 2.142490177687667, 18.847809463831336, 24.19425382081903, 24.86815405323309, 23.75789830121773, 101.01266878770306, 23.4596881854738, 24.030132309828996, 103.76115483822346, 17.13697081175537, 0.8980930232599632, 83.38416829124112, 4.155519396706625, 4.033322006181389, 342.99379551116266, 24.66508455580728, 21.603118737328238, 224.85543166608352], True, False) #Trot - Best