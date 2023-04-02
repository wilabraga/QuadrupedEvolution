# Greg Turk's routines for creating an articulated figure
# because URDF files are too long and painful to write

import pybullet as p
import math

# format a float for xml output, including quotes
def float_str(value):
    return "\"" + str(value) + "\""

# format a vector3 for xml output, including quotes
def vec3_str(list):
    return "\"" + str(list[0]) + " " + str(list[1]) + " " + str(list[2]) + "\""

# create a figure for pybullet, by writing to a URDF temporary file
class figure():

    # class variables
    figure_count = 1

    def __init__(self, name, density = -1, new_inertia = 1, reset_file_count = 0):
        # maybe reset the file count, so we don't make too many temporary .urdf files
        if reset_file_count:
            figure.figure_count = 1
        self.name = name
        self.body = None
        self.density = density  # -1 says to use specified mass for each link
        self.bodies = []
        self.joints = []
        self.num = figure.figure_count  # store a distinct number for each figure
        self.new_inertia_flag = new_inertia
        figure.figure_count += 1

    # write the figure info to a URDF file, and then read it in
    def create(self, pos, orn):

        # message that indicates using density instead of per-link masses
        if self.density > 0:
            print ("")
            print ("Using provided density instead of per-link masses for figure " + self.name + ".")
            print ("")

        # temporary file name, which needs to be distinct because
        # it looks like pybullet caches them, so cannot always write to temp.urdf
        # if we have more than one figure in the same scene
        filename = "temp_" + "%02d" % self.num + ".urdf"

        f = open(filename, "w")
        f.write("<?xml version=\"0.0\" ?>\n")
        f.write("<robot name=\"" + self.name + "\">\n")

        for body in self.bodies:

            # figure out volume (only really needed if we are using density)
            size = body[2]
            if body[1] == "box":
                vol = size[0] * size[1] * size[2]
            elif body[1] == "sphere":
                r = size
                vol = 4 / 3.0 * math.pi * r * r * r
            elif body[1] == "cylinder":
                h = size[0]
                r = size[1]
                vol = math.pi * r * r * h
            elif body[1] == "capsule":
                h = size[0]
                r = size[1]
                vol = math.pi * r * r * h + 4 / 3.0 * math.pi * r * r * r
            else:
                print ("error in link type: " + body[1])

            # calculate mass from volume, or use per-object specified mass
            if self.density > 0:
                mass = vol * self.density
                #print (body[0] + " volume = " + "%.6f" % vol + ", mass = " + "%.6f" % mass)
            else:
                mass = body[3]
                #print (body[0] + " mass = " + "%.6f" % mass)

            f.write("\n")
            f.write("<link name=\"" + body[0] + "\">\n")
            f.write("<contact>\n")
            f.write("  <lateral_friction value=\"1.0\"/>\n")
            f.write("  <inertia_scaling value=\"3.0\"/>\n")    # What is this?  Remove?
            f.write("</contact>\n")
            f.write("<inertial>\n")
            # the next line is problematic because I didn't use it for some of my older figures
            # (it moves the center-of-mass)
            if self.new_inertia_flag:
                f.write("  <origin xyz=" + vec3_str(body[4]) + " rpy=" + vec3_str(body[5]) + "/>\n")
            f.write("  <mass value=\"" + str(mass) +"\"/>\n")
            # inertia is re-calculated by bullet in loadURDF(), so just use dummy value
            f.write("  <inertia ixx=\"1\" ixy=\"0\" ixz=\"0\" iyy=\"1\" iyz=\"0\" izz=\"1\"/>\n")
            f.write("</inertial>\n")
            f.write("<collision>\n")
            f.write("  <origin xyz=" + vec3_str(body[4]) + " rpy=" + vec3_str(body[5]) + "/>\n")
            f.write("  <geometry>\n")
            if body[1] == "box":
                f.write("  <box size=" + vec3_str(body[2]) + "/>\n")
            elif body[1] == "sphere":
                f.write("  <sphere radius=" + float_str(body[2]) + "/>\n")
            elif body[1] == "cylinder":
                f.write("  <cylinder length=" + float_str(body[2][0]) + " radius=" + float_str(body[2][1]) + "/>\n")
            elif body[1] == "capsule":
                f.write("  <capsule length=" + float_str(body[2][0]) + " radius=" + float_str(body[2][1]) + "/>\n")
            f.write("  </geometry>\n")
            f.write("</collision>\n")            
            f.write("</link>\n")

        for joint in self.joints:
            f.write("\n")
            f.write("<joint name=\"" + joint[0] + "\" type=\"" + joint[3] + "\">\n")
            f.write("  <parent link=\"" + joint[1] + "\"/>\n")
            f.write("  <child link=\"" + joint[2] + "\"/>\n")
            f.write("  <origin xyz=" + vec3_str(joint[4]) + " rpy=" + vec3_str(joint[5]) + "/>\n")
            f.write("  <axis xyz=" + vec3_str(joint[6]) + "/>\n")
            f.write("</joint>\n")            

        f.write("\n")
        f.write("</robot>\n")
        f.close()

        # read in the URDF
        orn_quaternion = p.getQuaternionFromEuler(orn)
        self.body = p.loadURDF(filename,
                               pos, orn_quaternion)

        return (self.body)

    # specify a link for the figure
    def link(self, link_name, geom, size, mass, pos, orn):
        self.bodies.append([link_name, geom, size, mass, pos, orn])

    # specify a joint for the figure
    def joint(self, joint_name, parent, child, joint_type, pos, orn, axis):
        self.joints.append([joint_name, parent, child, joint_type, pos, orn, axis])

# some examples of various segment shapes:

#fig = figure.figure("name")
#fig.link ("sph1", "sphere", rad, mass, [x, y, z], [roll, pitch, yaw])
#fig.link ("box1", "box", [sx, sy, sz], mass, [x, y, z], [roll, pitch, yaw])
#fig.link ("cyl1", "cylinder", [len, rad], mass, [x, y, z], [roll, pitch, yaw])
#fig.link ("cap1", "capsule", [len, rad], mass, [x, y, z], [roll, pitch, yaw])
#fig.joint("joint12", "parent", "child", "continuous", [x, y, z], [roll, pitch, yaw], [xaxis, yaxis, zaxis])
#robot = fig.create([x, y, z], [roll, pitch, yaw])


