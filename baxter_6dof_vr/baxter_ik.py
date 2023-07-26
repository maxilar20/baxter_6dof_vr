# Some necessary imports
import numpy as np

from ikpy.chain import Chain
from ikpy.utils import plot

# First, let's import the baxter chains
baxter_left_arm_chain = Chain.from_json_file("./baxter/baxter_left_arm.json")
baxter_right_arm_chain = Chain.from_json_file("./baxter/baxter_right_arm.json")
baxter_pedestal_chain = Chain.from_json_file("./baxter/baxter_pedestal.json")
baxter_head_chain = Chain.from_json_file("./baxter/baxter_head.json")

from mpl_toolkits.mplot3d import Axes3D

fig, ax = plot.init_3d_figure()
baxter_left_arm_chain.plot([0] * (len(baxter_left_arm_chain)), ax)
baxter_right_arm_chain.plot([0] * (len(baxter_right_arm_chain)), ax)
baxter_pedestal_chain.plot([0] * (2 + 2), ax)
baxter_head_chain.plot([0] * (4 + 2), ax)
ax.legend()

### Let's try some IK
fig, ax = plot.init_3d_figure()


target = [0.3, 0.5, 0.15]
target_orientation = [1, 0, 0]

frame_target = np.eye(4)
frame_target[:3, 3] = target

ik = baxter_left_arm_chain.inverse_kinematics_frame(frame_target)

baxter_left_arm_chain.plot(ik, ax, target=target)

baxter_right_arm_chain.plot([0] * (len(baxter_left_arm_chain)), ax)
baxter_pedestal_chain.plot([0] * (2 + 2), ax)
baxter_head_chain.plot([0] * (4 + 2), ax)
ax.legend()
