import yaml

robot_name = 'j2n6s300'
dof = 6 
fingers = 3
robot_joints = []
finger_joints = []

for i in range(1,dof+1):  
  robot_joints.append(robot_name + '_joint_' + str(i))

for i in range(1,fingers+1):  
  finger_joints.append(robot_name + '_joint_finger_' + str(i))


joint_state_controller = {'joint_state_controller':{'type':'joint_state_controller/JointStateController', 'publish_rate' : 50}}

robot_joint_controllers = joint_state_controller
for joint in robot_joints:
  #add joint position controller
  robot_joint_position_controller =  { joint + '_position_controller':
										                     {
										                     'type':'effort_controllers/JointPositionController',
										                     'joint':joint,
										                     'pid': {'p': 1000.0, 'i': 0.01, 'd': 10.0}
										                     }
										                 }
  robot_joint_controllers.update(robot_joint_position_controller)
'''
for joint in robot_joints:
  #add joint velocity controller
  robot_joint_velocity_controller =  {joint + '_velocity_controller':
												                 {
												                 'type':'velocity_controllers/JointVelocityController',
												                 'joint':joint,
												                 'pid': '{p: 1000.0, i: 0.5, d: 1.0}'
												                 }
												             }
  robot_joint_controllers.update(robot_joint_velocity_controller)
'''

#add finger joint position controllers 
for joint in finger_joints:
  finger_joint_position_controller =  { joint + '_position_controller':
										                     {
										                     'type':'effort_controllers/JointPositionController',
										                     'joint':joint,
										                     'pid': {'p': 1000.0, 'i': 0.01, 'd': 10.0}
										                     }
										                 }
  robot_joint_controllers.update(finger_joint_position_controller)


config = {'kinova': robot_joint_controllers}

with open(robot_name +'_control_gen.yaml', 'w') as outfile:
    yaml.dump(config, outfile, default_flow_style=False)

