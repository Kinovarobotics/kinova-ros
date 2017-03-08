import yaml

robot_name = 'j2s7s300'
dof = 7 
fingers = 3
robot_joints = []
finger_joints = []

for i in range(1,dof+1):  
  robot_joints.append(robot_name + '_joint_' + str(i))

for i in range(1,fingers+1):  
  finger_joints.append(robot_name + '_joint_finger_' + str(i))

############################ Joint state controller
joint_state_controller = {'joint_state_controller':{'type':'joint_state_controller/JointStateController', 'publish_rate' : 50}}
robot_controllers = joint_state_controller

########################### trajectory controllers 

######## effort
#joints
joints =[]
gains = {}
constraints = {}
for joint in robot_joints:  
  joints.append(joint)
  gains.update({joint:
                         {'p': 1000.0, 'i': 0.01, 'd': 2.0, 'i_clamp': 1}
                      })    
  constraints.update({joint:
                       {
                        'trajectory':0.05,
                        'goal': 0.02
                       }
                     })
joints = {'joints': joints}
gains = {'gains': gains}
constraints_dic = {  'goal_time': 20.0,
                       'stopped_velocity_tolerance': 0.02}                   			              
constraints_dic.update(constraints)              
constraints_dic = {'constraints': constraints_dic}
robot_trajectory_position_controller = {'type':'effort_controllers/JointTrajectoryController'}
robot_trajectory_position_controller.update(joints)
robot_trajectory_position_controller.update(gains)
robot_trajectory_position_controller.update(constraints_dic)
robot_trajectory_position_controller_dic = {'effort_joint_trajectory_controller': 
                                             robot_trajectory_position_controller}

robot_controllers.update(robot_trajectory_position_controller_dic)

#fingers
joints =[]
gains = {}
constraints = {}
for joint in finger_joints:  
  joints.append(joint)
  gains.update({joint:
                         {'p': 10.0, 'i': 0.01, 'd': 0.0, 'i_clamp': 1}
                      })    
  constraints.update({joint:
                       {
                        'trajectory':0.05,
                        'goal': 0.02
                       }
                     })
joints = {'joints': joints}
gains = {'gains': gains}
constraints_dic = {  'goal_time': 20.0,
                       'stopped_velocity_tolerance': 0.02}                   			              
constraints_dic.update(constraints)              
constraints_dic = {'constraints': constraints_dic}
robot_trajectory_position_controller = {'type':'effort_controllers/JointTrajectoryController'}
robot_trajectory_position_controller.update(joints)
robot_trajectory_position_controller.update(gains)
robot_trajectory_position_controller.update(constraints_dic)
robot_trajectory_position_controller_dic = {'effort_finger_trajectory_controller': 
                                             robot_trajectory_position_controller}

robot_controllers.update(robot_trajectory_position_controller_dic)

######### velocity
'''
joints =[]
gains = {}
constraints = {}
for joint in robot_joints:  
  joints.append(joint)
  gains.update({joint:
                         {'p': 1000.0, 'i': 0.01, 'd': 2.0, 'i_clamp': 1}
                      })    
  constraints.update({joint:
                       {
                        'trajectory':0.05,
                        'goal': 0.02
                       }
                     })
joints = {'joints': joints}
gains = {'gains': gains}
constraints_dic = {  'goal_time': 20.0,
                       'stopped_velocity_tolerance': 0.02}                   			              
constraints_dic.update(constraints)              
constraints_dic = {'constraints': constraints_dic}
robot_trajectory_position_controller = {'type':'velocity_controllers/JointTrajectoryController'}
robot_trajectory_position_controller.update(joints)
robot_trajectory_position_controller.update(gains)
robot_trajectory_position_controller.update(constraints_dic)
robot_trajectory_position_controller_dic = {'velocity_joint_trajectory_controller': 
                                             robot_trajectory_position_controller}

robot_controllers.update(robot_trajectory_position_controller_dic)
'''
############ position
'''
joints =[]
gains = {}
constraints = {}
for joint in robot_joints:  
  joints.append(joint)
  gains.update({joint:
                         {'p': 1000.0, 'i': 0.01, 'd': 2.0, 'i_clamp': 1}
                      })    
  constraints.update({joint:
                       {
                        'trajectory':0.05,
                        'goal': 0.02
                       }
                     })
joints = {'joints': joints}
gains = {'gains': gains}
constraints_dic = {  'goal_time': 20.0,
                       'stopped_velocity_tolerance': 0.02}                   			              
constraints_dic.update(constraints)              
constraints_dic = {'constraints': constraints_dic}
robot_trajectory_position_controller = {'type':'position_controllers/JointTrajectoryController'}
robot_trajectory_position_controller.update(joints)
robot_trajectory_position_controller.update(gains)
robot_trajectory_position_controller.update(constraints_dic)
robot_trajectory_position_controller_dic = {'position_joint_trajectory_controller': 
                                             robot_trajectory_position_controller}

robot_controllers.update(robot_trajectory_position_controller_dic)
'''
########################### Joint by joint controllers

#####position
#add joint position controller
for joint in robot_joints:  
  robot_joint_position_controller =  { joint + '_position_controller':
										                     {
										                     'type':'effort_controllers/JointPositionController',
										                     'joint':joint,
										                     'pid': {'p': 1000.0, 'i': 0.01, 'd': 10.0}
										                     }
										                 }
  robot_controllers.update(robot_joint_position_controller)

#add finger joint position controllers 
for joint in finger_joints:
  finger_joint_position_controller =  { joint + '_position_controller':
										                     {
										                     'type':'effort_controllers/JointPositionController',
										                     'joint':joint,
										                     'pid': {'p': 1000.0, 'i': 0.01, 'd': 10.0}
										                     }
										                 }
  robot_controllers.update(finger_joint_position_controller)

#####velocity
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
  robot_controllers.update(robot_joint_velocity_controller)
'''
############################



config = {robot_name: robot_controllers}

with open(robot_name +'_control_gen.yaml', 'w') as outfile:
    yaml.dump(config, outfile, default_flow_style=False)

