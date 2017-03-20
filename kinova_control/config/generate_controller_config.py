import yaml

robots = ['j2n6s300', 'm1n6s300', 'm1n6s200', 'j2n7s300', 'j2s6s300', 'j2s7s300']
joint_p = [5000,5000,5000,500,200,500,500]
joint_i = [0,0,0,0,0,0,0]
joint_d = [0,0,0,0,0,0,0]
finger_p = [10,10,10]
finger_i = [0,0,0]
finger_d = [0,0,0]

for robot_name in robots:
  dof = int(robot_name[3])
  fingers = int(robot_name[5])
  robot_joints = []
  finger_joints = []

  if (dof ==7):
    joints_p = [5000,5000,500,500,200,500,500]

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
  i = 0
  for joint in robot_joints:  
    joints.append(joint)
    gains.update({joint:
                           {'p': joint_p[i], 'i': joint_i[i], 'd': joint_d[i], 'i_clamp': 10}
                        })    
    constraints.update({joint:
                         {
                          'trajectory':0.05,
                          'goal': 0.02
                         }
                       })
    i = i + 1
  joints = {'joints': joints}
  gains = {'gains': gains}
  constraints_dic = {  'goal_time': 1.0,
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
  i=0
  for joint in finger_joints:  
    joints.append(joint)
    gains.update({joint:
                           {'p': finger_p[i], 'i': finger_i[i], 'd': finger_d[i], 'i_clamp': 1}
                        })    
    constraints.update({joint:
                         {
                          'trajectory':0.05,
                          'goal': 0.02
                         }
                       })
    i = i + 1
  joints = {'joints': joints}
  gains = {'gains': gains}
  constraints_dic = {  'goal_time': 1.0,
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
    i = i + 1
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
  i = 0
  for joint in robot_joints:    
    robot_joint_position_controller =  { 'joint_' + str(i+1) + '_position_controller':
										                       {
										                       'type':'effort_controllers/JointPositionController',
										                       'joint':joint,
										                       'pid': {'p': joint_p[i], 'i': joint_i[i], 'd': joint_d[i]}
										                       }
										                   }
    robot_controllers.update(robot_joint_position_controller)
    i = i + 1

  #add finger joint position controllers 
  i = 0
  for joint in finger_joints:  
    finger_joint_position_controller =  { 'finger_' + str(i+1) + '_position_controller':
										                       {
										                       'type':'effort_controllers/JointPositionController',
										                       'joint':joint,
										                       'pid': {'p': finger_p[i], 'i': finger_i[i], 'd': finger_d[i]}
										                       }
										                   }
    i = i + 1
    robot_controllers.update(finger_joint_position_controller)

  #####velocity
  '''
  i = 0
  for joint in robot_joints:
    #add joint velocity controller
    robot_joint_velocity_controller =  {'joint_' + str(i+1) + '_velocity_controller':
												                   {
												                   'type':'velocity_controllers/JointVelocityController',
												                   'joint':joint,
												                   'pid': '{p: 1000.0, i: 0.5, d: 1.0}'
												                   }
												               }
    i = i + 1
    robot_controllers.update(robot_joint_velocity_controller)
  '''
  ############################



  config = {robot_name: robot_controllers}

  with open(robot_name +'_control.yaml', 'w') as outfile:
      yaml.dump(config, outfile, default_flow_style=False)

