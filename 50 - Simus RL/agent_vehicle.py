"""
Generic RL tool : vehicle agent
"""

import math
import numpy as np
import random
import circuit
import pygame

#######################################
#######################################
class AgentContext():
  #####################################
  def __init__(self, init_state, discrete_init_state):
    self.state=init_state
    self.discrete_state=discrete_init_state
    self.states_list=[init_state]
  #####################################
  def GetState(self):
    return self.discrete_state
  #####################################
  def AddState(self, state, discrete_state):
    self.states_list.append(state)
    self.state=state
    self.discrete_state=discrete_state
  #####################################
  def CutTrajectory(self, len):
    self.states_list=self.states_list[0:len+1]
    self.state=self.states_list[-1]
  
#######################################
#######################################
class Agent():
  """
  Agent class adapted for vehicle
  """
  # discretisation parameters
  odo_discretisation_resol=0.025
  #width_discretisation_steps=17
  width_discretisation_resol=0.025
  heading_discretisation_steps=36
  throttle_discretisation_steps=11
  steering_discretisation_steps=11

  delta_t=0.01 # time unit in sec, for discrete integration
  
  # physic parameters, dont modify
  aero_coeff=7.5e3
  power_coeff=60
  
  """
  # mechanical parameters for Tesla model 3
  vehicle_mass=1.5e6 # mass of vehicle, in grams
  vehicle_cx=0.3    # drag coefficient
  vehicle_surf=1.8*1.4 # surface of vehicle
  motor_pwr=275*763 # motor power in W
  """
  
  # mechanical parameters for brushless 1/10 RC 
  vehicle_mass=1.5e3 # mass of vehicle, in grams
  vehicle_cx=0.99    # drag coefficient
  vehicle_surf=1.8*1.4/100 # surface of vehicle
  motor_pwr=500 # motor power in W
  
  # model constants, computed from model and physic parameters 
  k_aero=-0.5*vehicle_cx*vehicle_surf*aero_coeff/vehicle_mass  
  k_propulsion=power_coeff*motor_pwr/vehicle_mass
  
  # throttle parameters
  throttle_steps=10
  
  # steering parameters
  steer_max=math.pi/2
  
  # Reward parameters
  bonus_reward=10000000
  penalty=-10000
  
  #####################################
  def __init__(self, circdesc_fname):
  
    # circuit initialisation
    self.circuit=circuit.Circuit()
    self.circuit.LoadDescription(circdesc_fname)
    self.circuit.GenerateSegments()
    # self.circuit.CreateImage(zoom_factor=0.25)
    # self.circuit.DrawImage()
    # self.pygame_display=pygame.display.set_mode((self.circuit.image.shape[0], self.circuit.image.shape[1]))

    # circuit discretisation
    self.discrete_state_odo_size=math.ceil(self.circuit.length/self.odo_discretisation_resol)+1
    self.discrete_state_width_size=math.ceil(self.circuit.roadwidth/self.width_discretisation_resol)+1
    self.discrete_state_head_size=self.heading_discretisation_steps
    self.discrete_action_steering_size=self.steering_discretisation_steps
    
    self.table_discrete_states_shape=(self.discrete_state_odo_size, self.discrete_state_width_size, self.discrete_state_head_size)
    self.table_discrete_actions_shape=(self.discrete_action_steering_size,)
    
    # set default initial state
    self.init_state_pt=self.circuit.start_pt
    [odo, width, segment_index]=self.circuit.PtXY2LineCoord(self.init_state_pt)
    self.init_state=[odo, width, self.circuit.start_heading]
    self.init_discrete_state=self.DiscretiseState(self.init_state)
    
    
    # set initial state of vehicle
    
    self.ResetState()
        
  #####################################
  def ResetState(self, state=None):
    """
    Reset vehicle state 
    If state is not provided, reset at circuit start point
    """
    if state is None:
      # set state from circuit start point and heading
      self.state_pt=self.init_state_pt
      self.state=self.init_state
      self.discrete_state=self.init_discrete_state
    else:
      self.state=self.init_state
      [odo, width, heading]=state
      [pt, s_idx]=self.circuit.PtLineCoord2XY(odo, width, None)
      self.state_pt=pt
      self.discrete_state=self.DiscretiseState(self.state)
      
    # reset trajectory
    self.trajectory=circuit.Trajectory(self.state_pt)

  #####################################
  def ResetDiscreteState(self, discrete_state=None):
    """
    Reset vehicle state 
    If state is not provided, reset at circuit start point
    """
    if discrete_state is None:
      # set state from circuit start point and heading
      self.state_pt=self.init_state_pt
      self.state=self.init_state
      self.discrete_state=self.init_discrete_state
    else:
      self.discrete_state=discrete_state
      self.state=self.UndiscretiseState(discrete_state)
      [odo, width, heading]=self.state
      [pt, s_idx]=self.circuit.PtLineCoord2XY(odo, width, None)
      self.state_pt=pt
      
    # reset trajectory
    self.trajectory=circuit.Trajectory(self.state_pt)

  #####################################
  def DrawInit(self, zoom_factor=1):
    self.circuit.CreateImage(zoom_factor=zoom_factor)
    self.pygame_display=pygame.display.set_mode((self.circuit.image.shape[0], self.circuit.image.shape[1]))
    self.circuit.DrawImage()

  #####################################
  def DrawBackground(self):
    self.circuit.DrawImage()
    self.circuit.ShowImage(self.pygame_display)
    
  #####################################
  def DrawTrajectory(self, context, color="green", point_size=1):
    [odo, width, heading]=context.states_list[0]
    [pt, s_idx]=self.circuit.PtLineCoord2XY(odo, width, None)
    trajectory=circuit.Trajectory(pt)
    for state in context.states_list[1:]:
      [odo, width, heading]=state
      [pt, s_idx]=self.circuit.PtLineCoord2XY(odo, width, None)
      trajectory.AddPoint(pt)
    self.circuit.DrawTrajectory(trajectory, color, ptsize=point_size)
    self.circuit.ShowImage(self.pygame_display)

  #####################################
  def SetState(self, state):
    # set LW state values
    self.state=state
    # compute XY coordinates of state
    odo=state[0]
    width=state[1]
    self.state_pt=self.circuit.PtLineCoord2XY(odo, width)
    # compute discrete state
    self.discrete_state=self.DiscretiseState(self.state)
    
  #####################################
  def SetDiscreteState(self, discrete_state):
    self.discrete_state=discrete_state
    # compute undiscretised state
    self.state=self.UndiscretiseState(self.discrete_state)
    # compute XY coordinates of state
    odo=self.state[0]
    width=self.state[1]
    [self.state_pt, dummy]=self.circuit.PtLineCoord2XY(odo, width)

  #####################################
  def DiscretiseState(self, state):
    """
    Compute discretisation of state
    """
    [odo, width, heading]=state
    disc_odo=int(round((odo)/self.odo_discretisation_resol))
    disc_width=int(round((width+self.circuit.roadwidth/2)/self.width_discretisation_resol))
    #disc_width=int(round(((width+self.circuit.roadwidth/2)/self.circuit.roadwidth)*(self.width_discretisation_steps-1)))
    disc_heading=int(round(((heading%(2*math.pi))/(2*math.pi))*(self.heading_discretisation_steps-1)))
    return (disc_odo, disc_width, disc_heading)
    
  #####################################
  def DiscretiseAction(self, action):
    """
    Compute discretisation of action
    """
    steering=action
    disc_action=int(steering*(self.steering_discretisation_steps-1))
    return (disc_action,)

  #####################################
  def UndiscretiseState(self, discrete_state):
    [disc_odo, disc_width, disc_heading]=discrete_state
    odo=disc_odo*self.odo_discretisation_resol
    width=disc_width*self.width_discretisation_resol-self.circuit.roadwidth/2
    #width=(disc_width/(self.width_discretisation_steps-1))*self.circuit.roadwidth-self.circuit.roadwidth/2
    heading=disc_heading*(2*math.pi/self.heading_discretisation_steps)
    return [odo, width, heading]
    
  #####################################
  def UndiscretiseAction(self, discrete_action):
    disc_steering=discrete_action[0]
    steering=disc_steering/(self.steering_discretisation_steps-1)
    return [steering]
    
  #####################################
  def RandomAction(self, discrete_state):
    steering=random.randrange(self.steering_discretisation_steps)
    return (steering,)
    
  #####################################
  def Transition(self, state, action):
    """
    Agent transition from a state with an action
    """
    # Get state point and heading
    odo=state[0]
    width=state[1]
    heading=state[2]
    [pt, s_idx]=self.circuit.PtLineCoord2XY(odo, width)
    
    steering=action[0]
    throttle=1 # throttle unused for now

    dist=throttle*0.2
    v=1 # speed unused for now
    
    # new heading
    new_heading=heading+(steering-0.5)*self.steer_max
    # new point
    new_pt=pt+np.array([math.cos(heading), math.sin(heading)])*dist
    
    # Compute LW coordinates of new point
    [new_odo, new_width, s_index]=self.circuit.PtXY2LineCoord(new_pt)
    new_state=(new_odo, new_width, new_heading)
    return new_state

  #####################################
  def Step(self, context, discrete_action):
    """
    Performs an agent step
    """
    state=context.state
    [odo, width, heading]=state
    action=self.UndiscretiseAction(discrete_action)
    # Agent transition
    new_state=self.Transition(state, action)
    [new_odo, new_width, new_heading]=new_state

    # consider circuit begining crossing for odo computation
    if self.circuit.length/2 < odo and odo <= self.circuit.length and 0 <= new_odo and new_odo < self.circuit.length/2 :
      new_odo=new_odo+self.circuit.length
    
    dist=new_odo-odo
      
    # reward computation including bonus and penalty
    #print("Step : newstate="+str(new_state)+" circuit length"+str(self.circuit.length))
    fail=False
    done=False
    if dist < 0 :
      # backward
      fail=True
      reward=self.penalty
      #print("backward")
    elif abs(new_width) >= self.circuit.roadwidth/2 :
      # wall crash
      fail=True
      reward=self.penalty
      #print("wall crash")
    elif dist > 0.5:
      # wall jumped !
      fail=True
      reward=self.penalty
      #print("wall jump")
    elif new_odo >= self.circuit.length:
      done=True
      # final reward = bonus - trajectory length
      reward=self.bonus_reward-len(context.states_list)
      print("done")
    else:
      # very little reward for each step such that circuit completion is
      # better than a long walk before wall crash
      reward=0.0001
      
    new_discrete_state=self.DiscretiseState(new_state)
    context.AddState(new_state, new_discrete_state)

    return [new_discrete_state, reward, fail, done]

  #####################################
  def NewContext(self, discrete_init_state):
    init_state=self.UndiscretiseState(discrete_init_state)
    context=AgentContext(init_state, discrete_init_state)
    return context
    
  #####################################
  def CutContextTrajectory(self, context, len):
    state=context.states_list[len+1]
    context.states_list=context.states_list[0:len+1]
    discrete_state=self.DiscretiseState(state)
    context.state=state
    context.discrete_state=discrete_state
    
    
    