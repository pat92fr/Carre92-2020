"""
Generic RL tool : vehicle agent
"""

import math
import numpy as np
import random
import circuit
import pygame

#######################################
class Agent():
  """
  Agent class adapted for vehicle
  """
  # discretisation parameters
  odo_discretisation_resol=0.1
  width_discretisation_steps=17
  heading_discretisation_steps=10
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
    self.discrete_state_odo_size=math.ceil(self.circuit.length/self.odo_discretisation_resol)
    self.discrete_state_width_size=self.width_discretisation_steps
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
    disc_width=int(round(((width+self.circuit.roadwidth/2)/self.circuit.roadwidth)*(self.width_discretisation_steps-1)))
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
    width=(disc_width/(self.width_discretisation_steps-1))*self.circuit.roadwidth-self.circuit.roadwidth/2
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
  def AddStateToTrajectory(self, state):
    # Convert state to point
    [odo, width, heading]=self.state
    pt=self.circuit.PtLineCoord2XY(odo, width)
    self.trajectory.AddPoint(pt)
    
  #####################################
  def AddDiscreteStateToTrajectory(self, discrete_state):
    state=self.UndiscretiseState(discrete_state)
    self.AddStateToTrajectory(state)
    
  #####################################
  def CutTrajectory(self, len):
    self.trajectory.Cut(len)

  #####################################
  def GetState(self):
    return self.state

  #####################################
  def GetDiscreteState(self):
    discrete_state=self.DiscretiseState(self.state)
    return discrete_state

  #####################################
  def Transition(self, action):
    # Get state point and heading
    odo=self.state[0]
    width=self.state[1]
    heading=self.state[2]
    
    steering=action[0]
    throttle=1 # throttle unused for now
    dist=throttle*0.25
    v=1 # speed unused for now
    
    # new heading
    heading=heading+(steering-0.5)*self.steer_max
    # new point
    pt=self.state_pt+np.array([math.cos(heading), math.sin(heading)])*dist
    # Compute LW coordinates of new point
    [odo, width, s_index]=self.circuit.PtXY2LineCoord(pt)
    state=[odo, width, heading]
    discrete_state=self.DiscretiseState(state)
    
    # save old state
    self.old_state=self.state
    self.old_state_pt=self.state_pt
    self.old_discrete_state=self.discrete_state
    # update state
    self.state=state
    self.state_pt=pt
    self.discrete_state=discrete_state
    # add new point to trajectory
    self.trajectory.AddPoint(pt)
    
  #####################################
  def DiscreteTransition(self, discrete_action):
    action=self.UndiscretiseAction(discrete_action)
    self.Transition(action)

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
  def DrawTrajectory(self, trajectory=None, color="green", point_size=1):
    if trajectory is None:
      trajectory=self.trajectory
    self.circuit.DrawTrajectory(self.trajectory, color, ptsize=point_size)
    self.circuit.ShowImage(self.pygame_display)

  #####################################
  def DrawQTable(self, qtable):
    pass
    
  #####################################
  def DiscreteStep(self, discrete_action):
    # Agent transition
    self.DiscreteTransition(discrete_action)
    # Compute reward, fail and done
    fail=False
    done=False
    [odo, width, heading]=self.state
    [old_odo, old_width, old_heading]=self.old_state
    
    reward=odo-old_odo
    if reward < 0 :
      # backward
      reward=0
      fail=True
      done=True
      reward=self.penalty
    if abs(width) >= self.circuit.roadwidth/2 :
      # wall crash
      fail=True
      done=True
      reward=self.penalty
    if odo >= self.circuit.length:
      done=True
    
    return [reward, fail, done]

