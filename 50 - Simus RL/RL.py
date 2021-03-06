"""
Generic RL tool : learning
"""

import os
import random
import numpy as np

#######################################
class RL():
  # parameters
  learning_rate=0.2
  actualisation_rate=1
  explore_rate=0.02
  
  #####################################
  def __init__(self, agent):
    self.agent=agent
    # Shape of QTable and TerminalTable
    self.table_states_shape=self.agent.table_discrete_states_shape
    self.table_actions_shape=self.agent.table_discrete_actions_shape
    self.table_shape=self.table_states_shape+self.table_actions_shape
    # Tables of Q values and terminal states
    self.QTable=np.zeros(self.table_shape, dtype=float)
    self.FailTable=np.ndarray(self.table_shape, dtype=bool)
    self.FailTable[:]=False
    
  #####################################
  def GetQValue(self, state, action):
    qval=self.QTable(state+action)
    return qval
    
  #####################################
  def MaxQTableForState(self, state):
    sub_qtable=self.QTable[state]
    maxval=sub_qtable.max()
    return maxval
    
  #####################################
  def MaxQTable(self):
    maxval=self.QTable.max()
    return maxval

  #####################################
  def UpdateQTable(self, states_list, actions_list, rewards_list):
    #print("Rfin={:.2f} ".format(rewards_list[-1]), end="")
    traj_len=len(rewards_list)
    # if non fail trajectory, we are able to compute next Q value 
    # for last state
    if len(states_list)>traj_len:
      state=states_list[traj_len]
      next_Q=self.MaxQTableForState(state)
    else:
      # trajectory ended with a fail, so next_Q=0
      next_Q=0
    
    for i in reversed(range(0,traj_len)):
      state=states_list[i]
      action=actions_list[i]
      reward=rewards_list[i]
      # if i<3:
        # print("i={} NextQ={:.2f}/Rw={:.2f} ".format(i, next_Q, reward), end="")
      # if i >= traj_len-4:
        # print("i={} NextQ={:.2f}/Rw={:.2f} ".format(i, next_Q, reward), end="")
      # update Q table
      self.QTable[state+action]=(1-self.learning_rate)*self.QTable[state+action]+self.learning_rate*(reward+self.actualisation_rate*next_Q)
      next_Q=self.MaxQTableForState(state)
      # if i == traj_len-1:
        # print("")
        # print("  "+str(self.QTable[state])+" max="+str(next_Q))
      # if i == 0:
        # print("  Initial State="+str(states_list[i]))
    # print("")
    #print("NextQ="+str(next_Q))

  #####################################
  def SaveTables(self, fname):
    np.savez(fname, QTable=self.QTable, FailTable=self.FailTable)
    
  #####################################
  def LoadTables(self, fname):
    if os.path.isfile(fname):
      load=np.load(fname)
      self.QTable=load["QTable"]
      self.FailTable=load["FailTable"]
    
  #####################################
  def NonFailActions(self, state):
    """
    Return a list of non fail actions for a state
    """
    # search for non failing actions
    indices=np.array(np.nonzero(self.QTable[state] >= 0))
    
    NonFail_actions_list=[]
    # index is an array of arrays of matrix indexes,
    # each action is a column of this matrix.
    # Must convert each column to a tuple
    for col in range(indices.shape[1]):
      NonFail_actions_list.append(tuple(indices[:,col]))
    return NonFail_actions_list
    
  #####################################
  def RandomNonFailAction(self, state):
    non_fail_actions=self.NonFailActions(state)
    if non_fail_actions:
      action=random.choice(non_fail_actions)
    else:
      action=self.agent.RandomAction(state)
    return action
      
  #####################################
  def GreedyAction(self, state):
    """
    Return best action for a state using QTable
    """
    QTableForState=self.QTable[state]
    max=QTableForState.max()
    #print("GreedyAction : max="+str(max))
    if max == 0:
      # random action if no best action
      non_fail_actions=self.NonFailActions(state)
      action=random.choice(non_fail_actions)
    else:
      # list all best actions 
      indices=np.array(np.nonzero(QTableForState == max))
      if indices.shape[1] == 1:
        # only one best action
        action=tuple(indices[0])
      else:
        # several best actions, choose one randomly
        action=tuple(indices[:,random.randrange(indices.shape[1])])
    return action
    
  #####################################
  def GreedyTrajectory(self, agent_context, max_steps):
    state=agent_context.GetState()
    #print("GreedyTrajectory : state="+str(state))
    for step_index in range(max_steps):
      #print("{} : max={}".format(step_index, self.QTable[state].max()))
      action=self.GreedyAction(state)
      if self.QTable[state+action] < 0:
        # dead end in this state
        break
      [state, reward, fail, done]=self.agent.Step(agent_context, action)
      if reward <= 0:
        break
      if fail:
        break
      if done:
        break
    
  #####################################
  def Learn(self, start_state, nb_steps, nb_epoch, nb_retries=1):
    """
    Generate trajectories from the start_state, compute reward 
    and update QTable
    """
    
    if nb_retries<1:
      nb_retries=1
    
    # Draw agent image background
    #self.agent.DrawBackground()
    
    # start learning
    for epoch in range(nb_epoch):
      state=start_state
      states_list=[state]
      actions_list=[]
      rewards_list=[]
      agent_context=self.agent.NewContext(state)
      
      # first trajectory of retries batch uses greedy actions
      force_random_action=False
      # Retries batch
      for retry_count in range(0, nb_retries):
        #print("Learn : retry_count="+str(retry_count))
        done=False
        fail=False
        step_count=0
        while not (done or fail or step_count>=nb_steps): 
          step_count+=1
          # Action
          if force_random_action or (random.random() < self.explore_rate):
            # random action, avoiding fail states
            action=self.RandomNonFailAction(state)
          else:
            action=self.GreedyAction(state)

          # agent step with action
          [state, reward, fail, done]=self.agent.Step(agent_context, action)
          
          # save new data in tables
          actions_list.append(action)
          rewards_list.append(reward)
          
          if not fail:
            # add last state to list as it is a non fail state
            states_list.append(state)
          
        # end of retry
        
        # draw agent trajectory
        #self.agent.DrawTrajectory(agent_context, color="red", point_size=1)
        
        # update QTable.
        self.UpdateQTable(states_list, actions_list, rewards_list)
        if fail :
          # fail ? try with modifying last action
          # retry with last step 
          if retry_count < nb_retries-1 :
            # force random action in next step
            force_random_action = True
            fail=False
            # cut randomly the trajectory
            if len(rewards_list) > 4:
              cutted_len=len(rewards_list)-3 
            elif len(rewards_list) > 3:
              cutted_len=len(rewards_list)-2 
            else:
              cutted_len=len(rewards_list)-1 
            #cutted_len=random.randrange(int(len(rewards_list)*2/3), len(rewards_list))
            states_list=states_list[0:(cutted_len+1)] # states_list contains last state
            actions_list=actions_list[0:cutted_len]
            rewards_list=rewards_list[0:cutted_len]
            state=states_list[-1]
            self.agent.CutContextTrajectory(agent_context, cutted_len)
        if done:
          # No need to retry
          break
      
      print("Epoch {}/{}".format(epoch, nb_epoch), end="\r")
      # end of epoch
    print("")