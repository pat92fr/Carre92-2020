"""
"""


import agent_vehicle as agent
import RL
import pygame
import time
#DEBUG
import numpy as np


color_red=(255,0,0)
color_green=(0,255,0)
color_blue=(0,0,255)

color_back=(64,64,64)
color_startline=(128,192,128)
color_endline=(192,0,0)
color_centerline=(255,255,255)
color_wall=(128,128,0)
color_road=(0,0,0)
color_trajectory=(128,255,128)
color_segment=(121,248,248)

circdesc_fname="CircuitTest2.txt"


###########################################################
###########################################################
if __name__ == '__main__':

  agent=agent.Agent(circdesc_fname)
  agent.DrawInit(zoom_factor=0.5)
  agent.DrawBackground()

  rl=RL.RL(agent)
  rl.LoadTables("RLTables.npz")

  agent_context=agent.NewContext(agent.init_discrete_state)
  rl.GreedyTrajectory(agent_context, 1000)
  agent.DrawTrajectory(agent_context, color="green", point_size=3)

  print(rl.QTable.shape)
  
  Cont=True
  auto_mode=False
  print
  while Cont:
    for event in pygame.event.get():
      if event.type == pygame.QUIT:
        Cont = False
      elif event.type == pygame.KEYDOWN :
        if event.key == pygame.K_ESCAPE:
          Cont = False
        elif event.key == pygame.K_q: # qwerty/azerty independant
          if auto_mode:
            auto_mode=False
          else:
            auto_mode=True
        else:
          pass

    if auto_mode:
      rl.Learn(agent.init_discrete_state, 1000, 10, nb_retries=10000)
      rl.SaveTables("RLTables.npz")
      print("#################################################")
      print("MaxQtable={}".format(rl.MaxQTable()))
      print("MaxQtable start={}".format(rl.QTable[agent.init_discrete_state].max()))
      for i in range(rl.QTable.shape[0]-2, 0, -100):
        print("MaxQtable {}={}".format(i, rl.QTable[i].max()))
      x=np.nonzero(rl.QTable == 0)
      print("{}/{} null elements in QTable, {:.3f}% filled".format(x[0].size, rl.QTable.size, 1-x[0].size/rl.QTable.size/100))
      agent.DrawBackground()
      agent_context=agent.NewContext(agent.init_discrete_state)
      rl.GreedyTrajectory(agent_context, 1000)
      agent.DrawTrajectory(agent_context, color="green", point_size=3)
      #time.sleep(3)
      
  pygame.quit()
