"""
"""


import agent_vehicle as agent
import RL
import pygame

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

agent=agent.Agent(circdesc_fname)
agent.DrawInit(zoom_factor=0.25)
agent.DrawBackground()

rl=RL.RL(agent)
rl.LoadTables("RLTables.npz")

traj=rl.GreedyTrajectory(agent.init_discrete_state, 1000)
agent.DrawTrajectory(color="green", point_size=3)


###########################################################
###########################################################
if __name__ == '__main__':
  Cont=True
  auto_mode=False
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
      rl.Learn(agent.init_discrete_state, 10000, 100, nb_retries=5)
      rl.SaveTables("RLTable.npz")
      print("MaxQtable={}".format(rl.MaxQTable()))
      agent.DrawBackground()
      traj=rl.GreedyTrajectory(agent.init_discrete_state, 1000)
      agent.DrawTrajectory(color="green", point_size=3)
      
  pygame.quit()
