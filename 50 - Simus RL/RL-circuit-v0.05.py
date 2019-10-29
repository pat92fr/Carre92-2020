
import os
import re
import math
import numpy as np
import skimage.draw
import random
import pygame
import time

#########################################################
def Expo(x, c):
  if x == 1:
    res=1
  else:
    res=np.power(1-np.exp(-1/(1-x*x))/np.exp(-1), c)
  print("Expo : x={:.2f}=>r={:.2f}".format(x, res))
  return res
#########################################################
def UpdateCmd(val, step):
  x=2*(val-0.5)
  y=x
  if step > 0:
    if x <= 1-step:
      y=x+step
  else:
    if x >= -1-step:
      y=x+step
  return y/2+0.5
  
#########################################################
def LineIntersection(sAB, sCD):
  AB=sAB.p2-sAB.p1
  CD=sCD.p2-sCD.p1
  
  mat=np.array([[AB[0], -CD[0]],[AB[1],-CD[1]]])
  X=sCD.p1-sAB.p1
  sol=np.linalg.solve(mat, X)
  return sol
  
#########################################################
def DoesSegmentsIntersect(sAB, sCD):
  [alpha, beta]=LineIntersection(sAB, sCD)
  if alpha>=0 and alpha<=1 and beta>=0 and beta<=1:
    return True
  else:
    return False

#########################################################
def Params2Int(text):
  """
  Return a list of int from a string "x1,...,xn)"
  """
  str=text.replace("(","").replace(")","")
  l=str.split(",")
  res=[]
  for s in l:
    res.append(int(s))
  return res

#########################################################
def GetPointCoordinatesFromText(text):
  """
  Return [x,y] from a string "(x,y)"
  """
  [x_s,y_s]=text[1:-1].split(",")
  x=float(x_s)
  y=float(y_s)
  return [x,y]

#########################################################
def Get2PointCoordinatesFromText(text):
  """
  Return [[x1, y1],[x2,y2] from a string "(x1,y1)(x2,y2)"
  """
  t=text[1:-1]
  pts=t.split(")(")
  [x_s,y_s]=pts[0].split(",")
  xy1=[float(x_s), float(y_s)]    
  [x_s,y_s]=pts[1].split(",")
  xy2=[float(x_s), float(y_s)]    
  return [xy1, xy2]

#########################################################
def PartDescHalfCircle(A,B):
  """ 
  Return circuit part description for a half circle arc, turning right
  for circle arc turning left, swap A and B
  A,B = start and end points of arc, arrays of 2 int
  """
  # Start and end points of arc
  AB=B-A
  AB_length=np.linalg.norm(AB)
  radius=AB_length/2
  # Center = middle of [A,B]
  C=A+AB/2
  # Circle arc length
  length=radius*math.pi
  # Initial heading
  heading=Vector.Heading(AB)+math.pi/2
  return [C,radius,length,heading]

#########################################################
def PartDescCircleArc(A,B,theta):
  """ 
  Return circuit part description for a circle arc, turning right
  for circle arc turning left, swap A and B
  A,B = start and end points of arc, arrays of 2 int
  theta: angle of arc, < pi/2
  """
  # Start and end points of arc
  AB=B-A
  AB_length=np.linalg.norm(AB)
  # middle of [A,B]
  M=A+AB/2
  # unitary vector colinear to AB
  u_colin=AB/AB_length
  # unitary vector orthogonal to AB
  u_orth=np.array([AB[1], -AB[0]])/AB_length
  # Radius of circle
  alpha=(math.pi-theta)/2 # angle CAB
  sin_alpha=math.sin(alpha)
  cos_alpha=math.cos(alpha)
  radius=AB_length/2/cos_alpha
  # Center of circle : 
  # direction = rotate AB with angle alpha
  # length = Radius
  # Rotation matrix, angle=-alpha
  rotmatrix=np.array([[cos_alpha, sin_alpha], [-sin_alpha, cos_alpha]])
  AC=rotmatrix.dot(u_colin)*radius
  C=A+AC
  # Circle arc length
  length=radius*theta
  # Initial heading
  heading=Vector.Heading(AC)+math.pi/2
  return [C,radius,length,heading]

###########################################################
def CircleSegmentation(center, radius, startpt, init_angle, angle, nb_of_segments):
  """
  Generate a segments list for a circle arc
  """
  #print("!!!!! CircleSegmentation")
  #print("  center="+str(center))
  #print("  init_angle"+str(init_angle))
  #print("  angle"+str(angle))
  delta_angle=angle/nb_of_segments
  angle_i=init_angle
  angle_e=init_angle+delta_angle
  pi=center+radius*np.array([math.cos(angle_i), math.sin(angle_i)])
  segments=[]
  for i in range(0,nb_of_segments):
    pe=center+radius*np.array([math.cos(angle_e), math.sin(angle_e)])
    s=Segment(p1=pi, p2=pe)
    segments.append(s)
    angle_e=angle_e+delta_angle
    pi=pe
  return segments

###########################################################
###########################################################
class Vector():
  #########################################################
  def Norm(u):
    return np.linalg.norm(u)
  #########################################################
  def Orthogonal(u):
    return np.array([-u[1], u[0]])
  #########################################################
  def UnitVectorAngle(angle):
    return np.array([math.cos(angle), math.sin(angle)])
  #########################################################
  def Heading(u):
    """
    return the angle of a vector with horizontal line
    """
    x=u[0]
    y=u[1]
    if x == 0:
      if y >=0:
        angle=math.pi/2
      else:
        angle=-math.pi/2
    else:
      angle=math.atan(y/x)
      if x < 0:
        angle=angle+math.pi
    return angle

###########################################################
###########################################################
class Segment():
  #########################################################
  def __init__(self, p1, p2=None, length=None, heading=None):
    if p2 is not None:
      # segment definition with p1 and p2
      self.p1=p1
      self.p2=p2
      u=p2-p1
      nu=Vector.Norm(u)
      if nu == 0:
        raise Exception("Definition of null segment")
      self.length=nu
      self.heading=Vector.Heading(u)
      self.u=u/nu
      self.normal=Vector.Orthogonal(self.u)
    else:
      # segment definition with p1, length and heading
      self.heading=heading
      self.length=length
      if length == 0:
        raise Exception("Definition of null segment")
      self.p1=p1
      self.p2=p1+np.array([int(length*math.cos(heading)), int(length*math.sin(heading))], dtype=int)
      u=self.p2-p1
      self.u=u/length
      self.normal=Vector.Orthogonal(self.u)
      
  #########################################################
  def __add__(self, other):
    if isinstance(other, np.ndarray): # translate a segment with a point/vector
      res=Segment(p1=self.p1+other, p2=self.p2+other)
      return res
    else:
      raise Exception("Cannot add segment with "+type(other))
      
  #########################################################
  def __sub__(self, other):
    if isinstance(other, np.ndarray): # translate a segment with a point/vector
      #print("Segment-point : "+str(self)+" sub ({},{})".format(other[0], other[1]))
      res=Segment(p1=self.p1-other, p2=self.p2-other)
      return res
    else:
      raise Exception("Cannot substract segment with "+type(other))

  #########################################################
  def __str__(self):
    return "({:.1f},{:.1f})->({:.1f},{:.1f})".format(self.p1[0], self.p1[1], self.p2[0], self.p2[1])
    
  #########################################################
  def Draw(self):
    rc=skimage.draw.line(int(self.p1[0]), int(self.p1[1]), int(self.p2[0]), int(self.p2[1]))
    return rc
    
  #########################################################
  def Side(self, p):
    """ 
    return +1 or -1 depending on the side of p of the line (AB)
    """ 
    PA=p-self.p1
    PH=PA[0]*self.normal[0]+PA[1]*self.normal[1]
    if PH > 0:
      return 1
    else :
      return -1

  #########################################################
  def DistToPoint(self, p):
    """
    return distance of p to segment²
    """
    # H = projeté de p sur la droite (AB)
    PA=self.p1-p
    PB=self.p2-p
    PH=PA[0]*self.normal[0]+PA[1]*self.normal[1]
    dPH=Vector.Norm(PH)
    dPA=Vector.Norm(PA)
    dPB=Vector.Norm(PB)
    dHA=math.sqrt(dPA*dPA-dPH*dPH)
    dHB=math.sqrt(dPB*dPB-dPH*dPH)
    dAB=Vector.Norm(self.p2-self.p1)
    #print("#### DistToPoint : p={:.1f},{:.1f}".format(p[0], p[1])+"s="+str(self))
    if dHA < dAB and dHB < dAB:
      # H is on the segment
      res = dPH
    else :
      res = min(dPA, dPB)
    #print("dPA={:.1f}, dPH={:.1f} dPB={:.1f}".format(dPA, dPH, dPB))
    return res

###########################################################
###########################################################
class Circuit():
  #########################################################
  def __init__(self):
    # nb of segment for a whole circle
    # shall be multiple of 8
    self.nb_segments_in_circle=16
    self.trajectory_points=[]
    self.trajectory_segments=[]
      
  #########################################################
  def LoadDescription(self, fname):
    """
    Load a description from a file
    """
    if not os.path.isfile(fname):
      raise Exception("Error while loading circuit description : file {} not found".format(fname))
      return 
    print("Loading description : {}".format(fname))
    self.parts_description=[]
    cumulated_length=0
    f=open(fname, "r")
    re_comment="^\s*#"
    re_emptyline="^\s*$"
    line_idx=0
    
    self.startline_dist=None
    self.endline_dist=None
    
    for l in f.readlines():
      l=l.strip()
      line_idx+=1
      if re.match(re_comment, l):
        continue
      if re.match(re_emptyline, l):
        continue
      cmdline=l.split(maxsplit=1)
      cmd=cmdline[0].lower()
      if len(cmdline) > 0:
        params=cmdline[1].replace(" ", "")
      
      ####################
      if cmd == "name":
        self.name = params
       
      elif cmd == "width":
        self.width = int(params)
        
      elif cmd == "height":
        self.height = int(params)
        
      elif cmd == "margins":
        self.margins = int(params)
        
      elif cmd == "roadwidth":
        self.roadwidth = int(params)
        
      elif cmd == "changeheading":
        changeheading = np.deg2rad(int(params))
        cur_start_heading+=changeheading
        cur_end_heading+=changeheading

      elif cmd == "startpoint":
        [x,y]=Params2Int(params)
        cur_start_pt=np.array([x,y], dtype=int)
        cur_end_pt=cur_start_pt
        self.start_pt=cur_start_pt
        self.end_pt=cur_end_pt
        
      elif cmd == "startline":
        self.startline_dist=int(params)
        
      elif cmd == "endline":
        self.endline_dist=int(params)

      elif cmd == "initialheading":
        cur_start_heading=np.deg2rad(int(params))
        cur_end_heading=cur_start_heading
        self.start_heading=cur_start_heading
        
      elif cmd == "line":
        # line parameters
        length=int(params)
        vector=length*Vector.UnitVectorAngle(cur_end_heading)
        start_pt=cur_end_pt
        end_pt=start_pt+vector
        # update current data
        cur_start_pt=start_pt
        cur_end_pt=end_pt
        cumulated_length+=length
        partdesc={'type':"line", 'startpt':start_pt, 'endpt':end_pt, 'length':length, 'cumulated_length':cumulated_length, 'heading_init':cur_start_heading}
        self.parts_description.append(partdesc)
        
      elif cmd == "curve":
        # curve parameters
        [radius, angle]=Params2Int(params)
        if angle > 0:
          turn=1
        else:
          turn=-1
        angle=np.deg2rad(angle)
        cur_start_heading=cur_end_heading
        cur_end_heading=cur_start_heading-angle
        if turn > 0:
          P1C=radius*Vector.UnitVectorAngle(cur_start_heading-math.pi/2)
          P2C=radius*Vector.UnitVectorAngle(cur_end_heading-math.pi/2)
        else:
          P1C=radius*Vector.UnitVectorAngle(cur_start_heading+math.pi/2)
          P2C=radius*Vector.UnitVectorAngle(cur_end_heading+math.pi/2)          
        start_pt=cur_end_pt
        center=start_pt+P1C
        end_pt=center-P2C
        cur_start_pt=start_pt
        cur_end_pt=end_pt
        length=radius*abs(angle)
        cumulated_length+=length
        partdesc={'type':"curve", 'startpt':cur_start_pt, 'endpt':cur_end_pt, 'length':length, 'cumulated_length':cumulated_length, \
                  'heading_init':cur_start_heading, 'heading_end':cur_end_heading, 'turn':turn, 'center':center, 'radius':radius, 'angle':angle}
        self.parts_description.append(partdesc)
      else:
        raise Exception("Command {} unknown".format(cmd))    
      
    # Some verifications
    if self.startline_dist is not None:
      if not (self.startline_dist > 0 and self.startline_dist <= self.parts_description[0]["length"]):
        print("!!! Start line is not on the first line")
    if self.endline_dist is not None:
      if not (self.endline_dist > 0 and self.endline_dist <= self.parts_description[0]["length"]):
        print("!!! End line is not on the first line")
    a=self.parts_description[0]["startpt"]
    b=self.parts_description[-1]["endpt"]
      
  #########################################################
  def GenerateSegments(self):
    self.cl_segments=[] # center line segments
    self.lw_segments=[] # left wall segments
    self.rw_segments=[] # right wall segments
    for p in self.parts_description:
      type = p["type"]
      if type=="line":
        #print("### Line : ")
        #print(p)
        cl_seg=Segment(p1=p["startpt"], p2=p["endpt"])
        self.cl_segments.append(cl_seg)
        lw_seg=cl_seg+(self.roadwidth/2)*cl_seg.normal
        self.lw_segments.append(lw_seg)
        rw_seg=cl_seg-(self.roadwidth/2)*cl_seg.normal
        self.rw_segments.append(rw_seg)
      elif type=="curve":
        #print("### Curve : ")
        #print(p)
        center=p["center"]
        radius=p["radius"]
        startpt=p["startpt"]
        turn=p["turn"]
        heading=p["heading_init"]+turn*math.pi/2
        angle=p["heading_end"]-p["heading_init"]
        segments=CircleSegmentation(center, radius, startpt, heading, angle, int(abs(self.nb_segments_in_circle*angle/(2*math.pi))))
        self.cl_segments.extend(segments)
        segments=CircleSegmentation(center, radius+turn*self.roadwidth/2, startpt, heading, angle, int(abs(self.nb_segments_in_circle*angle/(2*math.pi))))
        self.lw_segments.extend(segments)
        segments=CircleSegmentation(center, radius-turn*self.roadwidth/2, startpt, heading, angle, int(abs(self.nb_segments_in_circle*angle/(2*math.pi))))
        self.rw_segments.extend(segments)
      else:
        raise Exception("type {} unknown".format(type))   
   
    # close circuit
    spt=self.cl_segments[0].p1
    ept=self.cl_segments[-1].p2
    if (not spt[0] == ept[0]) or (not ept[1] == b[1]):
      print("!!! Circuit start and end are not the same, force closing")
      print("Start point : {:1.1f},{:1.1f}".format(spt[0],spt[1]))
      print("end point   : {:1.1f},{:1.1f}".format(ept[0],ept[1]))
      s=Segment(p1=spt, p2=ept)
      self.cl_segments.append(s)
      spt=self.lw_segments[0].p1
      ept=self.lw_segments[-1].p2
      s=Segment(p1=spt, p2=ept)
      self.cl_segments.append(s)
      spt=self.rw_segments[0].p1
      ept=self.rw_segments[-1].p2
      s=Segment(p1=spt, p2=ept)
      self.cl_segments.append(s)
    
    # start and end lines
    u=Vector.UnitVectorAngle(self.start_heading)
    uorth=Vector.Orthogonal(u)
    start_pt_l=self.start_pt+self.startline_dist*u+self.roadwidth/2*uorth
    start_pt_r=self.start_pt+self.startline_dist*u+-self.roadwidth/2*uorth
    self.start_segment=Segment(p1=start_pt_l, p2=start_pt_r)
    
    end_pt_l=self.end_pt+self.endline_dist*u+self.roadwidth/2*uorth
    end_pt_r=self.end_pt+self.endline_dist*u-self.roadwidth/2*uorth
    self.end_segment=Segment(p1=end_pt_l, p2=end_pt_r)
    
    self.lw_index=1
    for s in self.lw_segments:
      self.lw_index=self.lw_index*s.Side(self.start_pt)
    self.rw_index=1
    for s in self.rw_segments:
      self.rw_index=self.rw_index*s.Side(self.start_pt)

  #########################################################
  def AddTrajectoryPoint(self, p):
    """
    Add a point to a trajectory
    """
    if len(self.trajectory_points)>0:
      s=Segment(p1=self.trajectory_points[-1], p2=p)
      self.trajectory_segments.append(s)
    self.trajectory_points.append(p)
    
  #########################################################
  def IsLineCrossed(self):
    """
    Check if line is crossed by the last trajectory point
    """
    if DoesSegmentsIntersect(self.trajectory_segments[-1], self.end_segment):
      return True
    else:
      return False
  
  #########################################################
  def IsInside(self, p):
    O=np.array([0,0])
    sOP=Segment(p1=O,p2=p)

    i_lw=0
    for s in self.lw_segments:
      if DoesSegmentsIntersect(s, sOP):
        i_lw+=1

    i_rw=0
    for s in self.rw_segments:
      if DoesSegmentsIntersect(s, sOP):
        i_rw+=1
        
    if (i_lw%2)==1 and (i_rw%2)==0:
      return True
    else:
      return False
      
  #########################################################
  def CreateImage(self, red_factor=1):
    self.red_factor=red_factor
    self.pic_width=int((self.width+2*self.margins)/red_factor)
    self.pic_height=int((self.height+2*self.margins)/red_factor)
    self.image=np.zeros((self.pic_width, self.pic_height, 3), dtype=int)
    self.image[:,:]=(128,128,128)
    
  #########################################################
  def DrawPoint(self, p, color):
    rc=skimage.draw.circle(int(p[0]), int(p[1]), int(3*self.red_factor))
    x=np.array((rc[0]+self.margins)/self.red_factor, dtype=int)
    y=np.array(self.pic_height-1-(rc[1]+self.margins)/self.red_factor, dtype=int)
    self.image[x,y]=color

  #########################################################
  def DrawSegment(self, s, color):
    rc=s.Draw()
    x=np.array((rc[0]+self.margins)/self.red_factor, dtype=int)
    y=np.array(self.pic_height-1-(rc[1]+self.margins)/self.red_factor, dtype=int)
    self.image[x,y]=color

  #########################################################
  def DrawImage(self):
    self.image[:,:]=(64,64,64)
    # start and end lines
    self.DrawSegment(self.start_segment, (128,192,128))
    self.DrawSegment(self.end_segment, (192,0,0))
    # centerline
    for s in self.cl_segments:
      self.DrawSegment(s, (128,0,0))
    # left wall
    for s in self.lw_segments:
      self.DrawSegment(s, (128,128,0))
    # right wall
    for s in self.rw_segments:
      self.DrawSegment(s, (128,128,0))
    for s in self.trajectory_segments:
      print("Draw traj segment "+str(s))
      self.DrawSegment(s, (0,0,128))
    for p in self.trajectory_points:
      self.DrawPoint(p, (0,0,128))
    
  #########################################################
  def SaveImage(self, img_fname):
    # save image
    skimage.io.imsave(img_fname, self.image)

  #########################################################
  def ShowImage(self, display):
    surf=pygame.surfarray.make_surface(self.image)
    display.blit(surf, (0,0))
    pygame.display.update()

  #########################################################
  def WallDistance(self, p):
    dmin_lw=math.inf
    smin_lw=None
    for s in self.lw_segments:
      d=s.DistToPoint(p)
      if d<dmin_lw:
        dmin_lw=d
        smin_lw=s
    dmin_rw=math.inf
    smin_rw=None
    for s in self.rw_segments:
      d=s.DistToPoint(p)
      if d<dmin_rw:
        dmin_rw=d
        smin_rw=s
    return [dmin_lw, smin_lw, dmin_rw, smin_rw]
    
###########################################################
###########################################################
class Model():
  # throttle parameters
  aero_coeff=0.6 
  motor_pwr=100 # motor power in W
  brake_eff=100 # brake efficiency in W
  acc_coeff=1 # ecceleration coefficient, include model wheight
  time_step=1 # time quantization
  # steering parameters
  steer_max=math.pi/3 # 60 degrees
  thexpo_coeff=1
  stexpo_coeff=1
  
  #########################################################
  def __init__(self, circuit, pt, heading):
    self.circuit=circuit
    self.cur_pt=pt
    self.cur_heading=heading
    self.cur_speed=0
    self.cur_pwr=0
    #print("Model heading={}".format(self.cur_heading))
    
  #########################################################
  def Update(self, thottle, steering):
    [pt, heading, speed]=self.Transition(throttle, steering)
    self.cur_speed=speed
    self.cur_pt=pt
    self.cur_heading=heading

  #########################################################
  def Transition(self, thottle, steering):
    # model wheight not used
    # aerodynamic not used
    accell=self.acc_coeff*(throttle-0.5)*self.motor_pwr
    speed=self.cur_speed+accell*self.time_step
    # Heading
    heading=self.cur_heading+(2*(steering-0.5))*self.steer_max
    # Position
    pt=self.cur_pt+np.array([math.cos(heading), math.sin(heading)])*(speed*self.time_step)
    
    #print("UpdateTest : pt={},{}".format(pt[0], pt[1]))
    #print("  throttle={}".format(throttle))
    #print("  steering={}".format(steering))
    #print("  speed={}".format(speed))
    #print("  heading={}".format(np.rad2deg(heading)))
    return [pt, heading, speed]
    

###########################################################
###########################################################
class DashBoard():
  def __init__(self, pos, width, height, max_steering):
    self.posx=pos[0]
    self.posy=pos[1]
    self.width=width
    self.height=height
    self.st_rect_width=width*0.8
    self.max_steering=max_steering
    
    self.throttle=0.5
    self.steering=0.5
    self.warning=0
    
    self.rect=pygame.Rect(self.posx, 0, width, height)
    self.st_rect=pygame.Rect(self.posx+(width-self.st_rect_width)/2, (width-self.st_rect_width)/2, self.st_rect_width, self.st_rect_width)
    self.st_ctr=np.array([self.posx+self.width/2, self.width/2], dtype=int)
    self.st_left=self.st_ctr+self.st_rect_width/2*np.array([-math.sin(max_steering), -math.cos(max_steering)])
    self.st_right=self.st_ctr+self.st_rect_width/2*np.array([math.sin(max_steering), -math.cos(max_steering)])

    self.w_rect=pygame.Rect(self.posx+(width-self.st_rect_width)/2, width/2+100, self.st_rect_width, 20)

  #########################################################
  def Update(self, throttle, steering, warning):
    self.throttle=throttle
    self.steering=steering
    if warning > 1:
      self.warning=1
    else:
      self.warning=warning

  #########################################################
  def Show(self, display):
    pygame.draw.rect(display, (0,0,0), self.rect)
  
    # steering+throttle
    pygame.draw.arc(display, (255,255,255), self.st_rect, math.pi/2-self.max_steering, math.pi/2+self.max_steering)
    pygame.draw.line(display, (255,255,255), self.st_ctr, self.st_left)
    pygame.draw.line(display, (255,255,255), self.st_ctr, self.st_right)
    pygame.draw.circle(display, (255,255,255), self.st_ctr+np.array([-self.throttle*self.width*0.4*math.sin(self.max_steering*(self.steering-0.5)*2), -self.throttle*self.width*0.4*math.cos(self.max_steering*(self.steering-0.5)*2)], dtype=int), 2)

    # warning
    pygame.draw.rect(display, (255,255,255), self.w_rect, 1)
    self.w_rect_i=pygame.Rect(self.posx+(self.width*0.2)/2+1, self.width/2+100+1, 1+(self.st_rect_width-3)*self.warning, 18)
    pygame.draw.rect(display, (int(255*(self.warning)),int(255*(1-self.warning)),0), self.w_rect_i)
    
    pygame.display.update()
    
if __name__ == '__main__':
  s=Segment(np.array([0,0], dtype=int), length=10, heading=0)

  throttle=0.5
  throttle_step=0.05
  steering=0.5
  steering_step=0.05

  pygame.init()
  circdesc_fname="CircuitTest.txt"
  circdesc_fname="TRR_description.txt"
  # load circuit description
  circuit=Circuit()
  circuit.LoadDescription(circdesc_fname)
  model=Model(circuit, circuit.start_pt, circuit.start_heading)
  circuit.GenerateSegments()
  circuit.CreateImage(red_factor=3)
  display=pygame.display.set_mode((circuit.image.shape[0]+400, circuit.image.shape[1]))

  dashboard=DashBoard(circuit.image.shape, 400, circuit.image.shape[1], model.steer_max)
  circuit.AddTrajectoryPoint(circuit.start_pt)
  circuit.DrawImage()
  pt=circuit.start_pt
  heading=circuit.start_heading
  
  circuit.DrawImage()
  circuit.DrawPoint(pt, (128,255,128))
  circuit.ShowImage(display)
  dashboard.Show(display)

  Cont = True
  while Cont:
    for event in pygame.event.get():
      if event.type == pygame.QUIT:
        Cont = False
      elif event.type == pygame.KEYDOWN :
        print("###############")
        if event.key == pygame.K_UP:
          throttle=UpdateCmd(throttle, throttle_step)
          print("Throttle={:1.1f}".format(throttle))
        elif event.key == pygame.K_DOWN:
          throttle=UpdateCmd(throttle, -throttle_step)
          print("Throttle={:1.1f}".format(throttle))
        elif event.key == pygame.K_RIGHT:
          steering=UpdateCmd(steering, -steering_step)
        elif event.key == pygame.K_LEFT:
          steering=UpdateCmd(steering, steering_step)
        elif event.key == pygame.K_RETURN:
          model.Update(throttle, steering)
          pt=model.cur_pt
          circuit.AddTrajectoryPoint(model.cur_pt)
          if circuit.IsLineCrossed():
            print("Line Crossed !!!!!")
        elif event.key == pygame.K_KP_ENTER:
            print('ENTER')    
        elif event.key == pygame.K_ESCAPE:
          Cont = False
        [new_pt, new_heading, speed]=model.Transition(throttle, steering)
        s=Segment(p1=pt, p2=new_pt)
        [dl, sl, dr, sr]=circuit.WallDistance(new_pt)
        warning=Expo(1-min(dl, dr)*2/circuit.roadwidth, 2)
        inside=circuit.IsInside(new_pt)
        if not inside:
          warning=1
        circuit.DrawImage()
        circuit.DrawPoint(pt, (128,255,128))
        circuit.DrawPoint(new_pt, (128,255,128))
        circuit.DrawSegment(s, (128,255,128))
        circuit.DrawSegment(sl, (255,0,0))
        circuit.DrawSegment(sr, (255,0,0))

        print("Wall distance = {:.1f},{:.1f} => W={:.2f}, Inside={}".format(dl, dr, warning,inside))

        dashboard.Update(throttle, steering, warning)
        dashboard.Show(display)
        circuit.ShowImage(display)
  
  pygame.quit()