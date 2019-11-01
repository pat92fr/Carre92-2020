
import os
import re
import math
import numpy as np
import skimage.draw
import random
import pygame
import time

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
  """
  Find intersection of lines containing AB and CD
  Result is x and y such that the intersection point is
  P=x.AB=y.CD
  Result is [None,None] if line are parallel or AB or CD = 0.
  """
  AB=sAB.p2-sAB.p1
  CD=sCD.p2-sCD.p1
  
  mat=np.array([[AB[0], -CD[0]],[AB[1],-CD[1]]])
  if np.linalg.det(mat) == 0:
    return [None, None]
  else:
    X=sCD.p1-sAB.p1
    sol=np.linalg.solve(mat, X)
    return sol
  
#########################################################
def DoesSegmentsIntersect(sAB, sCD):
  [alpha, beta]=LineIntersection(sAB, sCD)
  if alpha is None:
    return False
  if alpha>=0 and alpha<=1 and beta>=0 and beta<=1:
    return True
  else:
    return False

#########################################################
def Params2Float(text):
  """
  Return a list of int from a string "x1,...,xn)"
  """
  str=text.replace("(","").replace(")","")
  l=str.split(",")
  res=[]
  for s in l:
    res.append(float(s))
  return res

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
      self.p2=p1+np.array([int(length*math.cos(heading)), int(length*math.sin(heading))], dtype=float)
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
    self.unit=1
      
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
    
    values_set=False
    
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
        values_set=True
        self.width = self.unit*float(params)
        
      elif cmd == "height":
        values_set=True
        self.height = self.unit*float(params)
        
      elif cmd == "margins":
        values_set=True
        self.margins = self.unit*float(params)
        
      elif cmd == "roadwidth":
        values_set=True
        self.roadwidth = self.unit*float(params)
        
      elif cmd == "unitlength":
        if values_set:
          raise Exception("Error : unit value shall be set before any other values")
        self.unit = self.unit*float(params)
        
      elif cmd == "changeheading":
        values_set=True
        changeheading = np.deg2rad(float(params))
        cur_start_heading+=changeheading
        cur_end_heading+=changeheading

      elif cmd == "startpoint":
        values_set=True
        [x,y]=Params2Float(params)
        x=self.unit*x
        y=self.unit*y
        cur_init_pt=np.array([x,y], dtype=float)
        cur_end_pt=cur_init_pt
        self.init_pt=cur_init_pt
        self.end_pt=cur_end_pt
        
      elif cmd == "startline":
        values_set=True
        self.startline_dist=self.unit*float(params)
        
      elif cmd == "endline":
        values_set=True
        self.endline_dist=self.unit*float(params)

      elif cmd == "initialheading":
        values_set=True
        cur_start_heading=np.deg2rad(float(params))
        cur_end_heading=cur_start_heading
        self.start_heading=cur_start_heading
        
      elif cmd == "line":
        values_set=True
        # line parameters
        length=self.unit*float(params)
        vector=length*Vector.UnitVectorAngle(cur_end_heading)
        init_pt=cur_end_pt
        end_pt=init_pt+vector
        # update current data
        cur_init_pt=init_pt
        cur_end_pt=end_pt
        cumulated_length+=length
        partdesc={'type':"line", 'startpt':init_pt, 'endpt':end_pt, 'length':length, 'cumulated_length':cumulated_length, 'heading_init':cur_start_heading}
        self.parts_description.append(partdesc)
        
      elif cmd == "curve":
        values_set=True
        # curve parameters
        [radius, angle]=Params2Float(params)
        radius=self.unit*radius
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
        init_pt=cur_end_pt
        center=init_pt+P1C
        end_pt=center-P2C
        cur_init_pt=init_pt
        cur_end_pt=end_pt
        length=radius*abs(angle)
        cumulated_length+=length
        partdesc={'type':"curve", 'startpt':cur_init_pt, 'endpt':cur_end_pt, 'length':length, 'cumulated_length':cumulated_length, \
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
    if (not spt[0] == ept[0]) or (not spt[1] == ept[1]):
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

    u=Vector.UnitVectorAngle(self.start_heading)
    uorth=Vector.Orthogonal(u)

    self.start_pt=self.startline_dist*u+self.init_pt
    start_pt_l=self.start_pt+self.roadwidth/2*uorth
    start_pt_r=self.start_pt-self.roadwidth/2*uorth
    self.start_segment=Segment(p1=start_pt_l, p2=start_pt_r)
    
    self.end_pt=self.endline_dist*u+self.init_pt
    end_pt_l=self.end_pt+self.roadwidth/2*uorth
    end_pt_r=self.end_pt-self.roadwidth/2*uorth
    self.end_segment=Segment(p1=end_pt_l, p2=end_pt_r)
    
    self.lw_index=1
    for s in self.lw_segments:
      self.lw_index=self.lw_index*s.Side(self.start_pt)
    self.rw_index=1
    for s in self.rw_segments:
      self.rw_index=self.rw_index*s.Side(self.start_pt)

  #########################################################
  def AreWallCrossed(self, segment):
    """
    Check if a wall is crossed by the segment
    """
    res=False
    for lws in self.lw_segments:
      if DoesSegmentsIntersect(lws, segment):
        res=True
    for rws in self.rw_segments:
      if DoesSegmentsIntersect(rws, segment):
        res=True
    return res
    
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
  def CreateImage(self, zoom_factor=1):
    self.red_factor=0.01*self.unit/zoom_factor
    self.pic_width=int((self.width+2*self.margins)/self.red_factor)
    self.pic_height=int((self.height+2*self.margins)/self.red_factor)
    self.image=np.zeros((self.pic_width, self.pic_height, 3), dtype=int)
    self.image[:,:]=(128,128,128)
    
  #########################################################
  def DrawPoint(self, p, color):
    rc=skimage.draw.circle(int((p[0]+self.margins)/self.red_factor), int(self.pic_height-1-(p[1]+self.margins)/self.red_factor), 3)
    self.image[rc[0],rc[1]]=color

  #########################################################
  def DrawSegment(self, s, color):
    ax=int((s.p1[0]+self.margins)/self.red_factor)
    ay=int(self.pic_height-1-(s.p1[1]+self.margins)/self.red_factor)
    bx=int((s.p2[0]+self.margins)/self.red_factor)
    by=int(self.pic_height-1-(s.p2[1]+self.margins)/self.red_factor)
    rc=skimage.draw.line(ax, ay, bx, by)
    self.image[rc[0],rc[1]]=color

  #########################################################
  def DrawPolygon(self, polygon, color):
    r=[]
    c=[]
    for p in polygon:
      r.append(int((p[0]+self.margins)/self.red_factor))
      c.append(int((p[1]+self.margins)/self.red_factor))
    rc=skimage.draw.polygon(r,c)
    x=rc[0]
    y=np.array(self.pic_height-1-rc[1], dtype=int)
    self.image[x,y]=color
    # polygon edge
    rc=skimage.draw.polygon_perimeter(r,c)
    x=rc[0]
    y=np.array(self.pic_height-1-rc[1], dtype=int)
    self.image[x,y]=color
    
  #########################################################
  def DrawTrajectory(self, trajectory, color):
    if trajectory.segments:
      self.DrawPoint(trajectory.segments[0].p1, color_trajectory)
      for s in trajectory.segments:
        self.DrawSegment(s, color_trajectory)
        self.DrawPoint(s.p2, color_trajectory)

  #########################################################
  def DrawImage(self):
    """
    Draw the circuit on the image
    """
    self.image[:,:]=color_back
    # road, walls and center line
    for cl, lw, rw in zip(self.cl_segments, self.lw_segments, self.rw_segments):
      self.DrawPolygon([lw.p1, lw.p2, rw.p2, rw.p1], color_road)
      self.DrawSegment(lw, color_wall)
      self.DrawSegment(rw, color_wall)
      self.DrawSegment(cl, color_centerline)
    # start and end lines
    self.DrawSegment(self.start_segment, color_startline)
    self.DrawSegment(self.end_segment, color_endline)
   
  #########################################################
  def SaveImage(self, img_fname):
    # save image
    image=self.image
    image=np.flipud(image)
    image=np.rot90(image, k=3)
    skimage.io.imsave(img_fname, image)

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
    
  #########################################################
  def IsEndLineCrossed(self, s):
    return DoesSegmentsIntersect(s, self.end_segment)
    
###########################################################
###########################################################
class Trajectory():
  def __init__(self, start_pt):
    self.endpoint=start_pt
    self.segments=[]
  def AddPoint(self, pt):
    s=Segment(self.endpoint, pt)
    self.segments.append(s)
    self.endpoint=pt
  def LastSegment(self):
    if self.segments :
      return self.segments[-1]
    else:
      return None

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
  def Transition_old(self, thottle, steering):
    """
    Deprecated model transition
    Use throttle to update speed
    """
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
    
  #########################################################
  def Transition(self, speed, steering):
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
  s=Segment(np.array([0,0], dtype=float), length=10, heading=0)

  throttle=0.2
  throttle_step=0.05
  steering=0.5
  steering_step=0.05

  pygame.init()
  #circdesc_fname="CircuitTest.txt"
  circdesc_fname="PR_description.txt"
  # load circuit description
  circuit=Circuit()
  circuit.LoadDescription(circdesc_fname)
  circuit.GenerateSegments()
  model=Model(circuit, circuit.start_pt, circuit.start_heading)
  circuit.CreateImage(zoom_factor=0.1)
  display=pygame.display.set_mode((circuit.image.shape[0]+400, circuit.image.shape[1]))

  dashboard=DashBoard(circuit.image.shape, 400, circuit.image.shape[1], model.steer_max)
  trajectory=Trajectory(circuit.start_pt)
  circuit.DrawImage()
  pt=circuit.start_pt
  heading=circuit.start_heading
  
  circuit.DrawImage()
  circuit.DrawPoint(circuit.start_pt, (128,255,128))
  circuit.ShowImage(display)
  circuit.SaveImage("Image.png")
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
          trajectory.AddPoint(pt)
          if circuit.AreWallCrossed(trajectory.LastSegment()):
            print("Wall Crossed !!!!!")
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
        circuit.DrawTrajectory(trajectory, color_trajectory)
        circuit.DrawPoint(pt, color_trajectory)
        circuit.DrawPoint(new_pt, color_segment)
        circuit.DrawSegment(s, color_segment)
        circuit.DrawSegment(sl, (255,0,0))
        circuit.DrawSegment(sr, (255,0,0))

        print("Wall distance = {:.1f},{:.1f} => W={:.2f}, Inside={}".format(dl, dr, warning,inside))
        if circuit.IsEndLineCrossed(s):
          print("End line crossed")
        dashboard.Update(throttle, steering, warning)
        dashboard.Show(display)
        circuit.ShowImage(display)
  
  pygame.quit()
