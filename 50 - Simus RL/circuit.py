###########################################################
#
# Circuit module
# v0.06
#
###########################################################

import os
import re
import math
import numpy as np
import skimage.draw
import random
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
    res=False
  elif alpha>=0 and alpha<=1 and beta>=0 and beta<=1:
    res=True
  else:
    res=False
  return res
  
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
    self.odo=0
    if p2 is not None:
      # segment definition with p1 and p2
      self.p1=p1
      self.p2=p2
      u=p2-p1
      nu=Vector.Norm(u)
      if nu == 0:
        #raise Exception("Definition of null segment")
        self.u=np.array([1,0], dtype=float)
      else:
        self.u=u/nu
      self.length=nu
      self.heading=Vector.Heading(u)
      self.normal=Vector.Orthogonal(self.u)
    else:
      # segment definition with p1, length and heading
      self.heading=heading
      self.length=length
      if length == 0:
        raise Exception("Definition of null segment")
      self.p1=p1
      self.p2=p1+np.array([length*math.cos(heading), length*math.sin(heading)], dtype=float)
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
      res=Segment(p1=self.p1-other, p2=self.p2-other)
      return res
    else:
      raise Exception("Cannot substract segment with "+type(other))

  #########################################################
  def __str__(self):
    return "({:.2f},{:.2f})->({:.2f},{:.2f})".format(self.p1[0], self.p1[1], self.p2[0], self.p2[1])
    
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
    dPH=Vector.Norm(PH)-1e-10
    dPA=Vector.Norm(PA)+1e-10
    dPB=Vector.Norm(PB)+1e-10
    if dPA*dPA<dPH*dPH:
      print("!!!! Erreur DistToPoint s={}, e={}".format(self, dPA*dPA-dPH*dPH))
      print("       dPA<dPH          p={:.2f},{:.2f}".format(p[0],p[1]))
    if dPB*dPB<dPH*dPH:
      print("!!!! Erreur DistToPoint s={}, e={}".format(self, dPB*dPB-dPH*dPH))
      print("       dPB<dPH          p={:.2f},{:.2f}".format(p[0],p[1]))
    dHA=math.sqrt(abs(dPA*dPA-dPH*dPH))
    dHB=math.sqrt(abs(dPB*dPB-dPH*dPH))
    dAB=Vector.Norm(self.p2-self.p1)
    if dHA < dAB and dHB < dAB:
      # H is on the segment
      res = dPH
    else :
      res = min(dPA, dPB)
    return res

  #########################################################
  def SetOdo(self, odo):
    self.odo=odo
    
###########################################################
###########################################################
class Circuit():
  colorDict={"red":(255,0,0),"green":(0,255,0),"blue":(0,0,255),"white":(255,255,255),"black":(0,0,0),"yellow":(128,128,0)}
  
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
        cl_seg=Segment(p1=p["startpt"], p2=p["endpt"])
        self.cl_segments.append(cl_seg)
        lw_seg=cl_seg+(self.roadwidth/2)*cl_seg.normal
        self.lw_segments.append(lw_seg)
        rw_seg=cl_seg-(self.roadwidth/2)*cl_seg.normal
        self.rw_segments.append(rw_seg)
      elif type=="curve":
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
      print("Delta={},{}".format(ept[0]-spt[0], ept[1]-spt[1]))
      s=Segment(p1=ept, p2=spt)
      self.cl_segments.append(s)
      spt=self.lw_segments[0].p1
      ept=self.lw_segments[-1].p2
      s=Segment(p1=ept, p2=spt)
      self.lw_segments.append(s)
      spt=self.rw_segments[0].p1
      ept=self.rw_segments[-1].p2
      s=Segment(p1=ept, p2=spt)
      self.rw_segments.append(s)
    
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
      
    # set odo value for segments
    odo=0
    for i in range(len(self.cl_segments)):
      self.cl_segments[i].SetOdo(odo)
      odo+=self.cl_segments[i].length
    self.length=odo

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
    O=np.array([0,0], dtype=float)
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
  def DrawPoint(self, p, color, size=3):
    rc=skimage.draw.circle(int((p[0]+self.margins)/self.red_factor), int(self.pic_height-1-(p[1]+self.margins)/self.red_factor), size)
    self.image[rc[0],rc[1]]=self.colorDict[color]

  #########################################################
  def DrawSegment(self, s, color):
    ax=int((s.p1[0]+self.margins)/self.red_factor)
    ay=int(self.pic_height-1-(s.p1[1]+self.margins)/self.red_factor)
    bx=int((s.p2[0]+self.margins)/self.red_factor)
    by=int(self.pic_height-1-(s.p2[1]+self.margins)/self.red_factor)
    rc=skimage.draw.line(ax, ay, bx, by)
    self.image[rc[0],rc[1]]=self.colorDict[color]

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
    self.image[x,y]=self.colorDict[color]
    # polygon edge
    rc=skimage.draw.polygon_perimeter(r,c)
    x=rc[0]
    y=np.array(self.pic_height-1-rc[1], dtype=int)
    self.image[x,y]=self.colorDict[color]
    
  #########################################################
  def DrawTrajectory(self, trajectory, color, ptsize=1):
    if trajectory.segments:
      self.DrawPoint(trajectory.segments[0].p1, color)
      for s in trajectory.segments:
        self.DrawSegment(s, color)
        self.DrawPoint(s.p2, color, size=ptsize)

  #########################################################
  def DrawImage(self):
    """
    Draw the circuit on the image
    """
    self.image[:,:]=self.colorDict["black"]
    # road, walls and center line
    for cl, lw, rw in zip(self.cl_segments, self.lw_segments, self.rw_segments):
      self.DrawPolygon([lw.p1, lw.p2, rw.p2, rw.p1], "black")
      self.DrawSegment(rw, "yellow")
      self.DrawSegment(lw, "yellow")
      self.DrawSegment(cl, "white")
    # start and end lines
    self.DrawSegment(self.start_segment, "white")
    self.DrawSegment(self.end_segment, "red")
   
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
    
  #########################################################
  def TraveledDistance(self, s):
    """
    Compute the traveled distance of a segment on the center line
    """
    # find nearest cl segment for segment extremity points
    p1_dist=math.inf
    p1_i=None
    p2_dist=math.inf
    p2_i=None
    for i, scl in enumerate(self.cl_segments):
      d=scl.DistToPoint(s.p1)
      if d<p1_dist:
        p1_dist=d
        p1_i=i
        s1=scl
      d=scl.DistToPoint(s.p2)
      if d<p2_dist:
        p2_dist=d
        p2_i=i
        s2=scl
    # find distance of point on the segments
    u=s1.u
    v=s.p1-s1.p1
    d1=u[0]*v[0]+u[1]*v[1]
    u=s2.u
    v=s.p2-s2.p1
    d2=u[0]*v[0]+u[1]*v[1]
    if p1_i == p2_i:
      # same cl segment
      d=d2-d1
    elif p1_i == p2_i+1:
      # next segment
      d=s1.length-d1+d2
    elif p1_i < p2_i+1:
      # at least 1 segment between
      x=0
      for i in range(p1_i+1, p2_i):
        x+=self.cl_segments[i].length
      d=s1.length-d1+d2+x
    elif p1_i==len(self.cl_segments)-1 and  p2_i==0:
      # last and 1st segment
      d=s1.length-d1+d2
    else :
      d=0
      raise Exception("Unimplemented case")
    return d

  #########################################################
  def IsEndLineCrossed(self, s):
    res=DoesSegmentsIntersect(s, self.end_segment)
    return res
    
  #########################################################
  def PtXY2LineCoord(self, p):
    """
    Convert (x,y) point coordinate into odo+center line distance coordinates
    """

    # search nearest centerline segment index
    p_dist=math.inf
    s_index=None
    for i, scl in enumerate(self.cl_segments):
      d=scl.DistToPoint(p)
      if d<p_dist:
        p_dist=d
        s_index=i
        s=scl

    # compute odo segments before
    odo=self.cl_segments[s_index].odo
    # compute distances on and to segment
    u=s.u
    v=p-s.p1
    dist_p_on_segment=u[0]*v[0]+u[1]*v[1]
    u=s.normal
    dist_p_to_segment=u[0]*v[0]+u[1]*v[1]
    odo+=dist_p_on_segment

    return [odo, dist_p_to_segment, s_index]
    
  #########################################################
  def PtLineCoord2XY(self, odo, width, s_index=None):
    """
    Convert odo+center line distance into (x,y) point coordinate 
    """
    if s_index is not None:
      s=self.cl_segments[s_index]
    else:
      s=self.cl_segments[0]
      # search segment for odometrie=odo
      for i, seg in enumerate(self.cl_segments):
        if odo>=seg.odo and odo < seg.odo+seg.length:
          s=seg
          s_idx=i
          break
          
    p=s.p1+(odo-s.odo)*s.u+width*s.normal

    return [p,s_idx]
  
###########################################################
###########################################################
class Trajectory():
  #########################################################
  def __init__(self, start_pt):
    self.endpoint=start_pt
    self.segments=[]
    self.steering=[]
    self.heading=[]
    self.traveled_distance=[]
    self.endline_cross=0

  #########################################################
  def AddPoint(self, pt, steering=0, heading=0, traveled_distance=0):
    s=Segment(self.endpoint, pt)
    self.segments.append(s)
    self.endpoint=pt
    self.steering.append(steering)
    self.heading.append(heading)
    self.traveled_distance.append(traveled_distance)

  #########################################################
  def LastSegment(self):
    if self.segments :
      return self.segments[-1]
    else:
      return None

  #########################################################
  def Cut(self, len):
    self.segments=self.segments[0:len]
    self.steering=self.steering[0:len]
    self.heading=self.heading[0:len]
    self.traveled_distance=self.traveled_distance[0:len]
    self.endpoint=self.segments[-1].p2
    
  #########################################################
  def NbOfSteps(self):
    return len(self.steering)
    
  #########################################################
  def TotalTraveledDistance(self):
    total=0
    for t in self.traveled_distance:
      total+=t
    return total
    
  #########################################################
  def Save(self, fname):
    with open(fname, "wb") as output:
      picke.dump(self, output, pickle.HIGHEST_PROTOCOL)

  #########################################################
  def Load(fname):
    with open(fname, "rb") as input:
      res=picke.load(input)
    return res
