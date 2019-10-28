# setup : particles = [Particle(N_LM) for _ in range(N_PARTICLE)]
# loop :
#  angles in radians
#  x = [x,y,yaw]
#  u = [v,w]
#  z = [2 x n] = [ [d1, d2, ...], [a1, a2, ...], [i1, i2, ...] ] d distance, a angle, i index of LM
#  u = np.array([v, yawrate]).reshape(2, 1)
#  z = from lidar (d,a) and association (..,i)
#  particles = fast_slam2(particles, u, z)
#  xEst = calc_final_state(particles)

import math
import numpy as np

# Parameters
N_PARTICLE = 100  # number of particle
NTH = N_PARTICLE / 1.5  # Number of particle for re-sampling
STATE_SIZE = 3  # State size [x,y,yaw]

LM_SIZE = 2  # LM srate size [x,y]
M_DIST_TH = 2.0  # Threshold of Mahalanobis distance for data association.

# helper
def pi_2_pi(angle):
	return (angle + math.pi) % (2 * math.pi) - math.pi

# Fast SLAM process covariance matrix Q
Q = np.diag([3.0, np.deg2rad(10.0)]) ** 2

# Fast SLAM measurement covariance matrix Q
R = np.diag([1.0, np.deg2rad(20.0)]) ** 2
	
# Particles
class Particle:

	def __init__(self,N_LM):
		self.w = 1.0 / N_PARTICLE
		self.x = 0.0
		self.y = 0.0
		self.yaw = 0.0
		# ???????????????????????????????????????
		self.P = np.eye(3)
		# landmark x-y positions
		self.lm = np.zeros((N_LM, LM_SIZE))
		# landmark position covariance
		self.lmP = np.zeros((N_LM * LM_SIZE, LM_SIZE))

# Center of particules
def calc_final_state(particles):
	xEst = np.zeros((STATE_SIZE, 1))
	particles = normalize_weight(particles)
	for i in range(N_PARTICLE):
		xEst[0, 0] += particles[i].w * particles[i].x
		xEst[1, 0] += particles[i].w * particles[i].y
		xEst[2, 0] += particles[i].w * particles[i].yaw
	xEst[2, 0] = pi_2_pi(xEst[2, 0])
	return xEst

# helper : zero copy weights normalisation
def normalize_weight(particles):
	sum_w = sum([p.w for p in particles])
	try:
		for i in range(N_PARTICLE):
			particles[i].w /= sum_w
	except ZeroDivisionError:
		for i in range(N_PARTICLE):
			particles[i].w = 1.0 / N_PARTICLE
		return particles
	return particles

# fast slam process
def fast_slam2(particles, u, z):
	particles = predict_particles(particles, u)
	particles = update_with_observation(particles, z)
	particles = resampling(particles)
	return particles		

# fast slam sub-process 1
def predict_particles(particles, u):
	for i in range(N_PARTICLE):
		px = np.zeros((STATE_SIZE, 1))
		px[0, 0] = particles[i].x
		px[1, 0] = particles[i].y
		px[2, 0] = particles[i].yaw
		#### 
		#ud = u + (np.random.randn(1, 2) @ R ** 0.5).T  # add noise
		#px = motion_model(px, ud)
		### changed by
		px = motion_model(px, u)
		####
		particles[i].x = px[0, 0]
		particles[i].y = px[1, 0]
		particles[i].yaw = px[2, 0]
	return particles

# helper : update pose
def motion_model(x, u):
	F = np.array([[1.0, 0, 0],
				  [0, 1.0, 0],
				  [0, 0, 1.0]])
	B = np.array([[DT * math.cos(x[2, 0]), 0],
				  [DT * math.sin(x[2, 0]), 0],
				  [0.0, DT]])
	x = F @ x + B @ u
	x[2, 0] = pi_2_pi(x[2, 0])
	return x


























# fast slam sub-process 2
def update_with_observation(particles, z):
	for iz in range(len(z[0, :])):
		lmid = int(z[2, iz])
		for ip in range(N_PARTICLE):
			# new landmark
			if abs(particles[ip].lm[lmid, 0]) <= 0.01:
				particles[ip] = add_new_lm(particles[ip], z[:, iz], Q)
			# known landmark
			else:
				w = compute_weight(particles[ip], z[:, iz], Q)
				particles[ip].w *= w
				particles[ip] = update_landmark(particles[ip], z[:, iz], Q)
				particles[ip] = proposal_sampling(particles[ip], z[:, iz], Q)
	return particles


def add_new_lm(particle, z, Q_cov):
	r = z[0]
	b = z[1]
	lm_id = int(z[2])
	s = math.sin(pi_2_pi(particle.yaw + b))
	c = math.cos(pi_2_pi(particle.yaw + b))
	particle.lm[lm_id, 0] = particle.x + r * c
	particle.lm[lm_id, 1] = particle.y + r * s
	# covariance
	Gz = np.array([[c, -r * s],
				   [s, r * c]])
	particle.lmP[2 * lm_id:2 * lm_id + 2] = Gz @ Q_cov @ Gz.T
	return particle


def compute_weight(particle, z, Q_cov):
	lm_id = int(z[2])
	xf = np.array(particle.lm[lm_id, :]).reshape(2, 1)
	Pf = np.array(particle.lmP[2 * lm_id:2 * lm_id + 2])
	zp, Hv, Hf, Sf = compute_jacobians(particle, xf, Pf, Q_cov)

	dz = z[0:2].reshape(2, 1) - zp
	dz[1, 0] = pi_2_pi(dz[1, 0])

	try:
		invS = np.linalg.inv(Sf)
	except np.linalg.linalg.LinAlgError:
		return 1.0

	num = math.exp(-0.5 * dz.T @ invS @ dz)
	den = 2.0 * math.pi * math.sqrt(np.linalg.det(Sf))

	w = num / den

	return w


def compute_jacobians(particle, xf, Pf, Q_cov):
	dx = xf[0, 0] - particle.x
	dy = xf[1, 0] - particle.y
	d2 = dx ** 2 + dy ** 2
	d = math.sqrt(d2)

	zp = np.array(
		[d, pi_2_pi(math.atan2(dy, dx) - particle.yaw)]).reshape(2, 1)

	Hv = np.array([[-dx / d, -dy / d, 0.0],
				   [dy / d2, -dx / d2, -1.0]])

	Hf = np.array([[dx / d, dy / d],
				   [-dy / d2, dx / d2]])

	Sf = Hf @ Pf @ Hf.T + Q_cov

	return zp, Hv, Hf, Sf


def update_landmark(particle, z, Q_cov):
	lm_id = int(z[2])
	xf = np.array(particle.lm[lm_id, :]).reshape(2, 1)
	Pf = np.array(particle.lmP[2 * lm_id:2 * lm_id + 2])

	zp, Hv, Hf, Sf = compute_jacobians(particle, xf, Pf, Q_cov)

	dz = z[0:2].reshape(2, 1) - zp
	dz[1, 0] = pi_2_pi(dz[1, 0])

	xf, Pf = update_kf_with_cholesky(xf, Pf, dz, Q, Hf)

	particle.lm[lm_id, :] = xf.T
	particle.lmP[2 * lm_id:2 * lm_id + 2, :] = Pf

	return particle


def update_kf_with_cholesky(xf, Pf, v, Q_cov, Hf):
	PHt = Pf @ Hf.T
	S = Hf @ PHt + Q_cov

	S = (S + S.T) * 0.5
	SChol = np.linalg.cholesky(S).T
	SCholInv = np.linalg.inv(SChol)
	W1 = PHt @ SCholInv
	W = W1 @ SCholInv.T

	x = xf + W @ v
	P = Pf - W1 @ W1.T

	return x, P

def proposal_sampling(particle, z, Q_cov):
	lm_id = int(z[2])
	xf = particle.lm[lm_id, :].reshape(2, 1)
	Pf = particle.lmP[2 * lm_id:2 * lm_id + 2]
	# State
	x = np.array([particle.x, particle.y, particle.yaw]).reshape(3, 1)
	P = particle.P
	zp, Hv, Hf, Sf = compute_jacobians(particle, xf, Pf, Q_cov)

	Sfi = np.linalg.inv(Sf)
	dz = z[0:2].reshape(2, 1) - zp
	dz[1] = pi_2_pi(dz[1])

	Pi = np.linalg.inv(P)

	particle.P = np.linalg.inv(Hv.T @ Sfi @ Hv + Pi)  # proposal covariance
	x += particle.P @ Hv.T @ Sfi @ dz  # proposal mean

	particle.x = x[0, 0]
	particle.y = x[1, 0]
	particle.yaw = x[2, 0]

	return particle



# #  Simulation parameter
# Q_sim = np.diag([0.3, np.deg2rad(2.0)]) ** 2
# R_sim = np.diag([0.5, np.deg2rad(10.0)]) ** 2
# OFFSET_YAW_RATE_NOISE = 0.01

# def observation(xTrue, xd, u, RFID):
#     # calc true state
#     xTrue = motion_model(xTrue, u)

#     # add noise to range observation
#     z = np.zeros((3, 0))

#     for i in range(len(RFID[:, 0])):

#         dx = RFID[i, 0] - xTrue[0, 0]
#         dy = RFID[i, 1] - xTrue[1, 0]
#         d = math.sqrt(dx ** 2 + dy ** 2)
#         angle = pi_2_pi(math.atan2(dy, dx) - xTrue[2, 0])
#         if d <= MAX_RANGE:
#             dn = d + np.random.randn() * Q_sim[0, 0] ** 0.5  # add noise
#             anglen = angle + np.random.randn() * Q_sim[1, 1] ** 0.5  # add noise
#             zi = np.array([dn, pi_2_pi(anglen), i]).reshape(3, 1)
#             z = np.hstack((z, zi))

#     # add noise to input
#     ud1 = u[0, 0] + np.random.randn() * R_sim[0, 0] ** 0.5
#     ud2 = u[1, 0] + np.random.randn() * R_sim[1, 1] ** 0.5 + OFFSET_YAW_RATE_NOISE
#     ud = np.array([ud1, ud2]).reshape(2, 1)

#     xd = motion_model(xd, ud)

#     return xTrue, z, xd, ud










