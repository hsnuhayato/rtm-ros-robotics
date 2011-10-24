# -*- coding: utf-8 -*-

from ivutils import *
from numpy import *

poses = {
    'init' : array([0,0,0,
                    -0.010463, 0.0, -1.745329, 0.265277, 0.164054, 0.055845,
                    0.010463, 0.0, -1.745329, -0.265277, 0.164054, -0.055845,
                    0.015708, -0.010472, -0.010472, 0.006981,
                    0.015708, -0.010472, -0.010472, 0.006981]),
    'prepare_right' : array([0.25, -0.4, 1.1,
                             -0.4, -0.3, -2.1, 0.286, 0.486, 0.695,
                             0.010, 0.000, -1.745, -0.265, 0.164, -0.056,
                             0, 0, 0, 0,
                             0, 0, 0, 0]),
    'prepare' : [0, 0, 1.1,
                 -0.4, -0.3, -2.1, 0.286, 0.486, 0.695,
                 0.4, -0.3, -2.1, -0.286, 0.486, -0.695,
                 0.8, -0.1, -0.8, 0.1,
                 0.8, -0.1, -0.8, 0.1]
    }

hand_poses = {
    'open' : array([0.6, -0.1, -0.6, 0.1]),
    'open2' : array([0.8, -0.1, -0.8, 0.1]),
    'close' : array([0.4, -0.4, -0.4, 0.4]),
    'close2' : array([0.3,-0.3, -0.3, 0.3]),
    'push' : array([-pi/6,2*pi/3+0.1,pi/6,-2*pi/3+0.1])
    }

Thd_leye = FRAME(mat=[[0.029886433477364114, 0.1237647407291818, 0.99186142683655232],
                      [-0.99948379941446497, -0.0080013424730153715, 0.03111451794026393],
                      [0.011787103207683197, -0.99227932935785168, 0.12346172170799942]],
                 vec=[31.697510479999998, 69.490635819999994, 90.783463150000003])

# Trh_cam = FRAME(xyzabc=[-30,2,-40,-pi/2,-(pi/2-0.03),0]) # HIRO_110603
# Trh_cam = FRAME(mat=[[0.02954946711261874, -1.4253335856819478e-06, -0.99956331915058227],
#                      [0.99956331349883965, -0.00010630849365328061, 0.029549467097131063],
#                      [-0.00010630418861786432, -0.99999999434823617, -1.7166481752542236e-06]],
#                 vec=[-16.771855992782442, 1.7798966914323426, -46.929327637457341])*FRAME(xyzabc=[0,0,0,deg2rad(3.0),0,0])

Trh_cam = FRAME(mat=[[0.021885085802327658, -0.031848937351081096, -0.99925306514867973], [0.99975797099235009, 0.0029421214171386212, 0.021802370486392553], [0.0022455415123891487, -0.9994883636697196, 0.031905617564034261]],
                vec=[-26.874735596738311, 1.5477540316711211, -45.2067657134371])

Tlh_cam = FRAME(mat=[[0.0021632939588691062, 0.020483004574187356, -0.999787860839919],
                     [-0.99986586801965138, 0.016275656812633266, -0.0018300176574340373],
                     [0.016234719848419218, 0.99965771618035904, 0.020515466178279727]],
                vec=[-32.502063583411996, -2.90324090955921, -45.40873075818665])


# 110922
Thd_kinectrgb =  FRAME(mat=[[-0.020665791755841132, -0.027054668638523939, 0.99942031696176958],
                            [-0.99973997051713659, 0.01019704332427368, -0.020396363838625829],
                            [-0.0096393154061338304, -0.99958194522125221, -0.027258363600893554]],
                       vec=[-39.032368085634957, 1.6231065841821675, 223.66945304907554])

# 110906
# Thd_kinectrgb = FRAME(mat=[[-0.017926910136146185, -0.022532394842858655, 0.99958537257985924],
#                            [-0.9994922056353186, 0.026744462708973238, -0.01732237248920012],
#                            [-0.026343059184855355, -0.99938832537560396, -0.023000398599357947]],
#                       vec=[-35.144783671587241, 1.6231065841821675, 223.10510496573863])

# before 110906
# Thd_kinectrgb = FRAME(mat=[[-1.1117269199954012e-06, -0.0012823607936395445, 0.99999917777444147],
#                            [-0.99999863374434517, 0.0016530300743643547, 1.008056385829424e-06],
#                            [-0.0016530300078927654, -0.9999978115187893, -0.0012823608793248919]],
#                       vec=[-28.74092948502755, 1.6231065841821675, 202.52593003532405])

Thd_kinectdepth = FRAME(xyzabc=[-28.74092948502755, 1.6231065841821675, 202.52593003532405, -pi/2,pi/2,0])*FRAME(xyzabc=[0,0,0,0.05,0,0])

Trwrist_ef = FRAME(xyzabc=[-99,0,-5,0,pi/2,0])
Tlwrist_ef = FRAME(xyzabc=[-99,0,-2,0,pi/2,0])

Tikoffset = FRAME(xyzabc=[-50+59,0,0,0,-pi/2,0])
