# -*- coding: utf-8 -*-

import roslib; roslib.load_manifest('iv_plan')

from numpy import *
from hironx_if import *
from openravepy import *

import cubeassembly

from init import *

def recognize(camera='lhand'):
    #マーカとブロックの座標の対応配列 [ブロック番号,[原点のずれ],[回転のずれ 軸,角度]]
    Tp_m_list=[
        [0,[-0.015,-0.015,0.06],[ [0,0],[0,0],[0,0] ],26],
        [0,[0.03,-0.015,0.015],[ [0,pi/2],[2,pi/2],[0,0] ],28],
        [1,[0,0.015,0.045],[ [1,-pi/2],[2,pi],[0,0] ],2],
        [1,[-0.015,0.015,0.06],[ [2,pi],[0,0],[0,0] ],3],
        [2,[0.015,0.015,0.06],[ [0,0],[0,0],[0,0] ],34],
        [2,[0.03,0.015,0.045],[ [1,-pi/2],[2,pi],[0,0] ],32],
        [4,[0.045,-0.015,0.09],[ [2,pi],[0,0],[0,0] ],8],
        [5,[0.015,0.015,0.09],[ [2,pi/2],[0,0],[0,0] ],24], 
        [6,[-0.015,-0.015,0.09],[ [0,0],[0,0],[0,0] ],17], 
        [6,[-0.015,-0.03,0.075],[ [2,pi/2],[0,pi/2],[0,0] ],19] 
        ]

    #Tp_m_listの各行のマーカのidの配列
    id_index=[]
    for i in range(0,len(Tp_m_list)):
        id_index.append(Tp_m_list[i][3]-1)

    xyz=[[1,0,0],[0,1,0],[0,0,1]]#回転軸ベクトル

    Tw_p_list=[]#Tw_pのリスト
    block_index_list=[]#Tw_p_listの各行のブロック番号の配列

    poselist=rr.recognize(camera)

    for i in range(0,len(poselist)):#i:何番目に認識したブロックか
        f = ctsvc.ref.Query(camera+'cam', pose2mat(poselist[i][1]), robotframe, rr.get_joint_angles())
        f = reshape(f, (4,4))

        #注目マーカーのあるブロックの番号
        blocknum=Tp_m_list[ id_index.index(poselist[i][0]) ][0]
        #注目マーカーの回転軸リスト
        axislist=Tp_m_list[ id_index.index(poselist[i][0]) ][2]

        # vpython env => openrave env
        relvec = array([150,0,0]) + array([-450,0,-710])
        f[0:3,3] += relvec
        f[0:3,3] /= 1000
                
        # marker => piece
        Tp_m = eye(4)

        for j in range(0,3):
            Tp_m[0:3,0:3] *= rotationMatrixFromAxisAngle( xyz[ axislist[j][0] ], axislist[j][1]  )
        Tp_m[0:3,3] = Tp_m_list[ id_index.index(poselist[i][0]) ][1]

        # f * Tp_m^-1
        Tw_p = dot(f,inverse_matrix(Tp_m))

        Tw_p_list.append([blocknum,Tw_p])

        
    # aqua piece
        # self.gmodels[Tw_p_list[i][0]].target.SetTransform(Tw_p_list[i][1])

    # for i in range(0,len(poselist)):
        # z=f*[0,0,1]
        # if block_index_list.index(blocknum) :
        #     block_index_list.append(blocknum)
        # Tw_p_list.insert( [blocknum,Tw_p] ,blocknum)         


    return Tw_p_list

self = cubeassembly.CubeAssembly(orrobot)
