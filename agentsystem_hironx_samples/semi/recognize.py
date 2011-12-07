# -*- coding: utf-8 -*-

import roslib; roslib.load_manifest('iv_plan')

from numpy import *
from hironx_if import *
from openravepy import *

import cubeassembly

from init import *

def recognize(camera='lhand'):
    #マーカとブロックの座標の対応配列 [ ブロック番号, [原点のずれ], [回転のずれ(軸番号,角度)] ]
    #回転のずれは３回の回転で表している
    #[どの軸で回転するか,その回転角]
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

    #Tp_m_listの各行のマーカに格納されているidの配列
    id_index=[]
    for i in range(0,len(Tp_m_list)):
        id_index.append(Tp_m_list[i][3]-1)
        Tp_m[0:3,0:3] = dot(Tp_m[0:3],rotationMatrixFromAxisAngle( xyz[ axislist[j][0] ], axislist[j][1]  ))
        Tp_m[0:3,3] = Tp_m_list[ id_index.index(poselist[i][0]) ][1]

        # f * Tp_m^-1
        Tw_p = dot(f,inverse_matrix(Tp_m))

        # Tw_p_list.append([blocknum,Tw_p])

        if blocknum not in block_index_list:# ブロックのリストに入っていなければ追加
            block_index_list.append(blocknum)
            Tw_p_list.append( [blocknum,Tw_p])
            self.gmodels[Tw_p_list[k][0]].target.SetTransform(Tw_p_list[k][1])
            k=k+1
        else:#ブロックのリストに入って座標のz軸とW系のz軸の内積が大きければ、ブロックの座標を入れ替え
            z=f[2][2]#既に登録されているブロックの座標のz軸のz成分
            index=block_index_list.index(blocknum)
            for i in range(0,len(block_index_list)):
                if Tw_p_list[index][1][2][2] > z:
                    Tw_p_list[ index]=[blocknum,Tw_p] 
                    self.gmodels[Tw_p_list[index][0]].target.SetTransform(Tw_p_list[index][1])

    return Tw_p_list

