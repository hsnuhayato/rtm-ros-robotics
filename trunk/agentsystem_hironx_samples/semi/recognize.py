def recognize(camera='lhand'):
#マーカとブロックの座標の対応配列 [ブロック番号,原点のずれ,回転のずれ（オイラー角）]
    Tp_m_list=[ [4,[0.045,-0.015,0.09],[pi,0,0]],
                [1,[-0.015,0.015,0.06],[pi,0,0]],
                [5,[0.015,0.015,0.09],[0,0,0]] ]
    id_index=[7,2,23]#Tp_m_listの各行のマーカのid

    zxz=[[0,0,1],[0,1,0],[0,0,1]]#オイラー角の回転軸ベクトル

    poselist=rr.recognize(camera)

    f=[]
    Tw_p_list=[]#Tw_pのリスト
    for i in range(0,len(poselist)):#i:何番目に認識したブロックか
        f.append( ctsvc.ref.Query(camera+'cam', pose2mat(poselist[i][1]), robotframe, rr.get_joint_angles()))
        f[i] = reshape(f[i], (4,4))

        # vpython env => openrave env
        relvec = array([150,0,0]) + array([-450,0,-710])
        f[i][0:3,3] += relvec
        f[i][0:3,3] /= 1000
                
        # marker => piece
        Tp_m = eye(4)
        for j in range(0,3):
            Tp_m[0:3,0:3] *= rotationMatrixFromAxisAngle(zxz[j], Tp_m_list[ id_index.index(poselist[i][0]) ] [2] [j]  )
        Tp_m[0:3,3] = Tp_m_list[ id_index.index(poselist[i][0]) ][1]

        # f * Tp_m^-1
        Tw_p = dot(f[i],inverse_matrix(Tp_m))
        # Tw_p_list[i][1]=Tw_p

        # Tw_p_list[i][0]=Tp_m_list[ id_index.index(poselist[i][0]) ][0]

        Tw_p_list.append([Tp_m_list[ id_index.index(poselist[i][0]) ][0],Tw_p])
    # aqua piece
        self.gmodels[Tw_p_list[i][0]].target.SetTransform(Tw_p_list[i][1])
    # self.gmodels[4].target.SetTransform(Tw_p)
    return Tw_p_list


