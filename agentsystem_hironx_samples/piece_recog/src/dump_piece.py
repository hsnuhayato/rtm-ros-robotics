#!/usr/bin/env python
# -*- coding: utf-8 -*-

l = 15.0

piecered = { 'vertices' :
             [
                 [-l,-l,l], # 0
                 [-l,l,l], # 1
                 [l,-l,l], # 2
                 [l,l,l], # 3
                 [3*l,-l,l], # 4
                 [3*l,l,l], # 5
                 [l,3*l,l], # 6
                 [3*l,3*l,l], # 7
                 [5*l,l,l], # 8
                 [5*l,3*l,l] # 9
                 ],
             'edges' :
             [
                 [0,1],
                 [0,2],
                 [1,3],
                 [2,3],
                 [2,4],
                 [4,5],
                 [3,5],
                 [3,6],
                 [6,7],
                 [5,7],
                 [5,8],
                 [7,9],
                 [8,9]
                 ]
             }

pieceaqua = { 'vertices' :
              [
                  [-l,-l,l], # 0
                  [l,-l,l], # 1
                  [-l,l,l], # 2
                  [l,l,l], # 3
                  [-l,3*l,l], # 4
                  [l,3*l,l], # 5
                  [-l,5*l,l], # 6
                  [l,5*l,l], # 7
                  [3*l,l,l], # 8
                  [3*l,3*l,l] # 9
                  ],
             'edges' :
             [
                 [0,1],
                 [0,2],
                 [1,3],
                 [2,3],
                 [2,4],
                 [4,5],
                 [3,5],
                 [4,6],
                 [6,7],
                 [5,7],
                 [3,8],
                 [8,9],
                 [9,5]
                 ]
             }

pieceyellow = { 'vertices' :
                [
                    [-l,l,l], # 0
                    [l,l,l], # 1
                    [3*l,l,l], # 2
                    [-l,-l,l], # 3
                    [l,-l,l], # 4
                    [3*l,-l,l], # 5
                    [-l,-3*l,l], # 6
                    [l,-3*l,l], # 7
                    [-l,-5*l,l], # 8
                    [l,-5*l,l] # 9
                    ],
             'edges' :
             [
                 [0,1],
                 [1,2],
                 [0,3],
                 [1,4],
                 [2,5],
                 [3,4],
                 [4,5],
                 [3,6],
                 [4,7],
                 [6,7],
                 [6,8],
                 [7,9],
                 [8,9]
                 ]
             }

pieceyellowgreen = { 'vertices' :
                     [
                         [-l,-l,l], # 0
                         [l,-l,l], # 1
                         [3*l,-l,l], # 2
                         [-l,l,l], # 3
                         [l,l,l], # 4
                         [3*l,l,l], # 5
                         [-l,3*l,l], # 6
                         [l,3*l,l] # 7
                         ],
                     'edges' :
                     [
                         [0,1],
                         [1,2],
                         [3,4],
                         [4,5],
                         [6,7],
                         [0,3],
                         [1,4],
                         [2,5],
                         [3,6],
                         [4,7],
                         ]
                     }

piecebrown = { 'vertices' :
               [
                   [-l,-l,l], # 0
                   [l,-l,l], # 1
                   [3*l,-l,l], # 2
                   [-l,l,l], # 3
                   [l,l,l], # 4
                   [3*l,l,l], # 5
                   [-l,3*l,l], # 6
                   [l,3*l,l], # 7
                   [3*l,3*l,l] # 8
                   ],
             'edges' :
             [
                   [0,1],
                   [1,2],
                   [3,4],
                   [4,5],
                   [6,7],
                   [7,8],
                   [0,3],
                   [3,6],
                   [1,4],
                   [4,7],
                   [2,5],
                   [5,8]
                   ]
               }


import operator

def spaces(n):
    return reduce(operator.__add__, map(lambda x: ' ', range(n)))

def format_vertice(v, ident_level=1):
    return spaces(ident_level*2)+'{%f, %f, %f}'%(v[0],v[1],v[2])

def format_vertices(vs, name='vertices'):
    code = reduce(lambda x,y: x+',\n'+y, map(format_vertice, vs))
    return 'double ' + name + '[][3] = {\n' + code + '\n};'

def format_edge(e, ident_level=1):
    return spaces(ident_level*2)+'{%d, %d}'%(e[0],e[1])

def format_edges(es, name='edges'):
    code = reduce(lambda x,y: x+',\n'+y, map(format_edge, es))
    return 'int ' + name + '[][2] = {\n' + code + '\n};'

def generate_piece_defs():
    print format_vertices(piecered['vertices'], 'redvertices')
    print format_edges(piecered['edges'], 'rededges')
    print format_vertices(pieceaqua['vertices'], 'aquavertices')
    print format_edges(pieceaqua['edges'], 'aquaedges')
    print format_vertices(pieceyellow['vertices'], 'yellowvertices')
    print format_edges(pieceyellow['edges'], 'yellowedges')
    print format_vertices(pieceyellowgreen['vertices'], 'yellowgreenvertices')
    print format_edges(pieceyellowgreen['edges'], 'yellowgreenedges')
    print format_vertices(piecebrown['vertices'], 'brownvertices')
    print format_edges(piecebrown['edges'], 'brownedges')

if __name__ == '__main__':
    generate_piece_defs()
    
