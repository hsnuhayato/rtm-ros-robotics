##
## scene_objects.py
## 
## R.Hanai 2010.12.08 - 
##


from math import *


# World
def world():
    return {'name' : 'world',
            'shape' : None,
            'children' : []}

# Floor
def floor():
    w,d,h = 3000.0, 3000.0, 20.0
    return {'name' : 'floor',
            'shape' : 'box',
            'color': (0.1, 0.1, 0.2),
            'material' : 'rough',
            'dimension' : (w, d, h),
            'children' : []}

# Table
def low_table():
    w,d,h = 900.0, 550.0, 450.0
    return {'name' : 'table',
            'shape' : 'box',
            'color' : (1, 0.8, 0.6),
            'material' : 'wood',
            'dimension' : [w, d, h],
            'children' : []}

# Table with 4 legs
def high_table():
    w,d,h,r = 1400, 700, 700, 16
    thickness = 40
    l = h - thickness
    a = w/2.0 -100
    b = d/2.0 -100

    topcol = (0.8, 0.8, 0.8)
    legcol = (0.6, 0.5, 0.4)
    legmat = 'wood'
    topmat = None

    top = {'name' : 'table top',
           'shape' : 'box',
           'color' : topcol,
           'material' : topmat,
           'dimension' : [w, d, thickness],
           'children' : []}
    leg1 = {'name' : 'table leg1',
           'shape' : 'cylinder',
           'color' : legcol,
           'material' : legmat,
           'dimension' : [l, r],
           'children' : []}
    leg2 = {'name' : 'table leg2',
           'shape' : 'cylinder',
           'color' : legcol,
           'material' : legmat,
           'dimension' : [l, r],
           'children' : []}
    leg3 = {'name' : 'table leg3',
           'shape' : 'cylinder',
           'color' : legcol,
           'material' : legmat,
           'dimension' : [l, r],
           'children' : []}
    leg4 = {'name' : 'table leg4',
           'shape' : 'cylinder',
           'color' : legcol,
           'material' : legmat,
           'dimension' : [l, r],
           'children' : []}
    table = {'name' : 'table',
             'shape' : None,
             'children' : [(leg1, [b,a,0,0,0,0]),
                           (leg2, [-b,a,0,0,0,0]),
                           (leg3, [b,-a,0,0,0,0]),
                           (leg4, [-b,-a,0,0,0,0]),
                           (top, [0,0,h-thickness/2,0,0,0])]}
    return table

# Basket
def basket():
    w,d,h = 330.0, 245.0, 65.0
    thickness = 3.0
    # l = h - thickness
    a = w/2.0
    b = d/2.0
    c = h/2.0

    col = (0.0, 0.4, 0.0)
    mat = None

    side1 = {'name' : 'basket side1',
             'shape' : 'box',
             'color' : col,
             'material' : mat,
             'dimension' : [w,thickness,h],
             'children' : []}
    side2 = {'name' : 'basket side2',
             'shape' : 'box',
             'color' : col,
             'material' : mat,
             'dimension' : [w,thickness,h],
             'children' : []}
    side3 = {'name' : 'basket side3',
             'shape' : 'box',
             'color' : col,
             'material' : mat,
             'dimension' : [thickness,d,h],
             'children' : []}
    side4 = {'name' : 'basket side4',
             'shape' : 'box',
             'color' : col,
             'material' : mat,
             'dimension' : [thickness,d,h],
             'children' : []}
    bottom = {'name' : 'basket bottom',
              'shape' : 'box',
              'color' : col,
              'material' : mat,
              'dimension' : [w,d,thickness],
              'children' : []}
    basket = {'name' : 'basket',
              'shape' : None,
              'children' : [(side1, [b,0,c,0,0,0]),
                            (side2, [-b,0,c,0,0,0]),
                            (side3, [0,a,c,0,0,0]),
                            (side4, [0,-a,c,0,0,0]),
                            (bottom, [0,0,0,0,0,0])
                            ]}
    return basket


def wall(name='wall0'):
    w,d,h = 20.0, 400.0, 200.0
    return {'name' : name,
            'shape' : 'box',
            'color' : (0.6, 0.5, 0.4),
            'material' : None,
            'dimension' : [w, d, h],
            'children' : []}

# Reference Hardware
def rhrobo():
    w,d,h = 550.0, 500.0, 300.0
    return {'name' : 'RH',
            'shape' : 'box',
            'color' : (1.0, 1.0, 1.0),
            'material' : None,
            'dimension' : [w, d, h],
            'children' : []}

# Can
def can(name):
    l,r = 103.0, 26.0
    return {'name' : name,
            'shape' : 'cylinder',
            'color' : (0, 1.0, 0.2),
            'material' : None,
            'dimension' : (l, r),
            'children' : []}

# parts A
def partsA(name):
    W,H,L = 38.0, 48.0, 28.0
    return {'name' : name,
            'shape' : 'box',
            'color' : (0.5, 0.5, 0.5),
            'material' : None,
            'dimension' : [W, H, L],
            'children' : []}

# parts B
def partsB(name):
    D,L = 25.0, 58.0
    return {'name' : name,
            'shape' : 'cylinder',
            'color' : (0.4, 0.4, 0.4),
            'material' : None,
            'dimension' : (L, D/2.0),
            'children' : []}

def W0(name):
    return {'name' : name,
            'shape' : 'mesh',
            'children' : []}

def rect_pocket(name):
    W,H,L = 43.0, 53.0, 10.0
    return {'name' : name,
            'shape' : 'box',
            'color' : (0.8, 0.0, 0.8),
            'material' : None,
            'dimension' : [W, H, L],
            'children' : []}

# pallete
def pallete(name):
    w,d,h = 300.0, 215.0, 85.0
    thickness = 2.0
    bottom_thickness = 20.0
    # l = h - thickness
    a = w/2.0
    b = d/2.0
    c = h/2.0

    col = (0.7, 0.7, 0.7)
    mat = None

    side1 = {'name' : 'pallete side1',
             'shape' : 'box',
             'color' : col,
             'material' : mat,
             'dimension' : [w,thickness,h],
             'children' : []}
    side2 = {'name' : 'pallete side2',
             'shape' : 'box',
             'color' : col,
             'material' : mat,
             'dimension' : [w,thickness,h],
             'children' : []}
    side3 = {'name' : 'pallete side3',
             'shape' : 'box',
             'color' : col,
             'material' : mat,
             'dimension' : [thickness,d,h],
             'children' : []}
    side4 = {'name' : 'pallete side4',
             'shape' : 'box',
             'color' : col,
             'material' : mat,
             'dimension' : [thickness,d,h],
             'children' : []}
    bottom = {'name' : 'pallete bottom',
              'shape' : 'box',
              'color' : (0.2,0.2,0.2),
              'material' : mat,
              'dimension' : [w,d,bottom_thickness],
              'children' : []}
    pllt = {'name' : name,
            'shape' : None,
            'children' : [(side1, [b,0,c,0,0,0]),
                          (side2, [-b,0,c,0,0,0]),
                          (side3, [0,a,c,0,0,0]),
                          (side4, [0,-a,c,0,0,0]),
                          (bottom, [0,0,bottom_thickness/2,0,0,0])
                          ]}
    return pllt

def hirobase():
    w,d,h=320.0,255.0,800.0
    return {'name' : 'hirobase',
            'shape' : 'box',
            'color' : (0.2, 0.2, 0.2),
            'material' : None,
            'dimension' : [d,w,h],
            'children' : []}


##
## Scene for Integrated Verification
##
def iv_scene():
    w = world()
    fl = floor()
    tbl = low_table()
    rh = rhrobo()
    tbldepth = tbl['dimension'][1]
    tblheight = tbl['dimension'][2]
    rhheight = rh['dimension'][2]

    rh['children'] = [(can('hole1'),[50,50,rhheight/2,0,0,0]),
                      (can('can2'),[50,-50,rhheight/2,0,0,0]),
                      (can('can3'),[-50,50,rhheight/2,0,0,0]),
                      (can('can4'),[-50,-50,rhheight/2,0,0,0])]
    w['children'] = [(fl,[0,0,0,0,0,0]),
                     (tbl,[0,(1100+tbldepth)/2,tblheight/2,0,0,pi/2]),
                     (rh,[750,0,rhheight/2,0,0,pi])]
    return w


##
## Empty scene
##
def empty_scene():
    w = world()
    fl = floor()
    w['children'].append((fl, [0,0,0,0,0,0]))
    return w

##
## Table scene
##
def table_scene():
    w = world()
    fl = floor()
    tbl = high_table()
    bskt = basket()
    wl0 = wall(name='wall0')
    wl1 = wall(name='wall1')
    w['children'].append((tbl, [500,0,0,0,0,0]))
    w['children'].append((fl, [0,0,0,0,0,0]))
    tbl['children'].append((bskt, [-200,-370,700,0,0,0]))
    tbl['children'].append((wl0, [-160,-180,800,0,0,0]))
    tbl['children'].append((wl1, [-160,-50,1150,pi/2,0,0]))
    return w

##
## Table scene with 2 cans
##
def ac_scene():
    w = world()
    fl = floor()
    bs = hirobase()
    tbl = high_table()
    pllt = pallete(name='pallete0')
    w['children'].append((tbl, [500,0,0,0,0,0]))
    w['children'].append((fl, [0,0,0,0,0,0]))
    w['children'].append((bs, [-40.0-150.0,0,400.0,0,0,0]))
    tbl['children'].append((pllt, [-220,-310,700,0,0,-pi/3]))

    A0 = partsA(name='A0')
    tbl['children'].append((A0, [-260,-50,714,0,0,pi/6]))
    A1 = partsA(name='A1')
    tbl['children'].append((A1, [-170,100,714,0,0,-pi/6]))
    A2 = partsA(name='A2')
    tbl['children'].append((A2, [-250,190,714,0,0,0]))
    A3 = partsA(name='A3')
    tbl['children'].append((A3, [-120,-10,714,0,0,pi/4]))

    B0 = partsB(name='B0')
    tbl['children'].append((B0, [-160,210,700,0,0,0]))
    B1 = partsB(name='B1')
    tbl['children'].append((B1, [-120,-70,700,0,0,0]))

    p1 = rect_pocket(name='P0')
    pllt['children'].append((p1, [40,40,20,0,0,0]))
    p2 = rect_pocket(name='P1')
    pllt['children'].append((p2, [-40,40,20,0,0,0]))
    p3 = rect_pocket(name='P2')
    pllt['children'].append((p3, [40,-40,20,0,0,0]))
    p4 = rect_pocket(name='P3')
    pllt['children'].append((p4, [-40,-40,20,0,0,0]))
    return w
