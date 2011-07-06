##
## mplan_env,py
## 
## R.Hanai 2010.11.30 - 
##


from viewer import *


class NameCollisionError(Exception):
    def __init__(self, msg):
        self.msg = msg
    def __str__(self):
        return self.msg


class MPlanEnv:
    def __init__(self):
        self.scene_objects = []
        self.mdlrobot = None
        self.tgtrobot = None
        self.markers = []


    #
    # markers
    #
    def add_marker(self, frm):
        vf = VFrame(FRAME(mat=frm.mat, vec=frm.vec), size=100.0, width=5.0)
        vf.set_visible(True)
        self.markers.append(vf)

    def clear_markers(self):
        for vf in self.markers:
            vf.set_visible(False)
        self.markers = []


    #
    # scene objects
    #
    def get_object(self, name):
        objs = [x for x in self.scene_objects if x.name == name]
        # 2 objects with a same name is not permitted
        if objs:
            return objs[0]
        else:
            return None

    def get_objects(self):
        return self.scene_objects

    def get_world(self):
        return self.get_object('world')

    # def insert_robot(self, robot, frame, parent):
    #     self.insert_object(robot.basejoint, frame, parent)

    def insert_object(self, obj, frame, parent=None):
        # check the names of existing objects
        if obj.name and self.get_object(obj.name):
            raise NameCollisionError(obj.name + ' cannot be used for 2 objects')

        self.scene_objects.append(obj)

        if parent:
            obj.affix(parent, frame)

    def delete_object(self, name):
        obj = self.get_object(name)

        # delete all children
        delete_candidate = [obj]
        for o in self.scene_objects:
            if o.ancestor(obj) and o != obj:
                delete_candidate.append(o)

        for o in delete_candidate:
            if isinstance(o, CoordinateObject):
                o.vframe.set_visible(False)
                #o.vbody.frame.set_visible(False)
                self.scene_objects.remove(o)
                # del self.scene_objects[name]

    def show_frames(self):
        print 'not yet implemented'

    def eval_sctree(self, ast):
        try:
            nm = ast['name']
        except:
            nm = None

        shape = ast['shape']

        if shape == None:
            obj = CoordinateObjectWithName(name=nm)
        else:
            col = ast['color']

            mat = ast['material']        
            if mat == 'wood':
                mat = visual.materials.wood
            elif mat == 'rough':
                mat = visual.materials.rough
            else:
                mat = None

            if shape == 'box':
                w,d,h = ast['dimension']
                body = visual.box(length=d, height=w, width=h,
                                  color=col, material=mat)
                obj = PartsObjectWithName(vbody=body, name=nm)
            elif shape == 'cylinder':
                l,r = ast['dimension']
                body = visual.cylinder(axis=(0,0,l), radius=r, color=col)
                obj = PartsObjectWithName(vbody=body, name=nm)            

        for child in ast['children']:
            cobj = self.eval_sctree(child[0])
            self.insert_object(cobj, FRAME(xyzabc = child[1]), parent=obj)

        obj.vframe.resize(1.0)
        return obj

    def load_scene(self, sctree):
        self.insert_object(self.eval_sctree(sctree), FRAME())

    def recog_env(self, name):
        obj = world[name]
        tr = obj.where()
        if obj.vbody.__class__ == visual.box:
            # approximate with an axis-aligned bounding box
            dim = obj.vbody.size
            # visual.vector to geo.VECTOR conversion
            dx = 0.5*VECTOR(x=dim[0],y=0,z=0) 
            dy = 0.5*VECTOR(x=0,y=dim[1],z=0)
            dz = 0.5*VECTOR(x=0,y=0,z=dim[2])
            pts_local = []
            pts_local.append(dx + dy + dz)
            pts_local.append(dx + dy - dz)
            pts_local.append(dx - dy + dz)
            pts_local.append(dx - dy - dz)
            pts_local.append(- dx + dy + dz)
            pts_local.append(- dx + dy - dz)
            pts_local.append(- dx - dy + dz)
            pts_local.append(- dx - dy - dz)
            pts = map(lambda pt: tr*pt, pts_local)
            minx = miny = minz = 1e+10
            maxx = maxy = maxz = -1e+10
            for pt in pts:
                if pt[0] < minx:
                    minx = pt[0]
                if pt[0] > maxx:
                    maxx = pt[0]
                if pt[1] < miny:
                    miny = pt[1]
                if pt[1] > maxy:
                    maxy = pt[1]
                if pt[2] < minz:
                    minz = pt[2]
                if pt[2] > maxz:
                    maxz = pt[2]
            return space.CAABB([minx,maxx,miny,maxy,minz,maxz])
        elif obj.vbody.__class__ == visual.cylinder:
            axvec_local = VECTOR(z=obj.vbody.length)
            return space.CCapsule(tr.vec, tr.mat*axvec_local, obj.vbody.radius)
        else:
            print 'undefined shape'
            return None
