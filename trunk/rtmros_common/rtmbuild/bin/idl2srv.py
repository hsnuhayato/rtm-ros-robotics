#!/usr/bin/env python

from optparse import OptionParser
from omniidl import idlast, idlvisitor, idlutil, idltype
import _omniidl
import os, os.path, sys, string, re

# TODO
#  multi dimensional array -> *MultiArray.msg # ??
#  how to manipulate namespace
#  include idl ??
#  unit conversion, mm<->m

TypeNameMap = {} # for ROS msg/srv
TypeNameMap[idltype.tk_boolean] = 'bool'
TypeNameMap[idltype.tk_char] = 'int8'
TypeNameMap[idltype.tk_octet] = 'uint8'
TypeNameMap[idltype.tk_short] = 'int16'
TypeNameMap[idltype.tk_ushort] = 'uint16'
TypeNameMap[idltype.tk_long] = 'int32'
TypeNameMap[idltype.tk_ulong] = 'uint32'
TypeNameMap[idltype.tk_longlong] = 'int64'
TypeNameMap[idltype.tk_ulonglong] = 'uint64'
TypeNameMap[idltype.tk_float] = 'float32'
TypeNameMap[idltype.tk_double] = 'float64'
TypeNameMap[idltype.tk_string] = 'string'

# convert functions for IDL/ROS
convert_functions = """\n
template<class T1, class T2> inline
void convert(T1& in, T2& out){ out = (T2)in; }
template<>
void convert(char*& in, std::string& out){ out = std::string(in); }
template<>
void convert(std::string& in, char*& out){ out = (char*)in.c_str(); }
template<>
void convert(CORBA::String_member& in, std::string& out){ out = std::string(in); }
template<>
void convert(std::string& in, CORBA::String_member& out){ out = (char*)in.c_str(); }
template<class S,class T>
void convert(S& s, std::vector<T>& v){
  int size = s->length();
  v = std::vector<T>(s->length());
  for(int i=0; i<size; i++) convert((*s)[i],v[i]);}
template<class S,class T>
void convert(std::vector<T>& v, S& s){
  int size = v.size();
  s = S(size, size, S::allocbuf(size), 1);
  for(int i=0; i<=size; i++) convert(v[i],s[i]);}
"""

# visitor for generate msg/srv/cpp/h for bridge compornents
class ServiceVisitor (idlvisitor.AstVisitor):

    def visitAST(self, node):
        for n in node.declarations():
            n.accept(self)
    def visitModule(self, node):
        for n in node.definitions():
            n.accept(self)
    def visitInterface(self, node):
        self.outputMsg(node)
        for c in node.contents():
            c.accept(self)
        if node.mainFile():
            self.genBridgeComponent(node)

    def visitEnum(self, node):
        self.outputMsg(node)
    def visitStruct(self, node):
        self.outputMsg(node)

    def visitOperation(self, node):
        if node.mainFile():
            self.outputSrv(node)
        for n in node.parameters():
            n.accept(self)
##
##
##
    def getCppTypeText(self, typ, out=False):
        if isinstance(typ, idltype.Sequence):
            return ('%s*' if out else '%s') % self.getCppTypeText(typ.seqType(), False)
        if isinstance(typ, idltype.String):
            return 'char*' # ??
        if isinstance(typ, idltype.Declared):
            postfix = '*' if out and isinstance(typ.unalias(), idltype.Sequence) else ''
            return self.getCppTypeText(typ.decl(), False) + postfix
        if isinstance(typ, idlast.Struct):
            return idlutil.ccolonName(typ.scopedName())
        if isinstance(typ, idlast.Enum):
            return 'int' # enum is int ??
        if isinstance(typ, idlast.Typedef):
            return idlutil.ccolonName(typ.declarators()[0].scopedName())
        if isinstance(typ, idlast.Declarator):
            return self.getCppTypeText(typ.alias(), out)
        return 'undefined'


    def getROSTypeText(self, typ):
        if isinstance(typ, idltype.Base):
            return TypeNameMap[typ.kind()]
        if isinstance(typ, idltype.String):
            return 'string' # ??
        if isinstance(typ, idltype.Sequence):
            etype = typ.seqType() # n-dimensional array -> 1-dimensional
            size = typ.bound()
            while not (isinstance(etype, idltype.Base) or isinstance(etype, idltype.String) or isinstance(etype, idlast.Struct) or isinstance(etype, idlast.Enum)):
                if isinstance(etype, idltype.Declared):
                    etype = etype.decl()
                elif isinstance(etype, idltype.Sequence):
                    size *= etype.bound()
                    etype = etype.seqType()
                elif isinstance(etype, idlast.Typedef):
                    arrsize = [size] + etype.declarators()[0].sizes()
                    size = reduce(lambda a,b: a*b, arrsize)
                    etype = etype.aliasType()
                elif isinstance(etype, idlast.Declarator):
                    etype = etype.alias()
                else:
                    return 'undefined'
            return self.getROSTypeText(etype) + ('[]' if size==0 else '[%d]' % size)
        if isinstance(typ, idltype.Declared):
            return self.getROSTypeText(typ.decl())

        if isinstance(typ, idlast.Struct):
            return typ.identifier()
        if isinstance(typ, idlast.Const):
            return TypeNameMap[typ.constKind()]
        if isinstance(typ, idlast.Enum):
            return 'int64' # enum is int64
        if isinstance(typ, idlast.Typedef):
            arraysize = typ.declarators()[0].sizes()
            if 0 < len(arraysize):
                return self.getROSTypeText(typ.aliasType()) + ('[%d]' % reduce(lambda a,b: a*b, arraysize))
            return self.getROSTypeText(typ.aliasType())
        if isinstance(typ, idlast.Declarator):
            return self.getROSTypeText(typ.alias())

        return 'undefined'

    # output .msg file defined in .idl
    def outputMsg(self, typ):
        if not (isinstance(typ, idlast.Enum) or isinstance(typ, idlast.Struct) or isinstance(typ, idlast.Interface)):
            return

        idlfile = typ.file()
        idldir = os.path.split(idlfile)[0]
        basedir = os.path.split(idldir)[0]
        msgfile = basedir + "/msg/" + typ.identifier() + ".msg"
        if not os.path.exists(basedir):
            return
        if options.filenames:
            print msgfile
            return

        if os.path.exists(msgfile) and not options.overwrite:
            return # do not overwrite

        os.system('mkdir -p %s/msg' % basedir)
        fd = open(msgfile, 'w')
        if isinstance(typ, idlast.Enum):
            for val in typ.enumerators():
                fd.write("int64 %s=%d\n" % (val.identifier(), val.value()))
        elif isinstance(typ, idlast.Struct):
            for mem in typ.members():
                fd.write(self.getROSTypeText(mem.memberType()) + " " + mem.declarators()[0].identifier() + "\n")
        elif isinstance(typ, idlast.Interface):
            for mem in typ.contents():
                if isinstance(mem, idlast.Const):
                    fd.write("%s %s=%s\n" % (self.getROSTypeText(mem), mem.identifier(), mem.value()))
        fd.close()

    # output .srv file defined in .idl
    def outputSrv(self, op):
        idlfile = op.file()
        idldir = os.path.split(idlfile)[0]
        basedir = os.path.split(idldir)[0]

        srvfile = basedir + "/srv/" + op.identifier() + ".srv"
        if not os.path.exists(basedir):
            return
        if options.filenames:
            print srvfile
            return

        if os.path.exists(srvfile) and not options.overwrite:
            return # do not overwrite
        os.system('mkdir -p %s/srv' % basedir)
        args = op.parameters()

        fd = open(srvfile, 'w')
        for arg in [arg for arg in args if arg.is_in()]:
            fd.write("%s %s\n" % (self.getROSTypeText(arg.paramType()), arg.identifier()))
        fd.write("---\n")
        if op.returnType().kind() != idltype.tk_void:
            fd.write("%s operation_return\n" % self.getROSTypeText(op.returnType()))
        for arg in [arg for arg in args if arg.is_out()]:
            fd.write("%s %s\n" % (self.getROSTypeText(arg.paramType()), arg.identifier()))
        fd.close()

    def convertFunctionCode(self):
        code = ''
        for tn in idltype.declaredTypeMap:
            typ = idltype.declaredTypeMap[tn].decl() # ??
            if not isinstance(typ, idlast.Struct):
                continue

            code += 'template<> void convert(%s& in, %s::%s& out){\n' % (self.getCppTypeText(typ), pkgname, self.getCppTypeText(typ))
            for mem in typ.members():
                var = mem.declarators()[0].identifier()
                code += '  convert(out.%s, in.%s);\n' % (var, var)
            code += '}\n'

            code += 'template<> void convert(%s::%s& in, %s& out){\n' % (pkgname, self.getCppTypeText(typ), self.getCppTypeText(typ))
            for mem in typ.members():
                var = mem.declarators()[0].identifier()
                code += '  convert(out.%s, in.%s);\n' % (var, var)
            code += '}\n'

        return code

    def ServiceBridgeFunction(self, op, ifname, pkgname):
        code = req_code = res_code = ''
        params = []
        for par in op.parameters():
            is_out = par.is_out()
            ptype = par.paramType()
            var = par.identifier()
            # temporary variables
            if isinstance(ptype.unalias(), idltype.Base):
                if (ptype.kind() == idltype.tk_boolean) and is_out:
                    code += '  bool %s;\n' % var
                    res_code += '  res.%s = %s;\n' % (var, var)
                    params += [par.identifier()]
                else:
                    params += [('res.' if is_out else 'req.') + par.identifier()]
            if isinstance(ptype.unalias(), idltype.Sequence) or isinstance(ptype.unalias(), idltype.String) or isinstance(ptype.unalias(), idltype.Declared):
                code += '  %s %s;\n' % (self.getCppTypeText(ptype,is_out), var)
                if is_out:
                    res_code += '  convert(%s, res.%s);\n' % (var, var)
                else:
                    req_code += '  convert(req.%s, %s);\n' % (var, var)
                params += [var]
        if len(params) == 0:
            params = ''
        else:
            params = reduce(lambda a,b:a+', '+b, params)

        code += ('  ROS_INFO("call %s");\n' % op.identifier()) + '\n' + req_code + ("""
  // call service
  res.operation_return = m_service0->%s(%s);
""" % (op.identifier(), params)) + '\n' + res_code

        return """bool %s::%s(%s::%s::Request &req, %s::%s::Response &res){\n%s\n  return true;\n};\n\n""" % (ifname, op.identifier(), pkgname, op.identifier(), pkgname, op.identifier(), code)

    # generate cpp source to bridge RTM/ROS
    def genBridgeComponent(self, interface):
        idlfile = interface.file()
        module_name = '%sROSBridge' % interface.identifier()
        service_name = interface.identifier()
        idl_name = os.path.split(idlfile)[1]
        tmpdir = '/tmp/idl2srv'
        wd = basedir + '/src'

        Comp_cpp = wd + '/' + module_name + 'Comp.cpp'
        mod_cpp = wd + '/' + module_name + '.cpp'
        mod_h = wd + '/' + module_name + '.h'
        if options.filenames:
            print Comp_cpp
            print mod_cpp
            print mod_h
            return

        if os.path.exists(Comp_cpp) and os.path.exists(mod_cpp) and os.path.exists(mod_h) and not options.overwrite:
            return # do not overwrite

        command = "rosrun openrtm rtc-template -bcxx --module-name=%s --consumer=%s:service0:%s --consumer-idl=%s --idl-include=%s" % (module_name, service_name, service_name, idlfile, idldir)
        os.system("mkdir -p %s" % tmpdir)
        os.system("mkdir -p %s" % wd)
        os.system("cd %s; yes 2> /dev/null | %s > /dev/null" % (tmpdir, command))

        operations = interface.callables()

        def addline(src, dest, ref):
            idx = dest.find(ref)
            idx = dest.find('\n',idx)
            return dest[0:idx] + '\n' + src + dest[idx:]

        # Comp.cpp
        #  call ros::init in Comp.cpp
        compsrc = open(tmpdir + '/' + module_name + 'Comp.cpp').read()
        compsrc = addline('  ros::init(argc, argv, "' + module_name + '", ros::init_options::NoSigintHandler);', compsrc, 'RTC::Manager::init(argc, argv);')
        open(wd + '/' + module_name + 'Comp.cpp', 'w').write(compsrc)

        #.cpp
        #  make ROS service in onInitialize
        #  make ROS bridge functions in .cpp
        compsrc = open(tmpdir + '/' + module_name + '.cpp').read()
        compsrc += """
RTC::ReturnCode_t VehicleServiceROSBridge::onExecute(RTC::UniqueId ec_id) {
  ros::spinOnce();
  return RTC::RTC_OK;
}\n\n"""

        compsrc += convert_functions + self.convertFunctionCode()

        for i in range(len(operations)):
            name = operations[i].identifier()
            srvinst = '  _srv%d = nh.advertiseService("%s", &%s::%s, this);' % (i, name, module_name, name)
            compsrc = addline(srvinst, compsrc, 'Bind variables and configuration variable')
        for op in operations:
            compsrc += self.ServiceBridgeFunction(op, module_name, pkgname)
        open(wd + '/' + module_name + '.cpp', 'w').write(compsrc)

        #.h
        #  add ros headers, service server functions, uncomment onExecute
        compsrc = open(tmpdir + '/' + module_name + '.h').read()
        compsrc = re.sub(basedir+"/idl/(.+).h", r'\1.h', compsrc)

        incs = ['', '// ROS', '#include <ros/ros.h>']
        incs += ['#include <%s/%s.h>' % (pkgname, op.identifier()) for op in operations]
        incs = '\n'.join(incs)
        compsrc = addline(incs, compsrc, '#define')

        compsrc = '\n'.join([ (a.replace('//','') if ('RTC::ReturnCode_t onExecute' in a) else a) for a in compsrc.split('\n')])

        srvfunc = ['  bool %s(%s::%s::Request &req, %s::%s::Response &res);' % (op.identifier(), pkgname, op.identifier(), pkgname, op.identifier()) for op in operations]
        srvfunc = '\n'.join(srvfunc)
        compsrc = addline(srvfunc, compsrc, 'public:')

        defsrv = "  ros::ServiceServer " + ', '.join(['_srv%d' % i for i in range(len(operations))]) + ';'
        compsrc = addline(defsrv, compsrc, 'private:')

        compsrc = addline("  ros::NodeHandle nh;", compsrc, 'private:')

        open(wd + '/' + module_name + '.h', 'w').write(compsrc)

        # finialize
        os.system("rm -rf %s" % tmpdir)
        return


# visitor only prints interface names
class InterfaceNameVisitor (idlvisitor.AstVisitor):
    def visitAST(self, node):
        for n in node.declarations():
            n.accept(self)
    def visitModule(self, node):
        for n in node.definitions():
            n.accept(self)
    def visitInterface(self, node):
        if options.interfaces and node.mainFile():
            print node.identifier()


if __name__ == '__main__':
    global options, pkgname

    parser = OptionParser()
    parser.add_option("-i", "--idl", dest="idlfile",
                      help="target idl file", metavar="FILE")
    parser.add_option("-I", "--include-dirs", dest="idlpath", metavar="PATHLIST",
                      help="list of directories to check idl include")
    parser.add_option("-o", "--overwrite", action="store_true",
                      dest="overwrite", default=False,
                      help="overwrite all generate files")
    parser.add_option("--filenames", action="store_true",
                      dest="filenames", default=False,
                      help="print filenames to generate")
    parser.add_option("--interfaces", action="store_true",
                      dest="interfaces", default=False,
                      help="print interface names")
    (options, args) = parser.parse_args()

    idlfile = options.idlfile
    if not os.path.exists(idlfile):
        exit
    idlfile = os.path.abspath(idlfile)
    idldir = os.path.split(idlfile)[0]
    basedir = os.path.split(idldir)[0]
    pkgname = os.path.split(basedir)[1] # global var

    # preproccess and compile idl
    if options.idlpath:
        pathlist = options.idlpath.strip('"')
        option = ' '.join(['-I'+d for d in pathlist.split(' ')])
    else:
        option = ''

    fd = os.popen('/usr/bin/omnicpp %s "%s"' % (option, idlfile), 'r')
    tree = _omniidl.compile(fd)

    # output msg/srv and bridge component src
    if options.interfaces:
        tree.accept(InterfaceNameVisitor())
    else:
        tree.accept(ServiceVisitor())
