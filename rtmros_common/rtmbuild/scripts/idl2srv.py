#!/usr/bin/env python

from optparse import OptionParser
from omniidl import idlast, idlvisitor, idlutil, idltype
from omniidl_be import cxx
import _omniidl
import os, os.path, sys, string, re

# TODO
# not generate unused msgs
# how to manipulate namespace -> under score concat

TypeNameMap = { # for ROS msg/srv
    idltype.tk_boolean:   ('bool',    'uchar'),
    idltype.tk_char:      ('int8',    'char'),
    idltype.tk_octet:     ('uint8',   'uchar'),
    idltype.tk_wchar:     ('int16',   'short'),
    idltype.tk_short:     ('int16',   'short'),
    idltype.tk_ushort:    ('uint16',  'ushort'),
    idltype.tk_long:      ('int32',   'int'),
    idltype.tk_ulong:     ('uint32',  'uint'),
    idltype.tk_longlong:  ('int64',   'long long'),
    idltype.tk_ulonglong: ('uint64',  'unsigned long long'),
    idltype.tk_float:     ('float32', 'float'),
    idltype.tk_double:    ('float64', 'double'),
p    idltype.tk_string:    ('string',  'undef'),
    idltype.tk_wstring:   ('string',  'undef'),
    idltype.tk_any:       ('string',  'undef'), # ??
    idltype.tk_TypeCode:  ('uint64',  'undef'), # ??
    idltype.tk_enum:      ('uint64',  'unsigned long long') }

MultiArrayTypeNameMap = { # for ROS msg/srv
    idltype.tk_char:     'std_msgs/Int8MultiArray',
    idltype.tk_octet:    'std_msgs/Uint8MultiArray',
    idltype.tk_wchar:    'std_msgs/Int16MultiArray',
    idltype.tk_short:    'std_msgs/Int16MultiArray',
    idltype.tk_ushort:   'std_msgs/Uint16MultiArray',
    idltype.tk_long:     'std_msgs/Int32MultiArray',
    idltype.tk_ulong:    'std_msgs/Uint32MultiArray',
    idltype.tk_longlong: 'std_msgs/Int64MultiArray',
    idltype.tk_ulonglong:'std_msgs/Uint64MultiArray',
    idltype.tk_float:    'std_msgs/Float32MultiArray',
    idltype.tk_double:   'std_msgs/Float64MultiArray'}

# convert functions for IDL/ROS
# _CORBA_String_element type is env depend ??
convert_functions = """\n
template<typename T,typename S>
void genseq(S& s, int size, _CORBA_Unbounded_Sequence<T>* p){
  s = S(size,size,S::allocbuf(size), 1);}
template<typename T,typename S,int n>
void genseq(S& s, int size, _CORBA_Bounded_Sequence<T,n>* p){
  s = S(size,S::allocbuf(size), 1);}
template<class T1, class T2> inline
void convert(T1& in, T2& out){ out = static_cast<T2>(in); }
template<typename T>
void convert(T& in, std::string& out){ out = std::string(in); }
template<typename T>
void convert(std::string& in, T& out){ out = static_cast<T>(in.c_str()); }
void convert(_CORBA_String_element in, std::string& out){ out = std::string(in); }
void convert(std::string& in, _CORBA_String_element out){ out = (const char*)in.c_str(); }
template<class S,class T>
void convert(S& s, std::vector<T>& v){
  int size = s.length();
  v = std::vector<T>(s.length());
  for(int i=0; i<size; i++) convert(s[i],v[i]);}
template<class S,class T>
void convert(std::vector<T>& v, S& s){
  int size = v.size();
  s = S(size, size, S::allocbuf(size), 1);
  for(int i=0; i<size; i++) convert(v[i],s[i]);}
template<class S,class T,std::size_t n>
void convert(S& s, boost::array<T,n>& v){
  for(std::size_t i=0; i<n; i++) convert(s[i],v[i]);}
template<class S,class T,std::size_t n>
void convert(boost::array<T,n>& v, S& s){
  for(std::size_t i=0; i<n; i++) convert(v[i],s[i]);}
"""

multiarray_conversion = """
template<class S> // convert multi-dimensional array
void convert(S& s, %s& v){
  if(v.layout.dim.size()==0 || v.layout.dim[v.layout.dim.size()-1].size==0) {
    int level = v.layout.dim.size();
    v.layout.dim.push_back(std_msgs::MultiArrayDimension());
    v.layout.dim[level].stride = 1;
    for(uint i=0; i<s.length(); i++){
      convert(s[i], v);
      if(i==0) v.layout.dim[level].size = s.length(); // initialized flag
    }
    for(uint i=level; i<v.layout.dim.size(); i++)
      v.layout.dim[level].stride *= v.layout.dim[level].size;
  } else {
    for(uint i=0; i<s.length(); i++){ convert(s[i], v); }
  }
}
template<>
void convert(%s& s, %s& v){ v.data.push_back(s); }
template<class S>
void convert(%s& v, S& s){
  int level, size;
  for(level=0; (size=v.layout.dim[level].size)==0; level++);
  genseq(s, size, &s);
  v.layout.dim[level].size = 0;
  for(uint i=0; i<s.length(); i++)
    convert(v, s[i]);
  v.layout.dim[level].size = size;
}
template<>
void convert(%s& v, %s& s){ convert(v.data[v.layout.data_offset++], s); }
""" # % (std_msgs::Float64MultiArray, double, std_msgs::Float64MultiArray) + (std_msgs::Float64MultiArray, std_msgs::Float64MultiArray, double)

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
    def getCppTypeText(self, typ, out=False, full=False):
        if isinstance(typ, idltype.Base):
            return TypeNameMap[typ.kind()][1]
        if isinstance(typ, idltype.String):
            return ('char*' if out else 'const char*') # ??
        if isinstance(typ, idltype.Declared):
            postfix = ('*' if out and cxx.types.variableDecl(typ.decl()) else '')
            return self.getCppTypeText(typ.decl(), False, full) + postfix

        if isinstance(typ, idlast.Struct):
            name = (idlutil.ccolonName(typ.scopedName()) if full else typ.identifier())
            return name + ('*' if out and cxx.types.variableDecl(typ) else '')
        if isinstance(typ, idlast.Enum):
            if full:
                return idlutil.ccolonName(typ.scopedName())
            else:
                return typ.identifier()
        if isinstance(typ, idlast.Typedef):
            if full:
                return idlutil.ccolonName(typ.declarators()[0].scopedName())
            else:
                return typ.declarators()[0].identifier()
        if isinstance(typ, idlast.Declarator):
            return self.getCppTypeText(typ.alias(), out, full)
        return 'undefined'


    def getROSTypeText(self, typ):
        if isinstance(typ, idltype.Base):
            return TypeNameMap[typ.kind()][0]
        if isinstance(typ, idltype.String) or isinstance(typ, idltype.WString):
            return 'string' # ??
        if isinstance(typ, idltype.Sequence):
            etype = typ.seqType() # n-dimensional array -> 1-dimensional
            size = typ.bound()
            dim = 1
            while not (isinstance(etype, idltype.Base) or isinstance(etype, idltype.String) or isinstance(etype, idltype.WString) or isinstance(etype, idlast.Struct) or isinstance(etype, idlast.Enum) or isinstance(etype, idlast.Forward)):
                if isinstance(etype, idltype.Declared):
                    etype = etype.decl()
                elif isinstance(etype, idltype.Sequence):
                    dim += 1
                    size *= etype.bound()
                    etype = etype.seqType()
                elif isinstance(etype, idlast.Typedef):
                    arrsize = [size] + etype.declarators()[0].sizes()
                    if(len(arrsize) != 1): dim += 1
                    size = reduce(lambda a,b: a*b, arrsize)
                    etype = etype.aliasType()
                elif isinstance(etype, idlast.Declarator):
                    etype = etype.alias()
#                elif isinstance(etype, idlast.Forward):
#                    etype = etype.fullDecl()
                else:
                    return 'undefined'
            if( 1 < dim ):
                return MultiArrayTypeNameMap[etype.kind()]
            return self.getROSTypeText(etype) + ('[]' if size==0 else '[%d]' % size)
        if isinstance(typ, idltype.Declared):
            return self.getROSTypeText(typ.decl())

        if isinstance(typ, idlast.Interface):
            return typ.identifier()
        if isinstance(typ, idlast.Struct):
            return typ.identifier()
        if isinstance(typ, idlast.Const):
            return TypeNameMap[typ.constKind()][0]
        if isinstance(typ, idlast.Enum):
            return TypeNameMap[idltype.tk_longlong][0] # enum is int64 ??
        if isinstance(typ, idlast.Union):
            return TypeNameMap[idltype.tk_double][0] # union is not in ROS
        if isinstance(typ, idlast.Typedef):
            arraysize = typ.declarators()[0].sizes()
            if 0 < len(arraysize):
                return self.getROSTypeText(typ.aliasType()) + ('[%d]' % reduce(lambda a,b: a*b, arraysize))
            return self.getROSTypeText(typ.aliasType())
        if isinstance(typ, idlast.Declarator):
            return self.getROSTypeText(typ.alias())
        if isinstance(typ, idlast.Forward):
            return self.getROSTypeText(typ.fullDecl())

        return 'undefined'

    # output .msg file defined in .idl
    def outputMsg(self, typ):
        if not (isinstance(typ, idlast.Enum) or isinstance(typ, idlast.Struct) or isinstance(typ, idlast.Interface)):
            return

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
                fd.write("%s %s=%d\n" % (self.getROSTypeText(typ), val.identifier(), val.value()))
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
        if not op.oneway() and op.returnType().kind() != idltype.tk_void:
            fd.write("%s operation_return\n" % self.getROSTypeText(op.returnType()))
        for arg in [arg for arg in args if arg.is_out()]:
            fd.write("%s %s\n" % (self.getROSTypeText(arg.paramType()), arg.identifier()))
        fd.close()

    def convertFunctionCode(self, interface):
        visitor = DependencyVisitor()
        interface.accept(visitor)
        code = ''

        for typ in visitor.multiarray:
            msg = MultiArrayTypeNameMap[typ.kind()].replace('/','::')
            cpp = TypeNameMap[typ.kind()][1]
            code += multiarray_conversion % (msg,cpp,msg,msg,msg,cpp)

        for typ in visitor.allmsg:
            if not isinstance(typ, idlast.Struct):
                continue

            code += 'template<> void convert(%s& in, %s::%s& out){\n' % (self.getCppTypeText(typ, full=True), pkgname, self.getCppTypeText(typ))
            for mem in typ.members():
                var = mem.declarators()[0].identifier()
                code += '  convert(in.%s, out.%s);\n' % (var, var)
            code += '}\n'

            code += 'template<> void convert(%s::%s& in, %s& out){\n' % (pkgname, self.getCppTypeText(typ), self.getCppTypeText(typ, full=True))
            for mem in typ.members():
                var = mem.declarators()[0].identifier()
                code += '  convert(in.%s, out.%s);\n' % (var, var)
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
            if isinstance(ptype.unalias(), idltype.Base) or \
               isinstance(ptype.unalias(), idltype.String) or \
               isinstance(ptype.unalias(), idltype.Sequence) or \
               isinstance(ptype.unalias(), idltype.Declared):
                code += '  %s %s;\n' % (self.getCppTypeText(ptype, out=is_out, full=True), var)
                params += [var]
            if isinstance(ptype.unalias(), idltype.Base) or \
               isinstance(ptype.unalias(), idltype.String):
                if is_out:
                    res_code += '  convert(%s, res.%s);\n' % (var, var)
                else:
                    req_code += '  convert(req.%s, %s);\n' % (var, var)
            if isinstance(ptype.unalias(), idltype.Sequence):
                if is_out:
                    res_code += '  convert(*%s, res.%s);\n' % (var, var)
                else:
                    req_code += '  convert(req.%s, %s);\n' % (var, var)
            if isinstance(ptype.unalias(), idltype.Declared):
                if is_out:
                    ptr = ('*' if cxx.types.variableDecl(ptype.decl()) else '')
                    res_code += '  convert(%s%s, res.%s);\n' % (ptr, var, var)

                else:
                    req_code += '  convert(req.%s, %s);\n' % (var, var)

        params = ', '.join(params)
        code += ('  ROS_INFO("call %s");\n' % op.identifier()) + '\n' + req_code

        if op.oneway() or op.returnType().kind() == idltype.tk_void:
            code += '  m_service0->%s(%s);\n' % (op.identifier(), params)
        elif isinstance(op.returnType().unalias(), idltype.Base):
            code += '  res.operation_return = m_service0->%s(%s);\n' % (op.identifier(), params)
        else:
            rtype = op.returnType()
            if isinstance(rtype.unalias(), idltype.String):
                ptr = ''
            elif isinstance(rtype.unalias(), idltype.Sequence):
                ptr = '*'
            elif isinstance(rtype.unalias(), idltype.Declared):
                ptr = ('*' if cxx.types.variableDecl(rtype.decl()) else '')
            else: ptr = ''
            code += '  %s operation_return;\n' % self.getCppTypeText(rtype, out=True, full=True)
            code += '  operation_return = m_service0->%s(%s);\n' % (op.identifier(), params)
            code += '  convert(%soperation_return, res.operation_return);\n' % ptr


        code += res_code

        return """bool %s::%s(%s::%s::Request &req, %s::%s::Response &res){\n%s\n  return true;\n};\n\n""" % (ifname, op.identifier(), pkgname, op.identifier(), pkgname, op.identifier(), code)

    # generate cpp source to bridge RTM/ROS
    def genBridgeComponent(self, interface):
        idlfile = interface.file()
        module_name = '%sROSBridge' % interface.identifier()
        #service_name = idlutil.ccolonName(interface.scopedName())
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

        command = "rosrun openrtm rtc-template -bcxx --module-name=%s --consumer=%s:service0:'%s' --consumer-idl=%s --idl-include=%s" % (module_name, service_name, service_name, idlfile, idldir)
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

        port_name_src = """  std::string port_name;
  nh.param<std::string>("service_port", port_name, "service0");"""
        compsrc = addline(port_name_src, compsrc, 'Set service consumers to Ports')
        compsrc = compsrc.replace('registerConsumer("service0"','registerConsumer(port_name.c_str()')

        compsrc += """
RTC::ReturnCode_t %s::onExecute(RTC::UniqueId ec_id) {
  ros::spinOnce();
  return RTC::RTC_OK;
}\n\n""" % module_name

        compsrc += convert_functions + self.convertFunctionCode(interface)

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

        compsrc = compsrc.replace('<%s>'%service_name, '<%s>'%idlutil.ccolonName(interface.scopedName()))

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

# all types depended by a interface
class DependencyVisitor (idlvisitor.AstVisitor):
    def checkBasicType(self, node):
        classes = [idltype.Base, idltype.String, idltype.WString]
        classes += [idlast.Const, idlast.Enum, idlast.Union]
        if not any([isinstance(node, cls) for cls in classes]):
            node.accept(self)

    def visitInterface(self, node):
        self.allmsg = []
        self.multiarray = []
        if node.mainFile():
            for n in node.callables():
                n.accept(self)
    def visitOperation(self, node):
        types = [p.paramType() for p in node.parameters()]
        types += [node.returnType()]
        for n in types:
            self.checkBasicType(n)

    def visitStruct(self, node):
        for mem in node.members():
            self.checkBasicType(mem.memberType())
        if not node in self.allmsg: # add self after all members
            self.allmsg += [node]

    def visitSequenceType(self, node):
        etype = node.seqType()
        dim = 1
        while not (isinstance(etype, idltype.Base) or isinstance(etype, idltype.String) or isinstance(etype, idltype.WString) or isinstance(etype, idlast.Struct) or isinstance(etype, idlast.Enum)):
            if isinstance(etype, idltype.Sequence):
                dim += 1
                etype = etype.seqType()
            elif isinstance(etype, idltype.Declared):
                etype = etype.decl()
            elif isinstance(etype, idlast.Typedef):
                if len(etype.declarators()[0].sizes()) != 0 : dim += 1
                etype = etype.aliasType()
            elif isinstance(etype, idlast.Declarator):
                etype = etype.alias()
            elif isinstance(etype, idlast.Forward):
                etype = etype.fullDecl()
        if 1 < dim and isinstance(etype, idltype.Base) and (not etype in self.multiarray) :
            self.multiarray += [etype]
        self.checkBasicType(node.seqType())
    def visitDeclaredType(self, node):
        self.checkBasicType(node.decl())
    def visitTypedef(self, node):
        self.checkBasicType(node.aliasType())
    def visitDeclarator(self, node):
        self.checkBasicType(node.alias())
    def visitForward(self, node):
        self.checkBasicType(node.fullDecl())


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
    global options, basedir, pkgname

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
