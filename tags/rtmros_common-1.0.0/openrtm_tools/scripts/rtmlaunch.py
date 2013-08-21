#!/usr/bin/env python

import roslib
roslib.load_manifest('openrtm_tools')

import sys
import os
import time
import optparse

from xml.dom.minidom import parse

import rtctree
from rtshell import rtcon
from rtshell import path
from rtshell import state_control_base
from rtshell import rts_exceptions

def alive_component(path):
    tree=rtctree.tree.RTCTree()
    if tree.has_path(path) and tree.is_component(path):
        node = tree.get_node(path)
        return node.plain_state_string
    else:
        return False

def wait_component(cmd_path):
    count=0
    path = rtctree.path.parse_path(cmd_path)[0]
    node = alive_component(path)
    while not node and count < 30:
        node = alive_component(path)
        print >>sys.stderr, "\033[33m[rtmlaunch] Wait for ",cmd_path," ",count,"/30\033[0m"
        count += 1
        time.sleep(1)
    if not node:
        raise rts_exceptions.NoSuchObjectError(cmd_path)
    return node

def check_connect(src_path, dest_path):
    tree=rtctree.tree.RTCTree()
    src_path, src_port = rtctree.path.parse_path(src_path)
    dest_path, dest_port = rtctree.path.parse_path(dest_path)
    src_node = tree.get_node(src_path)
    dest_node = tree.get_node(dest_path)
    port = src_node.get_port_by_name(src_port)
    for conn in port.connections:
        for name, p in conn.ports:
            tmp_dest_path, tmp_dest_port = rtctree.path.parse_path(name)
            if dest_path[-1] == tmp_dest_path[-1] and dest_port == tmp_dest_port:
                return True
    return False

def rtconnect(nameserver, tags):
    import re
    for tag in tags:
        source_path = nameserver+"/"+tag.attributes.get("from").value
        dest_path   = nameserver+"/"+tag.attributes.get("to").value
        source_path = re.sub("\$\(arg SIMULATOR_NAME\)",simulator,source_path);
        dest_path = re.sub("\$\(arg SIMULATOR_NAME\)",simulator,dest_path);
        # print >>sys.stderr, "[rtmlaunch] Connecting from %s to %s"%(source_path,dest_path)
        source_full_path = path.cmd_path_to_full_path(source_path)
        dest_full_path = path.cmd_path_to_full_path(dest_path)
        if tag.attributes.get("subscription_type") != None:
            sub_type = tag.attributes.get("subscription_type").value
            if not sub_type in ['flush','new','periodic']:
                print >>sys.stderr, sub_type+' is not a subscription type'
                continue
        else:
            sub_type = 'flush' # this is default value
        if sub_type == 'new':
            push_policy = 'all'
        # wait for proess
        try:
            wait_component(source_full_path)
            wait_component(dest_full_path)
            if check_connect(source_full_path,dest_full_path):
                continue
        except Exception, e:
            print >>sys.stderr, '\033[31m[rtmlaunch] [ERROR] Could not Connect : ', e,'\033[0m'
            return 1
        #print source_path, source_full_path, dest_path, dest_full_path;
        try:
            sub_type = str(sub_type)
            props = {'dataport.subscription_type': sub_type}
            if sub_type == 'new':
                props['dataport.publisher.push_policy'] = 'all'
            elif sub_type == 'periodic':
                props['dataport.publisher.push_policy'] = 'all'
                if tag.attributes.get("push_rate") != None:
                    props['dataport.push_rate'] = str(tag.attributes.get("push_rate").value)
                else:
                    props['dataport.push_rate'] = str('50.0')
            options = optparse.Values({'verbose': False, 'id': '', 'name': None, 'properties': props})
            print >>sys.stderr, "[rtmlaunch] Connected from",source_path
            print >>sys.stderr, "[rtmlaunch]             to",dest_path
            print >>sys.stderr, "[rtmlaunch]           with",options
            try :
                rtcon.connect_ports(source_path, source_full_path, dest_path, dest_full_path, options, tree=None)
            except Exception, e: # openrtm 1.1.0
                rtcon.connect_ports([(source_path,source_full_path), (dest_path, dest_full_path)], options, tree=None)
        except Exception, e:
            print >>sys.stderr, '\033[31m[rtmlaunch] {0}: {1}'.format(os.path.basename(sys.argv[1]), e),'\033[0m'
    return 0

def rtactivate(nameserver, tags):
    def activate_action(object, ec_index):
        object.activate_in_ec(ec_index)
    for tag in tags:
        cmd_path  = nameserver+"/"+tag.attributes.get("component").value
        full_path = path.cmd_path_to_full_path(cmd_path)
        # print >>sys.stderr, "[rtmlaunch] activate %s"%(full_path)
        try:
            state = wait_component(full_path)
            if state == 'Active':
                continue
            else:
                print >>sys.stderr, "[rtmlaunch] Activate <-",state,full_path
        except Exception, e:
            print >>sys.stderr, '\033[31m[rtmlaunch] Could not Activate : ', e,'\033[0m'
            return 1
        try:
            options = optparse.Values({"ec_index": 0, 'verbose': False})
            try :
                state_control_base.alter_component_state(activate_action, cmd_path, full_path, options, None)
            except Exception, e: # openrtm 1.1.0
                state_control_base.alter_component_states(activate_action, [(cmd_path, full_path)], options, None)
        except Exception, e:
            print >>sys.stderr, '[rtmlaunch] {0}: {1}'.format(os.path.basename(sys.argv[0]), e)
            return 1
    return 0

def main():
    global simulator
    usage = '''Usage: %prog [launchfile]'''
    if len(sys.argv) <= 1:
        print >>sys.stderr, usage
        return 1
    fullpathname = sys.argv[1]
    print >>sys.stderr, "[rtmlaunch] starting... ",fullpathname
    try:
        parser = parse(fullpathname)
        nodes = parser.getElementsByTagName("launch")[0].childNodes
        remove_nodes = []
        for node in nodes:
            if node.nodeName == u'group':
                val = node.getAttributeNode(u'if').value
                arg = val.split(" ")[1].strip(")") # To "USE_WALKING"
                if not get_flag_from_argv(arg):
                    remove_nodes.append(node)
        for remove_node in remove_nodes:
            nodes.remove(remove_node)
    except Exception,e:
        print e
        return 1

    if os.getenv("RTCTREE_NAMESERVERS") == None:
        print >>sys.stderr, "[rtmlaunch] RTCTREE_NAMESERVERS is not set, use localhost"
        nameserver = "localhost"
        os.environ["RTCTREE_NAMESERVERS"] = nameserver
    else:
        nameserver = os.getenv("RTCTREE_NAMESERVERS")

    simulator = os.getenv("SIMULATOR_NAME","Simulator")
    print >>sys.stderr, "[rtmlaunch] RTCTREE_NAMESERVERS", nameserver,  os.getenv("RTCTREE_NAMESERVERS")
    print >>sys.stderr, "[rtmlaunch] SIMULATOR_NAME", simulator
    while 1:
        print >>sys.stderr, "[rtmlaunch] check connection/activation"
        rtconnect(nameserver, parser.getElementsByTagName("rtconnect"))
        rtactivate(nameserver, parser.getElementsByTagName("rtactivate"))
        time.sleep(10)

def get_flag_from_argv(arg):
    for a in sys.argv:
        if arg in a: # If "USE_WALKING" is in argv
            return True if 'true' in a.split("=")[1] else False

if __name__ == '__main__':
    main()




