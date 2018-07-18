#!/usr/bin/python3
#-*- coding:utf-8 -*-
import re
import os
import sys

a_count = 0
decisions_nodes = {}
action_nodes = {}
reevaluate = {}
erbt = {}
edges = []
inherits = []

try:
    start_from = sys.argv[1]
except IndexError:
    start_from = ""

filelist = []

path_to_behaviour = "../src/bitbots_hcm/hcm_stack_machine"
subfolders = [""]

for sub in subfolders:
    decs = os.listdir(path_to_behaviour + ""+sub)
    for dec in decs:
        if ".py" in dec and not ".pyc" in dec:
            filelist.append(path_to_behaviour + "" + sub + "/" + dec)


acts = os.listdir(path_to_behaviour + "/actions")

for act in acts:
    if ".py" in act and not ".pyc" in act:
        filelist.append(path_to_behaviour + "/actions/" + act)


for datei in filelist[:]:
    fl = open(datei, "r")
    name = ""
    for line in fl:

        if "class" in line:
            name = re.findall(r"class (\w*)\((\w*)\)", line)
            if len(name):
                name = name[0]
            else:
                # somthing went wrong
                print("Could not match line! :", line)
                print("Drop File {} from Search!".format(datei))
                filelist.remove(datei)
                break
            #print line
            #print name
            if "actions" in datei:
                action_nodes[name[0]] = "action"
            else:
                decisions_nodes[name[0]] = "decision"

            if name[1] not in ["AbstractDecisionElement", "AbstractActionElement"]:
                #erbt[name[0]] = name[1]
                #nodes[name[0]].update(nodes[name[1]])
                #print edges
                extendlist = filter(lambda x: x[0] == name[1], edges)
                #print extendlist
                inherits.append((name[0], name[1]))
                """
                for _from, to in extendlist:
                    edges.append((name[0], to))
                """
                #edges.extend(extendlist)

    fl.close()

for datei in filelist:
    abstract = False
    edge_buffer = [] #alle kanten von der abstrakten klasse
    method = "" #aktuelle klasse
    method_buffer = {} # alle methoden der klasse
    class_buffer = []  # alle nicht abstrakten klassen
    fl = open(datei, "r")
    name = ""
    class_cache = []
    for line in fl:
        if "class" in line:
            name = re.findall(r"class (\w*)\((\w*)\)", line)[0]
            if "Abstract" in name[0]:
                abstract = True
                #print name[0]
            else:
                abstract = False
                class_buffer.append(name[0])
        elif "def " in line:
            if "get_reevaluate" in line:
                reevaluate[name[0]] = True
            #print line
            method = re.findall(r"def (\w*)\((.*)\)", line)[0]
            #print "Method: ", method[0]
            if abstract:
                method_buffer[method[0]] = []
            else:
                try:
                    method_buffer[method[0]].append(name[0]) # abstrakte methode wurde überschrieben und ist nicht mehr relevandt
                    #todo super calls beachten
                except:
                    pass

        elif "self.push(" in line:
            #print line
            to = re.findall(r"self.push\((\w*)", line)[0]
            #print to
            if to not in class_cache:
                if not abstract:
                    if to in action_nodes:

                        class_cache.append(to)
                        #to += str(a_count)
                        #print to
                        a_count += 1
                    edges.append([name[0], to])
                else:
                    if to in action_nodes:
                        #to += str(a_count)
                        #print to
                        a_count += 1
                    edge_buffer.append([name[0], method[0], to])


    for edge in edge_buffer:
        #if [edge[0], edge[1]] in method_buffer: # wenn methode nicht überschrieben
        for classe in class_buffer:
            if classe not in method_buffer.get(edge[1]):
                edges.append([classe, edge[2]])
    fl.close()
########
#filter#
########

filtered_decision_nodes = []
filtered_edges = []
eb = edges + inherits


def set_supressors(inode):
    for iedge in eb:
        #print iedge[0], " ", inode
        if iedge[0] == inode:
            if "Abstract" in iedge[1]:
                filtered_decision_nodes.append(iedge[1])

            filtered_decision_nodes.append(inode)
            filtered_edges.append(iedge)
            set_supressors(iedge[1])


if start_from:
    set_supressors(start_from)
else:
    filtered_edges = edges + inherits
    for d in decisions_nodes.keys():
        filtered_decision_nodes.append(d)


####
# Write down
####
writer = open("behaviour_graph.dt", "w")
writer.write("digraph Decisions {\n")

for node in decisions_nodes:
    if node in filtered_decision_nodes:
        if decisions_nodes[node] == "action":
            shape = "ellipse"
        else:
            shape = "box"
        writer.write(node + " [label=\"" + node + "\", shape=" + shape + "]")
        if node in reevaluate:
            writer.write(" [color=blue] ")
        if "Abstract" in node:
            #print node
            writer.write(" [fillcolor=\"#dddddd\"] [style=filled] ")
        writer.write(";\n")

writer.write("\n")


unique_edges = list([list(j) for j in set([tuple(i) for i in edges])])
for edge in unique_edges:
    if edge in filtered_edges:
        writer.write(edge[0] + "->" + edge[1] + "\n")

writer.write("edge [arrowhead=dot,arrowtail=dot];\n")

for edge in inherits:
    if edge in filtered_edges:
        writer.write(edge[1] + "->" + edge[0] + "\n")


writer.write("}")

writer.close()

os.system("dot behaviour_graph.dt -Tpng -o behaviour_graph.png")
os.remove("./behaviour_graph.dt")
