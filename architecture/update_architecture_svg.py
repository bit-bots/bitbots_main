#!/usr/bin/env python3
import os
import subprocess
import xml.etree.ElementTree as ET
import rospkg


def create_node_onclick(package_name, package_xml_path):
    """ Load information from the package.xml file and put it into an onclick event."""
    package_tree = ET.parse(package_xml_path)
    package_root = package_tree.getroot()

    name = package_root.find("name").text
    version = package_root.find("version").text
    description = package_root.find("description").text
    # remove linebreaks from xml
    description = description.replace("\n", "")
    maintainer = package_root.find("maintainer").text
    maintaimer_mail = package_root.find("maintainer").attrib['email']
    # authors = package_root.findall("author").text
    # license = package_root.find("license").text
    print(name)
    doc_tag = package_root.find("export/bitbots_documentation")
    if doc_tag is None:
        language = "not defined"
        language_img = ""
    else:
        language_img = ""
        try:
            language = doc_tag.find("language").text
        except AttributeError:
            language = ""
        if language.lower() == "python":
            language_img = "python.svg"
        elif language.lower() == "python2" or language.lower() == "python 2":
            language_img = "python2.svg"
        elif language.lower() == "python3" or language.lower() == "python 3":
            language_img = "python3.svg"
        elif language.lower() == "c++" or language.lower() == "cpp":
            language_img = "c++.svg"


    text_version = "Version: " + version
    text_description = "Description: " + description
    maintainer_title = "Maintainer: "

    # add zero width space to break on _ or / in heading
    package_name = package_name.replace('_', u'_\u200b').replace('/', u'/\u200b')

    onclick_js = "document.getElementById('description-heading').textContent = '" + package_name + "';" \
                 "document.getElementById('description-version').innerText = '" + text_version + "';" \
                 "document.getElementById('description-text').innerText = '" + text_description + "';" \
                 "document.getElementById('description-text').className = 'indented';" \
                 "document.getElementById('description-maintainer-mail').innerText = '" + maintainer + "';" \
                 "document.getElementById('description-maintainer-title').innerText = '" + maintainer_title + "';" \
                 "document.getElementById('description-text').style.whiteSpace = 'normal';"

    if maintaimer_mail:
        onclick_js += "document.getElementById('description-maintainer-mail').href = 'mailto:" + maintaimer_mail + "';"
    else:
        onclick_js += "document.getElementById('description-maintainer-mail').href = '';"

    if language_img:
        onclick_js += "document.getElementById('description-language').src = '" + language_img + "';" \
                      "document.getElementById('description-language').style.display = 'block';"
    else:
        onclick_js += "document.getElementById('description-language').style.display = 'none';"

    return onclick_js


def create_topic_onclick(package_name, definition):
    """ Load information from the message definition and put it into an onclick event."""

    # warning text if definition is none
    if not definition:
        def_text = "Definition not found"
    else:
        def_text = definition.decode('utf8')
        # fix line breaks
        def_text = def_text.replace("\n", "\\n")

    # add zero width space to break on _ or / in heading
    package_name = package_name.replace('_', u'_\u200b').replace('/', u'/\u200b')

    onclick_js = "document.getElementById('description-heading').textContent = '" + package_name + "';" \
                 "document.getElementById('description-text').innerText = '" + def_text + "';" \
                 "document.getElementById('description-text').style.whiteSpace = 'pre';" \
                 "document.getElementById('description-text').className = '';" \
                 "document.getElementById('description-version').innerText = '';" \
                 "document.getElementById('description-maintainer-title').innerText = '';" \
                 "document.getElementById('description-maintainer-mail').innerText = '';" \
                 "document.getElementById('description-maintainer-mail').href = '';" \
                 "document.getElementById('description-language').style.display = 'none';"

    return onclick_js


def get_status_color(package_xml_path):
    """ Get the status color of the package. This is taken from the package.xml"""
    unknown = "#ffffff"

    package_tree = ET.parse(package_xml_path)
    package_root = package_tree.getroot()

    doc_tag = package_root.find("export/bitbots_documentation")
    if doc_tag is None:
        return unknown

    status = doc_tag.find("status")
    if status is None:
        return unknown

    status = status.text
    if status == "unknown":
        return "#999999"
    elif status == "broken":
        return "#FF0000"
    elif status == "compiles":
        return "#f48024"
    elif status == "starts":
        return "#ffff00"
    elif status == "tested_viz":
        return "#20b2aa"
    elif status == "tested_simulator":
        return "#98fb98"
    elif status == "tested_robot":
        return "#00ff00"
    elif status == "tested_integration":
        return "#3cb371"
    elif status == "stable":
        return "#228b22"
    else:
        return "#999999"

def do_documentation_for(robotname):
    dia_file_name = robotname + ".dia"
    svg_file_name = robotname + ".svg"
    html_header = """<!DOCTYPE html>
    <html lang="en">
    <head>
        <meta charset="UTF-8">
        <title>Bitbots</title>
        <style>
            #description {
                font-size: 1.5em;
                position: fixed;
                right: 0;
                top: 0;
                width: 20%;
                background-color: white;
                border: double;
                padding: 0 10px;
                margin: 10px;
                border-radius: 10px;
                font-family: sans-serif;
            }
            #color {
                position: fixed;
                right: 0;
                bottom: 0;
                width: 10%;
                background-color: white;
                border: double;
                padding: 0 10px;
                margin: 10px;
                border-radius: 10px;
                font-family: sans-serif;
            }
            #description-heading {
                font-size: 2em;
                font-variant: small-caps;
                word-wrap: break-word;
            }
            #description-language {
                position: absolute;
                top: 14px;
                right: 14px;
                height: 30px;
                width: 30px;
                background-size: 30px;
            }
            .indented {
                padding-left: 10px;
                text-indent: -10px;
            }
            svg {
                margin-right: 30%;
            }
        </style>
    </head>
    <body>
    """
    html_footer = """<div id="description">
            <h1 id="description-heading">Bit-Bots Node Overview</h1>
            <p id="description-version"></p>
            <p id="description-text">Click on a node to get more information</p>
            <p><a id="description-maintainer-title"></a><a id="description-maintainer-mail"></a></p>
            <img id="description-language">
        </div>
        <div id="color">
            <table>
            <tr>
            <td bgcolor="#999999" style="min-width:50px"></td>
            <td>unknown</td>
            </tr>
            <tr>
            <td bgcolor="#FF0000" style="min-width:50px"></td>
            <td>broken</td>
            </tr>
            <tr>
            <td bgcolor="#f48024" style="min-width:50px"></td>
            <td>compiles</td>
            </tr>
            <tr>
            <td bgcolor="#ffff00" style="min-width:50px"></td>
            <td>starts</td>
            </tr>
            <tr>
            <td bgcolor="#20b2aa" style="min-width:50px"></td>
            <td>tested_viz</td>
            </tr>
            <tr>
            <td bgcolor="#98fb98" style="min-width:50px"></td>
            <td>tested_simulator</td>
            </tr>
            <tr>
            <td bgcolor="#00ff00" style="min-width:50px"></td>
            <td>tested_robot</td>
            </tr>
            <tr>
            <td bgcolor="#3cb371" style="min-width:50px"></td>
            <td>tested_integration</td>
            </tr>
            <tr>
            <td bgcolor="#228b22" style="min-width:50px"></td>
            <td>stable</td>
            </tr>
            </table>
        </div>
    </body>
    </html>"""

    print("Welcome to the fancy architecture script, please remember to source bitbots_meta before using this.")

    # export svg file
    print("Exporting dia file \"" + dia_file_name + "\" to svg.")
    os.system("dia -e " + svg_file_name + " -t svg " + dia_file_name)

    # read xml content of svg file
    svg_tree = ET.parse(svg_file_name)
    svg_root = svg_tree.getroot()


    #
    # Get used packages, messages, services actions
    #

    # get all the package names
    package_names = set([])
    rospack = rospkg.RosPack()
    for group in svg_root.findall(
            "./{http://www.w3.org/2000/svg}g[{http://www.w3.org/2000/svg}ellipse]"):
        ellipses = group.findall("./{http://www.w3.org/2000/svg}ellipse")
        name = group.findall("./{http://www.w3.org/2000/svg}text/{http://www.w3.org/2000/svg}tspan")[0].text
        path = rospack.get_path(name) + "/package.xml"
        # change color of the node, by using the white filled background ellipse
        for ellipse in ellipses:
            if "fill: #ffffff" in ellipse.attrib["style"]:
                ellipse.attrib["style"] = ellipse.attrib["style"].replace("#ffffff", get_status_color(path))
        # create onclick on group so that it also works if the text is clicked
        group.attrib["onclick"] = create_node_onclick(name, path)
        package_names.add(name)

    print("Found packages: " + str(package_names))

    # get all the message, service, action types
    message_names = set([])
    service_names = set([])
    action_names = set([])
    for group in svg_root.findall(
            "./{http://www.w3.org/2000/svg}g[{http://www.w3.org/2000/svg}rect]"):
        # there are two texts, we want the second for the type
        tspans = group.findall("./{http://www.w3.org/2000/svg}text/{http://www.w3.org/2000/svg}tspan")
        # we have to look which rects are services, action or topics, first find the right rectangle
        rects = group.findall("./{http://www.w3.org/2000/svg}rect")
        border_rect = None
        definition = None
        for r in rects:
            # the rect with stroke is the border rect
            if "stroke" in r.attrib["style"]:
                border_rect = r
        if "stroke-dasharray: 20" in border_rect.attrib["style"]:
            # this is dashed -> its a service, remove .srv
            name = tspans[1].text[:-4]
            service_names.add(name)
            if "/" not in name:
                name = "humanoid_league_msgs/" + name
            try:
                definition = subprocess.check_output("rossrv show " + name, shell=True)
            except:
                print("Couldn't find service " + name + " Please check the .dia file.")
                continue
        elif "stroke-dasharray: 4" in border_rect.attrib["style"]:
            # this is pointed -> its an action, remove .action
            name = tspans[1].text[:-7]
            action_names.add(name)
            if "/" not in name:
                name = "humanoid_league_msgs/" + name
            try:
                definition = subprocess.check_output("rosmsg show " + name, shell=True)
            except:
                print("Couldn't find action " + name + " Please check the .dia file.")
                continue
        else:
            # solid border -> its a topic
            name = tspans[1].text
            message_names.add(name)
            if "/" not in name:
                name = "humanoid_league_msgs/" + name
            try:
                definition = subprocess.check_output("rosmsg show " + name, shell=True)
            except:
                print("Couldn't find message type " + name + " Please check the .dia file.")
                continue
        group.attrib["onclick"] = create_topic_onclick(name, definition)

    print("\n")
    print("Found message types: " + str(message_names))
    print("Found service types: " + str(service_names))  # todo improve output when empty
    print("Found action types: " + str(action_names))

    #
    # Generate html code
    #

    html_svg = ET.tostring(svg_root).decode()
    html_svg = html_svg.replace('ns0:', '')
    html = html_header + html_svg + html_footer

    with open(robotname+'.html', 'w') as f:
        f.write(html)

do_documentation_for("Wolfgang")
