#!/usr/bin/env python
'''This script automates many of the tasks necessary to port a ROS (1)
catkin package to a ROS 2 ament package.
Modified from https://github.com/bponsler/ros2-support

'''
import os
import re
import argparse
import tempfile
from os.path import exists
from xml.dom import minidom
import xml.etree.ElementTree as etree
from pathlib import Path

# Load a module for parsing cmake lists content
try:
    import parse_cmake.parsing as cmkp
except ImportError:
    import traceback

    traceback.print_exc()
    print("ERROR: You must install the parse_cmake python module!")
    print("")
    print("    sudo pip install parse_cmake")
    exit(1)

# Names of package files to modify
PACKAGE_XML = "package.xml"
CMAKELISTS = "CMakeLists.txt"

# List of ROS packages that do not currently exist in ROS 2
UNKNOWN_ROS_PACKAGES = [
    "dynamic_reconfigure",
    "roslib",
    "rosconsole"
]

# List of ROS packages that have been renamed in ROS 2
RENAMED_ROS_PACKAGES = {
    "tf": "tf2",
    "roscpp": "rclcpp",
    "rospy": "rclpy",
    "nodelet": "rclcpp",  # nodelets have become components in ROS 2
    "message_generation": "rosidl_default_generators",
    "message_runtime": "rosidl_default_runtime",
}

# List of packages that do not need to be found by CMake
NO_CMAKE_FIND_PACKAGES = [
    "rosidl_default_runtime",
]

MESSAGES = {
    "JointState",
    "Odometry",
    "Char",
    "Imu",
    "SupportState",
    "TransformStamped"
}

RENAME_PACKAGE_IMPORTS = {
    "ros/ros.h": "rclcpp/rclcpp.hpp"
}

REMOVE_IMPORTS = [
    "ros/console.h"
]

HARDCODED_REPLACEMENTS = {
    "#include <std_msgs/Time.h>": "#include <builtin_interfaces/msg/time.hpp>"
}


def source_code_replacement():
    files = list(Path(".").rglob(r"*"))
    cpp_files = []
    for file in files:
        if not "CMake" in file.name and (".cpp" in file.name or ".h" in file.name or ".hpp" in file.name):
            cpp_files.append(file)
    for filename in cpp_files:
        # replace the regex with the replacement in the given file
        with open(filename, "r+") as f:
            content = f.read()
            # rename packages that we want to import
            for find, replace in RENAME_PACKAGE_IMPORTS.items():
                content = re.sub(find, replace, content)
            # rename message imports, since they have an extra "/msg" and are .hpp files
            for message in MESSAGES:
                message_snake_case = re.sub(r'(?<!^)(?=[A-Z])', '_', message).lower()
                # import
                content = re.sub(message + ".h>", "msg/" + message_snake_case+".hpp>", content)
                # usage
                content = re.sub("::" + message, "::msg::" + message, content)

            for package in REMOVE_IMPORTS:
                content = re.sub(r"#include <" + package + ">", "", content)

            for find, replace in HARDCODED_REPLACEMENTS.items():
                content = re.sub(find, replace, content)

            f.seek(0)
            f.write(content)


def executeSedCmd(pattern, filename, dryrun=False):
    # Choose flags based on whether the file should be modified or not
    sedFlags = "" if dryrun else "-i"

    cmd = "sed %s '%s' %s" % (sedFlags, pattern, filename)
    return os.system(cmd) == 0


class PackageXmlPorter:
    @classmethod
    def port(cls, dryrun=False):
        # Make sure the file exists (even though this is already checked)
        if not exists(PACKAGE_XML):
            print("ERROR: Unable to locate the package XML: %s" % PACKAGE_XML)
            return False

        # Parse the package XML
        tree = etree.parse(PACKAGE_XML)
        packageRoot = tree.getroot()

        # ROS 2 only supports package XML format version 2
        packageRoot.set("format", "2")

        # Make sure there's a final newline
        packageRoot.tail = "\n"

        # List of package root element indices to remove
        removeElements = []

        # TODO: need to handle the different semantics of tags between format 1 and 2
        #       see: http://www.ros.org/reps/rep-0140.html

        generatesMessages = False
        foundExport = False
        foundBuildTool = False
        for child in packageRoot.getchildren():
            # Handle specific elements
            if child.tag == "build_depend":
                # Message generation no longer exists
                if child.text == "message_generation":
                    generatesMessages = True
                    removeElements.append(child)
            elif child.tag == "run_depend":
                # The run_depend tag has changed to exec_depend in format 2
                child.tag = "exec_depend"

                # Remove message generation
                if child.text == "message_generation":
                    generatesMessages = True
                    removeElements.append(child)
            elif child.tag == "buildtool_depend":
                # Update the package to use ament instead of catkin
                if child.text == "catkin":
                    child.text = "ament_cmake"
                    foundBuildTool = True
            elif child.tag == "export":
                foundExport = True

                # Found an export, check children for build type
                foundBuildType = False
                for export in child.getchildren():
                    # The build type needs to be specified for ament packages
                    if export.tag == "build_type":
                        export.text = "ament_cmake"
                        foundBuildType = True

                # If the build type element was not found, then we must
                # add one to specify that this package used ament
                if not foundBuildType:
                    buildTypeElement = etree.Element("build_type")
                    buildTypeElement.text = "ament_cmake"
                    buildTypeElement.tail = "\n  "  # Spacing for export close

                    # Add spacing for open of the build type element
                    if len(child.getchildren()) > 0:
                        lastExport = child.getchildren()[-1]
                        lastExport.tail = "\n    "  # Spacing to open of build type
                    else:
                        child.tail = "\n    "  # Spacing to build type element

                    child.append(buildTypeElement)

        # Remove all desired elements
        for element in removeElements:
            packageRoot.remove(element)

        # If this package generates messages, then certain dependencies
        # need to be added to the package
        if generatesMessages:
            # TODO: only add this when Duration or Time are used within the messages/services
            # buildToolElement = etree.Element("depend")
            # buildToolElement.text = "builtin_interfaces"
            # buildToolElement.tail = "\n  "  # Spacing to next element

            buildToolElement = etree.Element("buildtool_depend")
            buildToolElement.text = "rosidl_default_generators"
            buildToolElement.tail = "\n  "  # Spacing to next element

            execDependElement = etree.Element("exec_depend")
            execDependElement.text = "rosidl_default_runtime"
            execDependElement.tail = "\n  "  # Spacing to next element

            # Add spacing before the open of the build tool depend element
            lastChild = packageRoot.getchildren()[-1]
            lastChild.tail = "\n\n  "  # Spacing for open build tool depend

            packageRoot.append(buildToolElement)
            packageRoot.append(execDependElement)

        # If the build tool was not specified, add it (it should
        # always be specified, but just in case)
        if not foundBuildTool:
            buildToolElement = etree.Element("buildtool_depend")
            buildToolElement.text = "ament_cmake"
            buildToolElement.tail = "\n"  # Spacing to next element

            # Add spacing before the open of the build tool depend element
            lastChild = packageRoot.getchildren()[-1]
            lastChild.tail = "\n\n  "  # Spacing for open build tool depend

            packageRoot.append(buildToolElement)

        # If the export element was not found, we must create one to
        # specify that this package used ament
        if not foundExport:
            # Create the build type element
            buildTypeElement = etree.Element("build_type")
            buildTypeElement.text = "ament_cmake"
            buildTypeElement.tail = "\n  "  # Spacing for export close

            # Create the wrapping export element
            exportElement = etree.Element("export")
            exportElement.text = "\n    "  # Spacing for open build type
            exportElement.tail = "\n"  # Spacing for package close
            exportElement.append(buildTypeElement)

            # Add spacing before the open of the export element
            lastChild = packageRoot.getchildren()[-1]
            lastChild.tail = "\n\n  "  # Spacing for open export

            packageRoot.append(exportElement)

        if not dryrun:
            # Write the content to the file
            tree.write(PACKAGE_XML, encoding='utf-8', xml_declaration=True)
        else:
            # Write the XML to a temporary file
            fid, tempFilename = tempfile.mkstemp("_catkin_to_ament")
            os.close(fid)
            tree.write(tempFilename, encoding='utf-8', xml_declaration=True)

            # Read the temporary file
            fid = open(tempFilename, 'r')
            content = fid.read().strip()
            fid.close()

            # Delete the temporary file, now that we're done with it
            os.remove(tempFilename)

            # Print the content
            print(content)

        return True  # Success


class CmakeListsPorter:
    @classmethod
    def port(cls, dryrun=False):
        # Make sure the file exists (even though this is already checked)
        if not exists(CMAKELISTS):
            print("ERROR: Unable to locate CMakeLists.txt")
            return False

        # Read the file content
        fid = open(CMAKELISTS, "r")
        content = fid.read()
        fid.close()

        # Parse the cmake content
        cmake = cmkp.parse(content)

        # Item indices to remove from the file
        removeIndices = []

        # List of catkin packages this package depends on
        catkinDepends = []

        # Libraries created by this package
        packageLibs = []

        # Message and service files to generate
        msgsAndSrvs = []

        projectDeclIndex = -1
        hasCpp11 = False
        for index, item in enumerate(cmake):
            # Skip non commmand items
            if type(item) != type(cmkp.Command("a", "b")):
                continue

            # Grab names of all arguments to this command
            args = [b.contents for b in item.body]

            if item.name == "cmake_minimum_required":
                # Make sure to use at least cmake 3.5
                if len(args) == 2 and args[0] == "VERSION":
                    item.body[1] = cmkp.Arg("3.5")
            elif item.name == "project":
                projectDeclIndex = index
            elif item.name == "add_definitions":
                # Handle C++11 support added through definitions
                if "-std=c++11" in args:
                    hasCpp11 = True
            elif item.name == "set":
                # Handle C++11 support specifically set to CXX flags
                if "CMAKE_CXX_FLAGS" in args:
                    for arg in args:
                        if "-std=c++11" in arg:
                            hasCpp11 = True
                            break
            elif item.name == "find_package":
                if len(args) > 0 and "catkin" == args[0]:
                    removeIndices.append(index)

                    # An example of command is:
                    #     find_package(catkin REQUIRED COMPONENTS pkg1 pkg2)
                    if "COMPONENTS" in args:
                        componentsStart = args.index("COMPONENTS")
                        catkinDepends = args[componentsStart + 1:]

                        generatesMessages = ("message_generation" in args)

                        # Remove packages that no longer exist in ROS 2
                        catkinDepends = list(filter(
                            lambda p: p not in UNKNOWN_ROS_PACKAGES,
                            catkinDepends))

                        # Handle packages that have been renamed in ROS 2
                        catkinDepends = list(map(
                            lambda p: RENAMED_ROS_PACKAGES.get(p, p),
                            catkinDepends))

                        # Add additional packages needed for message generation
                        if generatesMessages:
                            catkinDepends.extend([
                                "rosidl_default_generators",
                                "rosidl_default_runtime",
                            ])
            elif item.name == "catkin_package":
                # Remove the catkin_packge element
                removeIndices.append(index)
            elif item.name == "include_directories":
                # Remove the include directories, which will be added later
                removeIndices.append(index)
            elif item.name == "link_directories":
                # Remove the link directories, which will be added later
                removeIndices.append(index)
            elif item.name == "catkin_destinations":
                # Remove this command as it no longer exists
                removeIndices.append(index)
            elif item.name == "catkin_metapackage":
                # Remove this command as it no longer exists
                removeIndices.append(index)
            elif item.name == "catkin_python_setup":
                # Remove this command as it no longer exists
                removeIndices.append(index)
            elif item.name == "add_dependencies":
                # The catkin exported targets variable no longer exists,
                # so remove this
                if "${catkin_EXPORTED_TARGETS}" in args:
                    removeIndices.append(index)
            elif item.name == "target_link_libraries":
                # Replace the reference to catkin libraries (which no longer
                # exists) with a reference to libraries variable
                if "${catkin_LIBRARIES}" in args:
                    catkinIndex = args.index("${catkin_LIBRARIES}")
                    item.body[catkinIndex] = cmkp.Arg("${LIBS}")
            elif item.name == "add_library":
                # Keep track of the names of all libraries created
                # by this package
                if len(args) > 0:
                    packageLibs.append(args[0])
            elif item.name == "add_message_files":
                if len(args) > 1:
                    msgFiles = list(map(
                        lambda s: "msg/%s" % s, args[1:]))
                    msgsAndSrvs.extend(msgFiles)

                # Remove this command as it has been replaced
                removeIndices.append(index)
            elif item.name == "add_service_files":
                if len(args) > 1:
                    serviceFiles = list(map(
                        lambda s: "srv/%s" % s, args[1:]))
                    msgsAndSrvs.extend(serviceFiles)

                # Remove this command as it has been replaced
                removeIndices.append(index)
            elif item.name == "generate_messages":
                # Remove this command as it has been replaced
                removeIndices.append(index)
            elif item.name == "if":
                # Replace if(CATKIN_ENABLE_TESTING) with if(BUILD_TESTING)
                if len(args) == 1 and args[0] == "CATKIN_ENABLE_TESTING":
                    item.body[0] = cmkp.Arg("BUILD_TESTING")
            elif item.name == "catkin_add_gtest":
                # Replace catkin_add_gtest with ament_add_gtest
                cmake[index] = cmkp.Command("ament_add_gtest", item.body)
            elif item.name == "catkin_add_nosetests":
                # Replace catkin_add_nosetests with ament_add_nose_test, add
                # an additional arg which is the name of the test. Add a unique
                # name that can be changed as desired later
                body = [cmkp.Arg("${PROJECT_NAME}_nose_test_%s" % index)] + item.body
                cmake[index] = cmkp.Command("ament_add_nose_test", body)

        # Should never happen, but just in case...
        if projectDeclIndex == -1:
            print("ERROR: Failed to locate project declaration!")
            return False

        # Remove all indices in reverse sorted order to prevent the
        # indices from changing values
        for index in sorted(removeIndices, reverse=True):
            del cmake[index]

        # Make sure C++11 support is added
        if not hasCpp11:
            comment = cmkp.Comment("# Add support for C++17")

            openIf = cmkp.Command("if", [cmkp.Arg("NOT"), cmkp.Arg("CMAKE_CXX_STANDARD")])

            addDef = cmkp.Command(
                "set", [cmkp.Arg("CMAKE_CXX_STANDARD"), cmkp.Arg("17")])

            closeIf = cmkp.Command("endif", [])

            # Add all the items
            items = [
                cmkp.BlankLine(),
                comment,
                openIf,
                addDef,
                closeIf,
                cmkp.BlankLine(),
            ]
            for item in items:
                projectDeclIndex += 1
                cmake.insert(projectDeclIndex, item)

        ## Add all find_package calls

        # Must find the ament_cmake package
        catkinDepends.insert(0, "ament_cmake")

        # Remove duplicate entries
        catkinDepends = list(set(catkinDepends))

        # Remove packages that don't need to be found by CMake
        for pkg in NO_CMAKE_FIND_PACKAGES:
            if pkg in catkinDepends:
                catkinDepends.remove(pkg)

        # Add calls to find all other dependency packages
        for pkg in catkinDepends:
            findPkg = cls.__findPackage(pkg)
            projectDeclIndex += 1
            cmake.insert(projectDeclIndex, findPkg)

        # Add a blank line
        projectDeclIndex = cls.__addBlankLine(cmake, projectDeclIndex)

        ## Handle message generation
        if len(msgsAndSrvs) > 0:
            # rosidl_generate_interfaces(urg_node_msgs
            #    "msg/Status.msg"
            #    DEPENDENCIES builtin_interfaces std_msgs
            # )
            cmdArgs = [cmkp.Arg("${PROJECT_NAME}")]
            for filename in msgsAndSrvs:
                cmdArgs.append(cmkp.Arg('"%s"' % filename))

            # Add dependencies on message packages
            cmdArgs.extend([
                cmkp.Arg("DEPENDENCIES"),
                cmkp.Arg("builtin_interfaces"),
            ])
            for pkg in catkinDepends:
                if pkg.endswith("_msgs") or pkg.endswith("srvs"):
                    cmdArgs.append(cmkp.Arg(pkg))

            genIfaceCmd = cmkp.Command("rosidl_generate_interfaces", cmdArgs)
            projectDeclIndex += 1
            cmake.insert(projectDeclIndex, genIfaceCmd)

            # Add a blank line
            projectDeclIndex = cls.__addBlankLine(cmake, projectDeclIndex)

        ## Define variables for all include dirs and libraries
        includeArgs = [cmkp.Arg("INCLUDE_DIRS")]
        libArgs = [cmkp.Arg("LIBS")]
        libDirArgs = [cmkp.Arg("LIBRARY_DIRS")]
        for pkg in catkinDepends:
            includeArgs.append(cmkp.Arg("${%s_INCLUDE_DIRS}" % pkg))
            libArgs.append(cmkp.Arg("${%s_LIBRARIES}" % pkg))
            libDirArgs.append(cmkp.Arg("${%s_LIBRARY_DIRS}" % pkg))

        # If an include directory exists for this package, add it to
        # the include dirs
        if exists("include"):
            includeArgs.insert(1, cmkp.Arg("include"))

        # Add command to set include dirs
        setIncludeDirs = cmkp.Command("set", includeArgs)
        projectDeclIndex += 1
        cmake.insert(projectDeclIndex, setIncludeDirs)

        ## Add the include_directories command
        includeDirs = cmkp.Command(
            "include_directories", [cmkp.Arg("${INCLUDE_DIRS}")])
        projectDeclIndex += 1
        cmake.insert(projectDeclIndex, includeDirs)
        projectDeclIndex = cls.__addBlankLine(cmake, projectDeclIndex)

        # Add command to set lib dirs
        setLibDirs = cmkp.Command("set", libDirArgs)
        projectDeclIndex += 1
        cmake.insert(projectDeclIndex, setLibDirs)
        projectDeclIndex = cls.__addBlankLine(cmake, projectDeclIndex)

        ## Add the link_directories command
        linkDirs = cmkp.Command(
            "link_directories", [cmkp.Arg("${LIBRARY_DIRS}")])
        projectDeclIndex += 1
        cmake.insert(projectDeclIndex, linkDirs)
        projectDeclIndex = cls.__addBlankLine(cmake, projectDeclIndex)

        # Add command to set libs
        setLibs = cmkp.Command("set", libArgs)
        projectDeclIndex += 1
        cmake.insert(projectDeclIndex, setLibs)
        projectDeclIndex = cls.__addBlankLine(cmake, projectDeclIndex)

        ## Export all ament dependencies at the bottom of the file
        cmake.append(cmkp.BlankLine())
        for pkg in catkinDepends:
            export = cmkp.Command("ament_export_dependencies", [cmkp.Arg(pkg)])
            cmake.append(export)

        ## Export include directories
        exportIncludes = cmkp.Command(
            "ament_export_include_directories",
            [cmkp.Arg("${INCLUDE_DIRS}")])
        cmake.append(exportIncludes)

        ## Export all known libraries
        if len(packageLibs) > 0:
            exportLibs = cmkp.Command(
                "ament_export_libraries",
                [cmkp.Arg(lib) for lib in packageLibs])

            # Include all dependency libraries
            exportLibs.body.append(cmkp.Arg("${LIBS}"))
            cmake.append(exportLibs)

        # Add the final call to initialize the ament package
        # (this must be at the bottom of the file!)
        projectDeclIndex = cmake.append(cmkp.BlankLine())
        amentPackageCmd = cmkp.Command("ament_package", [])
        cmake.append(amentPackageCmd)

        # Remove any double blank lines
        index = 0
        while index < (len(cmake) - 1):
            isBlank = lambda i: (type(i) == type(cmkp.BlankLine()))
            item = cmake[index]
            nextItem = cmake[index + 1]

            if isBlank(item) and isBlank(nextItem):
                del cmake[index]
                continue  # Repeat this index

            index += 1

        # Convert the CMakeLists content into a nicely formatted string
        cmakeData = '\n'.join(cmkp.compose_lines(
            cmake, cmkp.FormattingOptions())) + '\n'

        # Replace all instances of variables that no longer exist
        renamedVariables = {
            "${CATKIN_DEVEL_PREFIX}/": "",
            "${CATKIN_GLOBAL_BIN_DESTINATION}": "bin",
            "${CATKIN_GLOBAL_INCLUDE_DESTINATION}": "include",
            "${CATKIN_GLOBAL_LIB_DESTINATION}": "lib",
            "${CATKIN_GLOBAL_LIBEXEC_DESTINATION}": "lib",
            "${CATKIN_GLOBAL_SHARE_DESTINATION}": "share",
            "${CATKIN_PACKAGE_BIN_DESTINATION}": "lib/${PROJECT_NAME}",
            "${CATKIN_PACKAGE_INCLUDE_DESTINATION}": "include/${PROJECT_NAME}",
            "${CATKIN_PACKAGE_LIB_DESTINATION}": "lib",
            "${CATKIN_PACKAGE_SHARE_DESTINATION}": "share/${PROJECT_NAME}",
        }
        for var, replacement in renamedVariables.items():
            cmakeData = cmakeData.replace(var, replacement)

        if not dryrun:
            fid = open(CMAKELISTS, "w")
            fid.write("%s\n" % cmakeData.strip())
            fid.close()
        else:
            print(cmakeData)

        return True  # Success

    @classmethod
    def __findPackage(cls, pkgName):
        '''Create a command to find a specific package.

        * pkgName - the name of the package

        '''
        return cmkp.Command(
            "find_package",
            [cmkp.Arg(pkgName), cmkp.Arg("REQUIRED")])

    @classmethod
    def __addBlankLine(cls, cmake, index):
        index += 1
        cmake.insert(index, cmkp.BlankLine())
        return index


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description="Port a ROS (1) catkin package to a ROS 2 ament package")
    parser.add_argument(
        "--dryrun",
        action="store_true",
        help='do not make any changes to any files')
    parser.add_argument(
        "--only-source",
        action="store_true",
        help='only modify the source files')
    args = parser.parse_args()

    # Quick check to make sure this script was run from within
    # a ROS catkin package
    if not exists(PACKAGE_XML) or not exists(CMAKELISTS):
        print(
            "ERROR: you must run this script from within a ROS " +
            "catkin package directory!")
        exit(1)

    if args.dryrun:
        print("Performing a dryrun...")

    if not args.only_source:
        # Port the package XML
        if not PackageXmlPorter.port(args.dryrun):
            print("ERROR: Failed to port package XML")
            exit(2)

        # Port the CMakeLists file
        if not CmakeListsPorter.port(args.dryrun):
            print("ERROR: Failed to port CMakeLists.txt")
            exit(3)

    source_code_replacement()
