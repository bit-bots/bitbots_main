import collections
import importlib
import os
import re
from typing import List

from bitbots_dsd.abstract_decision_element import AbstractDecisionElement


def defaultdict_listfactory():
    return collections.defaultdict(list)


def register_element(path: str) -> dict:
    """
    Extract all the classes from the files in the given path and return a dictionary containing them

    :param path: The path containing the files that should be registered
    :return: A dictionary with classnames as keys and classes as values
    """
    elements = {}
    files = [f for f in os.listdir(path) if f.endswith('.py')]
    for file in files:
        with open(os.path.join(path, file), "r") as dp:
            for line in dp:
                try:
                    m = re.search(r"(?<=class\s)[a-zA-Z0-9]*", line)
                    if m:
                        classname = m.group()
                        # relative_filename is the name relative to the src directory (from where it will be imported)
                        relative_filename = os.path.join(os.path.relpath(os.path.relpath(path), os.path.dirname(os.path.dirname(path))), file)
                        module = importlib.import_module(relative_filename.replace("/", ".").replace("\\", ".").replace(".py", ""))
                        elements[classname] = getattr(module, classname)
                except Exception as e:
                    print(e)
    return elements


class DSD:
    """
    One decision is defined as the root decision, the starting point.
    Each decision element, which is pushed on the stack, is immediately executed until no further element is pushed. 
    Following, each iteration, for each element, is checked if it requires to be reevaluated and finally the
     top element of the stack will be executed, usually an action.
    If the outcome of a reevaluated element changes, the entire stack on top of this element will be dropped and the
     stack newly constructed.
    As soon as the action is complete, the element will be popped off the stack and the module underneath will be
     executed in the next iteration.
    If this is a decision, it again pushes a further decision or an action and the new top element will be executed.

    By this structure, it is always visible which action the robot tries to perform and which decisions were made.

    If a new element is pushed on top of the stack, it is directly executed.
    In most cases, the pushing element is completing its execution with the push of another element. 
    Any following code will be executed as soon as the stack is not further expanded.
    """

    stack = []
    start_element = None
    start_element_data = None
    stack_excec_index = -1
    stack_reevaluate = False
    do_not_reevaluate = False
    old_representation = ""

    def __init__(self, blackboard):
        self.blackboard = blackboard

        self.tree = {}
        self.stack = []
        self.path: str = ""

        self.actions = {}
        self.decisions = {}

    def register_actions(self, modulepath, path):
        self.actions = register_element(modulepath, path)

    def register_decisions(self, modulepath, path):
        self.decisions = register_element(modulepath, path)

    def load_behavior(self, path):

        tree_build = collections.defaultdict(defaultdict_listfactory)
        curr_path = ""

        # Parsing the behavior file

        sub = ""
        next_is_start = False
        lastline = ""
        with open(path, "r") as bfile:
            comment = False
            lnr = 0
            for line in bfile:
                lnr += 1
                try:
                    oline = line.rstrip()
                    if not oline:
                        continue
                    if "**//" in oline:
                        comment = False
                        continue
                    if "//**" in oline:
                        comment = True
                        continue
                    if oline.strip()[:2] == "//":
                        continue
                    if not comment:

                        indent = len(oline) - len(oline.lstrip())
                        if indent % 4 != 0:
                            raise AssertionError(f"Ident in line {lnr} is not a multiple of 4")
                        last_indent = len(lastline) - len(lastline.lstrip())

                        line = oline.lstrip()

                        if indent == 0 and line[0] == "#":
                            # create subtree
                            sub = line[1:]
                            curr_path = ""

                        if indent == 0 and line[0:3] == "-->":
                            next_is_start = True
                            sub = "root"
                            lastline = oline
                            continue

                        if next_is_start:
                            next_is_start = False
                            curr_path = ""  # line[1:]
                            self.path = line
                            self.set_start_element(self.decisions[line.strip('$')])

                        # Go one layer up
                        if indent < last_indent:
                            curr_path = ".".join(curr_path.split(".")
                                                 [:len(curr_path.split(".")) - ((last_indent - indent) // 4)])
                        # Handle sub-behaviors
                        snd = re.split(r"\s*-?->\s*", line)
                        if len(snd) >= 2 and snd[1][0] == "#":
                            if snd[1][1:] not in tree_build:
                                raise AssertionError(f"#{snd[1][1:]} not defined")
                            for k, v in tree_build[snd[1][1:]].items():
                                tree_build[sub][curr_path + "." + k].extend(v)
                                tree_build[sub][curr_path + "." + k] = list(set(tree_build[sub][curr_path + "." + k]))

                            tree_build[sub][curr_path].append(
                                (snd[0], list(tree_build[snd[1][1:]].keys())[0].split(".")[0]))

                        elif "->" in line or "@" in line:

                            # Add path with None for Actions
                            if "@" in line:
                                if curr_path:
                                    ecp = curr_path + "."
                                else:
                                    ecp = ""
                                tree_build[sub][ecp + re.split(r"-?->|\+", line.strip())[1].strip()
                                                if "->" in line else line.strip()] = [None]

                            # Add (or extend) return option for current path wich is specified in this line
                            if curr_path:
                                tree_build[sub][curr_path].append(
                                    tuple((x.strip() for x in re.split(r"-?->|\+", line))))

                        # Go one layer deeper fox next line
                        if (indent >= last_indent and "@" not in line and "#" not in line) or (
                                indent == 0 and line[0] != "#"):
                            curr_path += "." if curr_path else ""
                            curr_path += re.sub(r"[#]", "",
                                                re.split(r"-?->|\+", line.strip())[1].strip()
                                                if "->" in line else line.strip())
                        lastline = oline
                except Exception as e:
                    print("Error parsing line ", lnr, e)
                    # raise e
        self.tree = tree_build["root"]

    def _init_element(self, element, init_data=None):
        """
            Initialises the element.
        """
        mod = element(self.blackboard, init_data)
        mod.setup_internals(self, init_data)
        return mod

    def set_start_element(self, start_element, init_data=None):
        """
            This method dfines the start element on the stack, which stays always on the bottom of the stack.

            This method should be called in __init__.
        """
        self.stack = []
        self.start_element = start_element
        self.start_element_data = init_data
        self.stack.append(self._init_element(start_element, init_data))

    def interrupt(self):
        """
            An interrupt is an event which clears the complete stack to reset the behavior.
            In the special case of RoboCup, we use it when the game-state changes, but it can also be used for
            example if the robot is kidnapped or paused.
            In the following iteration, the stack will be newly created starting at the root element.
        """
        if self.stack_reevaluate:
            # we were currently checking preconditions 
            # we stop this, so that update() knows that it has to stop
            self.stack_reevaluate = False
        self.stack = [self._init_element(self.start_element, self.start_element_data)]

    def update(self, reevaluate=True):
        """
        Calls the element which is currently on top of the stack.
        Before doing this, all preconditions are checked (all decision elements where reevaluate is true).
        
        :param: reevaluate: Can be set to False to inhibit the reevaluation
        :type reevaluate: bool
        """
        if not reevaluate:
            self.publish_debug_msg()

        if reevaluate and not self.do_not_reevaluate:
            self.stack_excec_index = 0
            self.stack_reevaluate = True
            for element in self.stack[:-1]:
                # check all elements, exept the top one not the actions
                if isinstance(element, AbstractDecisionElement) and element.get_reevaluate():
                    ret = element.perform(self.blackboard, True)
                    self.push(ret)

                    if not self.stack_reevaluate:
                        # We had some external interrupt, we stop here
                        return
                self.stack_excec_index += 1
            self.stack_reevaluate = False
        if reevaluate:
            # reset flag
            self.do_not_reevaluate = False
        # run the top module
        ret = self.stack[-1].perform(self.blackboard)
        self.push(ret)

    def push(self, element_name, init_data=None, perform=True):
        """
        Put a new element on the stack and start it directly. 

        .. warning::
            After using push, you should not have further code, since this
            leads to difficult to debug behavior. Try to use::

                return self.push(xxxElement, data)            

        :param element: The element that should be put on top of the stack. Do not initilize!            
        :type element: Element
        :param init_data: This data will be given to the new module during its init, optional
        :param perform: Optional parameter to disable direct call of the perform method during this cycle
        """

        choices: List[tuple] = self.tree[self.path]
        next_t = tuple(x[1] for x in choices if x[0] == element_name)
        next_name: str = next_t[0]
        if len(next_t) == 2:
            init_data: dict = next_t[1]

        if "@" in next_name:
            element = self.actions[re.sub(r"[$@~!]", "", next_name)]
        elif "$" in next_name:
            element = self.decisions[re.sub(r"[$@~!]", "", next_name)]
        else:
            raise AssertionError()

        if self.stack_reevaluate:
            # we are currently checking pre conditions
            # check if we made the same decision (push) as last time
            if type(self.stack[self.stack_excec_index + 1]) == element and \
                    self.stack[self.stack_excec_index + 1].get_init_data() == init_data:
                # decision was the same, reevaluation passed, precondition did not change
                return
            else:
                # result changed. we clear all stack above us and push the new element on top
                self.stack = self.stack[0:self.stack_excec_index + 1]
                self.path = ".".join(self.path.split(".")[0:self.stack_excec_index + 1])
                # reevaluate is finished
                self.stack_reevaluate = False
        self.stack.append(self._init_element(element, init_data))
        self.path += "." + next_name
        # we call the new element without another reevaluate
        if perform:
            self.update(False)

    def pop(self):
        """
        Removes the element from the stack. The previous element will not be called again.
        """
        if len(self.stack) > 1:
            if self.stack_reevaluate:
                # we are currently reevaluating. we shorten the stack here                
                if self.stack_excec_index > 0:
                    # only shorten stack if it still has one element
                    self.stack = self.stack[0:self.stack_excec_index]
                # stop reevaluating
                self.stack_reevaluate = False
            else:
                self.stack.pop()
        else:
            # rospy.logwarn("Can't pop in %s: stack is empty, Module: %s" % (repr(self), repr(self.stack[0])))
            pass

    def set_do_not_reevaluate(self):
        """No reevaluation on next iteration"""
        self.do_not_reevaluate = True

    def get_stack(self):
        """
        Returns the current stack
        """
        return self.stack

    def publish_debug_msg(self):
        """
        Helper method to publish debug data
        """
        pass
        # if self.debug_active:
        #    msg_data = ",".join([repr(x) for x in self.stack])
        #    msg = String(data=msg_data)
        # self.debug_pub.publish(msg)
