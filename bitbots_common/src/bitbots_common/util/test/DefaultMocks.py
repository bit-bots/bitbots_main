from bitbots.util.test.PyMock import PyMock

from bitbots.ipc import STATE_CONTROLABLE


def IPCMock():
    state = STATE_CONTROLABLE

    def set_state(s):
        state = s

    def get_state():
        return state

    ipcMock = PyMock()
    ipcMock._setSomething("set_state", set_state, call=True)
    ipcMock._setSomething("get_state", get_state, call=True)

    return ipcMock


def GoalInfoMock():
    x = 0
    y = 0
    _ = 0

    ipcMock = PyMock()
    ipcMock._setSomething("x", 0)
    ipcMock._setSomething("y", 0)
    ipcMock._setSomething("_", 0)

    return ipcMock
