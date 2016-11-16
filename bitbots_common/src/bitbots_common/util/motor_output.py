# -*- coding:utf8 -*-
from bitbots_common.util.ascii import motorIdsFormated as img
try:
    import urwid
    URWID = True
except ImportError:
    URWID = False


def raise_exit(loop, data):
    """ Callback for the urwid loop that causes an abort
    """
    raise urwid.ExitMainLoop()


def exit_on_key(key):
    raise urwid.ExitMainLoop()


def show_errors(idlist, duration=5):
    """ Receives a list of Motor-Id's to be displayed as faulty
    :value duration: Duration of the display in seconds
    :returns: Nothing
    """
    if not URWID:
        print("Urwid ist nicht Installiert!")
        return
    palette = []
    for element in idlist:
        if element in range(0, 20):
            palette.append(('mid%s' % element, 'white', 'dark red'))
        else:
            print("Kann nix mit der Motornummer '%s' anfangen!" % element)
    txt = urwid.Text(img)
    fill = urwid.Filler(txt, 'top')
    loop = urwid.MainLoop(fill, palette, unhandled_input=exit_on_key)
    if duration is not None:
        loop.set_alarm_in(duration, raise_exit)
    loop.run()
