#-*- coding:utf-8 -*-

cdef class Joint:
    ''' Diese Klasse modelliert ein Gelenk des Roboters.

        Über das zugeordnete mx-Attribute kann man alle Werte des Motors setzen
        und auslesen.
    '''
    cdef public object cid
    cdef public bytes name
    cdef public list tags
    cdef public int opposing
    cdef public int inverted
    cdef public int limit_default
    cdef public int limit_min
    cdef public int limit_max

    def __init__(self, cid, bytes name):
        self.cid = cid
        self.name = name
        self.tags = []
        self.limit_default = 0
        self.limit_min = -180
        self.limit_max = 180

cdef class JointGroup:
    ''' Eine JointGroup kann mehrere Gelenke halten, die nach Name oder
        Tag abgefragt werden können.
    '''
    cdef dict joints
    cdef dict cidmap

    def __init__(self, list joints):
        cdef Joint j
        self.joints = dict((j.name, j) for j in joints)
        self.cidmap = dict((j.cid, j) for j in joints)

    def get_names(self):
        ''' Gibt die Menge aller Namen der Gelenke in dieser JointGroup zurück.
            Das zurückgegebene Objekt ist nicht veränderlich
        '''
        return self.joints.keys()

    def get_joints(self, bytes tag):
        ''' Gibt eine JointGroup gefiltert nach 'tag' zurück.
            Ist tag gleich None, wird die Gruppe unverändert zurück gegeben,
            sonst wird nach Gelenknamen und -tags gefiltert.
        '''
        if tag is None:
            return self

        if self.has(tag):
            # Gelenk mit dem Namen existiert,
            # also geben wir es direkt zurück
            return JointGroup([self.joints[tag]])

        # Gucken welche Gelenke einen passenden Tag haben
        joints = [j for j in self.joints.itervalues() if tag in j.tags]
        return JointGroup(joints)

    cpdef Joint get_joint_by_cid(self, int cid):
        return self.cidmap[cid]

    cpdef Joint get(self, bytes name):
        ''' Gibt das Gelenk mit dem Namen 'name' zurück.
            Es wird eine Exception vom Typ KeyError geworfen, wenn das
            gewünschte Gelenk nicht existiert
        '''
        return self.joints[name]

    def __getitem__(self,name):
        '''Ermöglicht zugriff auf Gelenke nach Namen wie in einem Dict
        '''
        return self.joints[name]

    cpdef has(self, bytes name):
        ''' Prüft ob das Gelenke 'name' vorhanden ist '''
        return name in self.joints

    def __iter__(self):
        ''' Gibt einen Iterator über alle Gelenke zurück. '''
        return self.joints.itervalues()

    def __len__(self):
        return len(self.joints)

    def __repr__(self):
        return "<JointGroup with %d joints>" % len(self)


import yaml

def load_joints_from_yaml(name):
    ''' Läd alle Gelenke aus einer .json-Datei '''
    with open(name) as fp:
        infolist = yaml.load(fp)

    cdef Joint joint
    cdef dict info
    cdef list joints = []
    for info in infolist:
        joint = create_joint_from_info(info)
        joints.append(joint)

    return JointGroup(joints)

cdef Joint create_joint_from_info(dict info):
    ''' Erzeugt ein Gelenk aus einer Beschreibung '''
    cid = int(info["id"])
    name = info["name"].encode("utf8")
    limits = info.get("limits", {})
    limit_default = limits['default']
    limit_min = limits['min']
    limit_max = limits['max']
    tags = info.get("tags", [])
    opposing = info["opposing"]
    inverted = info["inverted"]

    cdef Joint joint = Joint(cid, name)
    joint.tags = tags
    joint.opposing = opposing
    joint.inverted = inverted
    joint.limit_default = limit_default
    joint.limit_min = limit_min
    joint.limit_max = limit_max
    return joint

