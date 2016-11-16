import collections


class DDR(collections.MutableMapping):
    """A dictionary which applies an arbitrary key-altering function
    before accessing the keys"""

    def __init__(self, *args, **kwargs):
        self.store = dict()
        # use the free update to set keys
        self.update(dict(*args, **kwargs))

    def __getitem__(self, key):
        return self.store[self.__keytransform__(key)]

    def __getattr__(self, attr):
        return self.__getitem__(attr)

    def __call__(self, attr, kwattr=None):
        return self.__getitem__(attr)

    def __setitem__(self, key, value):
        self.store[self.__keytransform__(key)] = value

    def __delitem__(self, key):
        del self.store[self.__keytransform__(key)]

    def __iter__(self):
        return iter(self.store)

    def __len__(self):
        return len(self.store)

    def __keytransform__(self, key):
        return key


class PyMock():
    def __init__(self):
        self.calldict = DDR()

    def _passMethod(self, *args, **kwargs):
        print args
        print kwargs
        pass

    def _setSomething(self, string, returns, call=False):
        """ This Method registers a new mock call
            As string there should be a dot seperated chain of modifier which can be accesed afterwards with the dot operator
            As returns shall be provided a return value which can be given or a function that should be called
            The call argument specifys wheather the last property in the chain is callable 
                * If returns is None then the _passMethod in this class is used
                * Otherwise the given argument is used as a function call
            
            Examples:
                
                a.)
                    originalMock = PyMock()
                    originalMock._setSomething("head_pan.position", 15)
            
                    assert originalMock.head_pan.position == 15
                
                b.)
                    originalMock = PyMock()
                    originalMock._setSomething("log", None, call=True)
            
                    originalMock.log()
                    originalMock.log("Blabla")
                    originalMock.log(suppe="abcd")
                    
                c.)
                    originalMock = PyMock()
                    
                    def assertLogIsRight(message):
                        assert message == "LogMessage"
                        
                    originalMock._setSomething("log", assertLogIsRight, call=True)
                    
                    # Assertion ok
                    originalMock.log("LogMessage")
                
                    # Assertion fails
                    originalMock.log("FalseMEssage")
                
                
        """
        splitted = string.split('.')
        scope = self.calldict
        for i in range(len(splitted) - 1):
            element = splitted[i]
            if element not in scope:
                scope[element] = DDR()
            scope = scope[element]
        if call is False:
            scope[splitted[-1]] = returns
        else:
            if returns is None:
                scope[splitted[-1]] = self._passMethod
            else:
                scope[splitted[-1]] = returns


    def __getattr__(self, param):
        return self.calldict[param]

    def __call__(self, attr, kwattr=None):
        print attr

    def __getitem__(self, key):
        return self.__getattr__(key)


if __name__ == "__main__":
    originalMock = PyMock()
    originalMock._setSomething("head_pan.position", 15)

    def mockMethodLog(message):
        assert message == "My Log"

    originalMock._setSomething("debug.log", mockMethodLog, call=True)

    print originalMock.head_pan.position

    originalMock.debug.log("sadS")

