import argparse

def parse(id_req=False, register_req=False):
    ret = {}

    parser = argparse.ArgumentParser()
    parser.add_argument("--p1", help="use old protocol version", action="store_true")
    parser.add_argument("--port",  dest='port', default='ACM0', help="Choose port, e.g. USB0 or ACM0. Standard is ACM0")
    parser.add_argument("--b", default="2000000", help="Choose baudrate. Standard is 2000000")

    if id_req:
        parser.add_argument("id")
    if register_req:
        parser.add_argument("register")

    args = parser.parse_args()

    if id_req:
        ret['id'] = int(args.id)
    if register_req:
        ret['reg'] = int(args.register)

    if args.p1:
        protocol = 1
    else:
        protocol = 2
    ret['protocol'] = protocol

    ret['baudrate'] = int(args.b)
    ret['device'] = ("/dev/" + args.port).encode('utf-8')

    return ret