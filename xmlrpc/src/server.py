from xmlrpc.server import SimpleXMLRPCServer

VSK_DIR = '/home/plusk01/Documents/vicon'

def test1(*params):
    print(params)
    return params

def test2(*params):
    print(params)
    arr = ['polozka0', 'polozka1', 'polozka2', 'polozka3', 'polozka4']
    return {'status': 200, 'statusMessage': 'OK', 'struct': { 'int': params[0], 'string': params[1]}, 'array': arr}

if __name__ == '__main__':
    server = SimpleXMLRPCServer(('0.0.0.0', 2424), allow_none=True)
    server.register_function(test1)
    server.register_function(test2)
    server.serve_forever()