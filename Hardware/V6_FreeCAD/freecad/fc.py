"""Minimal client for the FreeCAD MCP addon's XML-RPC server (FreeCAD GUI on localhost:9875)."""
import xmlrpc.client, sys, time, base64

def proxy():
    return xmlrpc.client.ServerProxy('http://localhost:9875', allow_none=True)

def wait(timeout=120):
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            return proxy().ping()
        except Exception:
            time.sleep(2)
    raise TimeoutError('FreeCAD RPC server not reachable')

def run(code, timeout=None):
    """Execute code on FreeCAD's GUI thread; print its stdout; raise on error."""
    r = proxy().execute_code(code) if timeout is None else proxy().execute_code(code, timeout)
    if not r.get('success'):
        raise RuntimeError(r.get('error') or r)
    out = r.get('message', '').split('Output: ', 1)[-1]
    return out

def screenshot(path, view='Isometric', width=1600, height=1000, focus=None):
    b64 = proxy().get_active_screenshot(view, width, height, focus)
    if b64:
        open(path, 'wb').write(base64.b64decode(b64))
    return bool(b64)

if __name__ == '__main__':
    code = sys.stdin.read() if len(sys.argv) < 2 else open(sys.argv[1]).read()
    print(run(code, timeout=float(sys.argv[2]) if len(sys.argv) > 2 else None))
