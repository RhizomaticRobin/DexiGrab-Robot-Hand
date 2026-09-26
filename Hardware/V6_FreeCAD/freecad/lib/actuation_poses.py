"""SUPERSEDED early draft (inclined V5 seats).  The motor poses are now defined by lib/mod_actuation.py and
published with mod_actuation.write_motors_json() -> regions/actuation_motors.json.  Kept only so an old import does
not fail; running it rewrites the JSON from mod_actuation (needs freecadcmd)."""

if __name__ == '__main__':
    import sys
    sys.path.insert(0, '/path/to/DexiGrab-Robot-Hand/Hardware/V6_FreeCAD/freecad/lib')
    import mod_actuation
    d = mod_actuation.write_motors_json()
    print('wrote actuation_motors.json', len(d['motors']))
