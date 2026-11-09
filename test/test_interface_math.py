"""Exercise diagnostic parsing/math without importing ROS on the build host."""
import ast
import math
from pathlib import Path
import struct
from types import SimpleNamespace as NS
import unittest

source = Path(__file__).resolve().parents[1] / 'scripts/check_go2_interfaces.py'
tree = ast.parse(source.read_text())
functions = [node for node in tree.body if isinstance(node, ast.FunctionDef)
             and node.name in ('xyz', 'stamp', 'world_point')]
scope = {'struct': struct, 'math': math}
exec(compile(ast.Module(body=functions, type_ignores=[]), str(source), 'exec'), scope)


class DiagnosticMathTest(unittest.TestCase):
    def test_padded_rows_and_endianness(self):
        for endian in ('<', '>'):
            data = bytearray(48)
            struct.pack_into(endian+'fff', data, 0, 1,2,3)
            struct.pack_into(endian+'fff', data, 24, 4,5,6)
            msg = NS(fields=[NS(name=n,offset=4*i,datatype=7)
                             for i,n in enumerate(('x','y','z'))],
                     is_bigendian=endian=='>', height=2, width=1,
                     row_step=24, point_step=16, data=data)
            self.assertEqual(list(scope['xyz'](msg)), [(1,2,3),(4,5,6)])

    def test_rotation_translation_and_quaternion_normalization(self):
        pose = NS(orientation=NS(x=0,y=0,z=2,w=2), position=NS(x=10,y=20,z=30))
        p = scope['world_point']((2,3,4), pose)
        for a,b in zip(p, (7,22,34)):
            self.assertAlmostEqual(a,b)
        pose.orientation.z = pose.orientation.w = 0
        with self.assertRaises(ValueError):
            scope['world_point']((2,3,4),pose)

    def test_integer_sensor_stamp(self):
        msg = NS(header=NS(stamp=NS(sec=1700000000,nanosec=123)))
        self.assertEqual(scope['stamp'](msg), 1700000000000000123)


if __name__ == '__main__':
    unittest.main()
