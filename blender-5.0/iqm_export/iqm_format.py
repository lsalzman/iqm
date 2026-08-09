import struct

IQM_POSITION = 0
IQM_TEXCOORD = 1
IQM_NORMAL = 2
IQM_TANGENT = 3
IQM_BLENDINDEXES = 4
IQM_BLENDWEIGHTS = 5
IQM_COLOR = 6
IQM_CUSTOM = 0x10

IQM_BYTE = 0
IQM_UBYTE = 1
IQM_SHORT = 2
IQM_USHORT = 3
IQM_INT = 4
IQM_UINT = 5
IQM_HALF = 6
IQM_FLOAT = 7
IQM_DOUBLE = 8

IQM_LOOP = 1

IQM_HEADER = struct.Struct("<16s27I")
IQM_MESH = struct.Struct("<6I")
IQM_TRIANGLE = struct.Struct("<3I")
IQM_JOINT = struct.Struct("<Ii10f")
IQM_POSE = struct.Struct("<iI20f")
IQM_ANIMATION = struct.Struct("<3IfI")
IQM_VERTEXARRAY = struct.Struct("<5I")
IQM_BOUNDS = struct.Struct("<8f")
