import struct
file = open( "/dev/input/mice", "rb" )

def getMouseEvent():
  buf = file.read(3)
  print(buf)
  buttons = ord( int.from_bytes(buf[0], 'big') )
  bLeft = buttons & 0x1
  bMiddle = ( buttons & 0x4 ) > 0
  bRight = ( buttons & 0x2 ) > 0
  x,y = struct.unpack( "bb", buf[1:] )
  print ("L:%d, M: %d, R: %d, x: %d, y: %d\n" % (bLeft,bMiddle,bRight, x, y) )

while True:
  getMouseEvent()
file.close()