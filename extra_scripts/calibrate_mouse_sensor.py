import struct
import time

file = open( "/dev/input/mice", "rb" )

x_dist = 0

def getMouseEvent():
  global x_dist
  buf = file.read(3)
  # print(buf)
  buttons = buf[0]
  bLeft = buttons & 0x1
  bMiddle = ( buttons & 0x4 ) > 0
  bRight = ( buttons & 0x2 ) > 0
  x,y = struct.unpack( "bb", buf[1:] )
  # print ("L:%d, M: %d, R: %d, x: %d, y: %d\n" % (bLeft,bMiddle,bRight, x, y) )
  x_dist += x

def time_check():
  global start_time
  return time.time() - start_time >= 20


print('Move mouse 10cm in the next 20 seconds')
start_time = time.time()

while not time_check():
  getMouseEvent()
  time.sleep(0.01)
  print('time passed')

print(f'x total: {x_dist}, Dots per meter: {x_dist*10}')

file.close()