import struct

file = open( "/dev/input/mice", "rb" );
totalX = 0
totalY = 0

def getMouseEvent():
  buf = file.read(3);
  # print buf & 0x10
  button = ord( buf[0] );
  bLeft = button & 0x1;
  bMiddle = ( button & 0x4 ) > 0;
  bRight = ( button & 0x2 ) > 0;
  x,y = struct.unpack( "bb", buf[1:] );
  #print ("L:%d, M: %d, R: %d, x: %d, y: %d\n" % (bLeft,bMiddle,bRight, x, y) );
  return x, y

while( 1 ):
  x, y = getMouseEvent();
  totalX += x
  totalY += y
  print "Total X: %.f,  Total Y: %.f" % (totalX, totalY)
file.close();