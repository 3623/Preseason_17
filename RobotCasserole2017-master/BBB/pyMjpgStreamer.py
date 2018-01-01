import sys,os,time
import numpy as np
import cv2    
import socket
import threading

if (sys.version_info >= (3, 0)):
    from http.server import BaseHTTPRequestHandler,HTTPServer
    from socketserver import ThreadingMixIn
    
else:
    from BaseHTTPServer import BaseHTTPRequestHandler,HTTPServer
    from SocketServer import ThreadingMixIn
    

    



class CamHandler(BaseHTTPRequestHandler):
    def __init__(self, something, somethingelse, anotherthing):
        try:
            self.address = bytearray(socket.gethostbyname(socket.gethostname()), 'utf8')
        except:
            #eeeh, we'll just guess
            self.address = b'10.17.36.9'
        BaseHTTPRequestHandler.__init__(self, something, somethingelse, anotherthing)

    def do_GET(self):
        global img, rc, port
        print ("Got mjpg stream client connection from " + str(self.client_address[0]) + " requesting resource " + self.path)
        if self.path.endswith('.mjpg'):
            self.send_response(200)
            self.send_header('Cache-Control','no-store, no-cache, must-revalidate, pre-check=0, post-check=0, max-age=0')
            self.send_header('Connection','close')
            self.send_header('Content-type','multipart/x-mixed-replace;boundary=--jpgboundary')
            self.send_header('Pragma','no-cache')
            self.end_headers()
            while(True):# infinite loop with no exit condition
                try:
                    if not rc:
                        time.sleep(0.1)
                        continue                         
                    else:
                        #imgRGB = cv2.cvtColor(img,cv2.COLOR_BGR2RGB)
                        r, buf = cv2.imencode(".jpg",img,(200,200))

                        self.wfile.write(b"--jpgboundary\r\n")
                        self.send_header('Content-type','image/jpeg')
                        self.send_header('Content-length',str(len(buf)))
                        self.end_headers()
                        self.wfile.write(bytearray(buf))
                        self.wfile.write(b'\r\n')
                        rc = False
                        time.sleep(0.2)
                except socket.error:
                    print("Client " + str(self.client_address[0]) + " disconnected.")
                    break

                    
            return

        elif self.path.endswith('.html') or self.path=="/":
            self.send_response(200)
            self.send_header('Content-type','text/html')
            self.end_headers()
            self.wfile.write(b'<html><head></head><body>')
            self.wfile.write(b'<img src="http://'+self.address+b':'+ bytearray(str(port), 'utf8') +b'/cam.mjpg"/>')
            self.wfile.write(b'</body></html>')
            return

        
class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    """Handle requests in a separate thread."""

    
class pyMjpgStreamer():
    
    
    def __init__(self,port_in=8080):
        global img, rc, port
        img = None
        rc = False
        port = port_in
    
    
        # Server setup
        self.server = ThreadedHTTPServer(('', port), CamHandler)
        self.server_host_thread = threading.Thread(target = self.server.serve_forever)
        self.server_host_thread.daemon=True
        self.server_host_thread.start()
        print("Started server on port "+ str(port))

        
    def setImg(self,img_in, rc_in):
        global img, rc
        img = img_in
        rc = rc_in
        
    def stop(self):
        print("Shutting down server")
        self.server.shutdown()
        print("Shutting down server main thread")
        #self.server_host_thread.join()


#Actual main code starts here
if __name__ == "__main__":
    """Do Nothing"""