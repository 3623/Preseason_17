import sys
import cv2
import numpy as np
import socket

class VisualOdometry:

    def __init__(self,channel=0,height=120,width=160,scale=0.1,reset=1,min=3,max=4,eigen=0.05,debug=False):
        self.CHANNEL = channel

        self.HEIGHT = height
        self.WIDTH = width
        self.SCALE_FACTOR = scale

        self.POINTS_RESET = reset
        self.MINIMUM_POINTS = min
        self.MAXIMUM_POINTS = max
        self.MIN_EIGENVAL = eigen

        self.test = debug

        if self.test:
            ## Create some random colors
            self.color = np.random.randint(0, 120, (100, 3))
            ## Use preset video
            self.CHANNEL = 'drop_tile.mp4'

        ## Minimum distance between points
        self.minDistanceP = (self.WIDTH + self.HEIGHT) / 2 / 40
        ## params for ShiTomasi corner detection
        self.FEATURE_PARAMS = dict(maxCorners=self.MAXIMUM_POINTS,
                              qualityLevel=0.2,
                              minDistance =self.minDistanceP,
                              blockSize=7)
        ## Parameters for lucas kanade optical flow
        self.LK_PARAMS = dict(winSize=(15, 15),
                         maxLevel=2,
                         criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03),
                         minEigThreshold=self.MIN_EIGENVAL)
        ### TODO mineigenvals??
        self.resetAll()
        self.CHANNEL = 0

    def transCoord(a):
        return ([a[0] - (0 * self.width / 2), (0 * self.height / 2) - a[1], 1])

    def reshape(self, image):
        return cv2.resize(image, (self.WIDTH, self.HEIGHT))

    def resetAll(self):
        self.count = 0
        self.totalR = 0
        self.totalX = 0
        self.totalY = 0

    def run(self):
        cap = cv2.VideoCapture(self.CHANNEL)
        ret, frame = cap.read()
        cap.release() ## Releases no next frame will have to be fresh
        cv2.imshow("Fuck", frame)

        old_frame = cv2.cvtColor(self.reshape(frame), cv2.COLOR_BGR2GRAY)
        ## Take first frame and find corners in it
        p0 = cv2.goodFeaturesToTrack(old_frame, mask=None, **self.FEATURE_PARAMS)
        if (self.test):
            ## Create a mask image for drawing purposes
             mask = np.zeros_like(old_frame)

        while True:
            ## Counter- resets at a certain count
            self.count += 1
            if self.count == 60:
                None
            else:
                None

            cap = cv2.VideoCapture(self.CHANNEL)
            ret, frame = cap.read()
            cap.release()  ## Releases no next frame will have to be fresh
            if frame is None:
                break
            else:
                new_frame =  cv2.cvtColor(self.reshape(frame), cv2.COLOR_BGR2GRAY)
            print p0
            ## calculate optical flow
            p1, st, err = cv2.calcOpticalFlowPyrLK(old_frame, new_frame, p0, None, **self.LK_PARAMS)
            ## Replace "good_new" with new points


            good_new = p1[st == 1]
            good_old = p0[st == 1]
            ### TODO Need a way to maintain points if tracking fails

            try:
                x_prime = np.matrix(np.apply_along_axis(transCoord, 1, good_new))
                x = np.matrix(np.apply_along_axis(transCoord, 1, good_old))
                x_inv = np.linalg.pinv(x)
                t = np.asarray(x_inv * x_prime)
                if (self.test):  ## Testing outputs
                    print x_prime
                    print x
                    transpose = np.asarray(np.transpose(t))
                    print "========================="
                    print transpose.round(4)
                    print x_inv.shape, x_prime.shape
                    print t.round(4)
                    print "========================="
                    check = x * t
                    print check.round(3)
            except:
                print "ERR %.f|MATR- Old Points- %.f,  New Points- %.f" % (self.count, len(good_new), len(good_old))
                continue

            ### Error checking ## TODO need error handling
            opfl_eigenval_error = False
            if len(good_new) < len(p1):
                print "ERR %.f|OPFL/EIGEN- " \
                      "Points: %.f,  Tracked: %.f,  Threshold: %g" \
                      % (self.count, len(p1), len(good_new), self.MIN_EIGENVAL)
                opfl_eigenval_error = True

            opfl_points_error = False
            if len(good_new) < self.MINIMUM_POINTS:
                print "ERR %.f|OPFL/POINTS- " \
                      "Points: %.f" \
                      % (self.count, len(good_new))
                opfl_points_error = True

            opfl_corner_error = False
            if np.abs(t[0][2]) > 0.05 or np.abs(t[1][2]) > 0.05 or np.abs(t[2][2] - 1) > 0.05:
                print "ERR %.f|OPFL/BR- " \
                      "Bottom row: %.5f, %.5f, %.5f.  Points: %.f" \
                      % (self.count, t[0][2].round(5), t[1][2].round(5), t[2][2].round(5), len(good_new))
                opfl_corner_error = True

            opfl_cos_error = False
            if np.abs(t[1][1] - t[0][0]) > 0.075:
                print "ERR %.f|OPFL/COS- " \
                      "Cosine values: %.5f, %.5f.  Points: %.f" \
                      % (self.count, t[0][0].round(5), t[1][1].round(5), len(good_new))
                opfl_cos_error = True

            opfl_sin_error = False
            if np.abs(t[0][1] - t[1][0]) > 0.075:
                print "ERR %.f|OPFL/SIN- " \
                      "Sine values: %.5f, %.5f.  Points: %.f" \
                      % (self.count, t[0][1].round(5), t[1][0].round(5), len(good_new))
                opfl_sin_error = True

            opfl_matrix_error = opfl_corner_error or opfl_cos_error or opfl_points_error

            if (self.test):
                ## draw the tracks
                if opfl_matrix_error:
                    for i, (new, old) in enumerate(zip(good_new, good_old)):
                        a, b = new.ravel()
                        c, d = old.ravel()
                        mask = cv2.line(mask, (a, b), (c, d), (0, 0, 255), 2)
                        resize = cv2.circle(resize, (a, b), 5, color[i].tolist(), -1)
                else:
                    for i, (new, old) in enumerate(zip(good_new, good_old)):
                        a, b = new.ravel()
                        c, d = old.ravel()
                        mask = cv2.line(mask, (a, b), (c, d), (0, 255, 0), 1)
                        resize = cv2.circle(resize, (a, b), 5, (255, 0, 255), -1)
                img = cv2.add(new_frame, mask)

            x = t[2][0]
            y = t[2][1]
            cos = t[0][0] + t[1][1]
            sin = t[0][1] - t[1][0]
            r = np.arctan2(sin, cos)
            degrees = np.degrees(r)
            totalX += x
            totalY += y
            totalR += degrees
            if (self.test):
                print x.round(4), y.round(4), cos.round(4), sin.round(4), r.round(4), degrees.round(4)
                print totalX.round(4), totalY.round(4), totalR.round(4)

            if count % RESET == 0:
                p0 = cv2.goodFeaturesToTrack(frame_gray, mask=None, **FEATURE_PARAMS_1_1_1)
            else:
                p0 = p1

            ## Now update the previous frame and previous points
            old_frame = new_frame.copy()

            if (self.test):
                cv2.imshow("Optical Flow", img)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break



        ## End program output
        print "======================================================="
        print "Frame Count- %.f,  Point Reset- %.f,  Eigen Val Threshold- %g" % (self.count, RESET, MIN_EIGENVAL)
        print "Displacement X: %.2f,  Displacement Y: %.2f,  Displacement R: %.2f" % \
              (round(totalX, 2), round(totalY, 2), round(totalR, 2))
        print "Image Height- %.f, Image Width- %.f, Pixels- %.f" % (height, width, height * width)

        if (self.test):
            cv2.imshow("Optical Flow", img)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

if __name__ == '__main__':
    # while True:
    #     cap = cv2.VideoCapture(0)
    #     ret, frame = cap.read()
    #     # cap.release()
    #     cv2.imshow(" ", frame)
    #     if cv2.waitKey(2000) != -1:
    #         break

    visual_odometry = VisualOdometry(debug=True)
    print "Visual_Odometry Running"
    visual_odometry.run()

