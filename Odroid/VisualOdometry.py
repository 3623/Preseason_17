import sys
import cv2
import numpy as np
import socket

class VisualOdometry:

    def __init__(self,
                 channel=0,
                 height=240,
                 width=320,
                 scale=0.1,
                 reset=1,
                 min=3,
                 max=4,
                 eigen=0.05,
                 debug=False):

        self.CHANNEL = channel

        self.HEIGHT = height
        self.WIDTH = width
        self.SCALE_FACTOR = scale

        self.POINTS_RESET = reset
        self.MINIMUM_POINTS = min
        self.MAXIMUM_POINTS = max
        self.MIN_EIGENVAL = eigen

        self.test = debug

        if (self.test):
            None
            ## Use preset video
            # self.CHANNEL = 'drop_tile.mp4'

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

        ### TODO Init should handle logging

        self.setCamera

        self.resetAll() ## or self.run()

    def translateCoordinates(self, a):
        return ([a[0] - (0 * self.WIDTH / 2), (0 * self.HEIGHT / 2) - a[1], 1])


    def resetAll(self):
        self.count = 0
        # self.
        # self.run()


    def setCamera(self, cap):
        cap.set()


    def reshape(self, image):
        return cv2.resize(image, (self.WIDTH, self.HEIGHT))


    def calculateMatrix(self, new_points, old_points, debug):
        try:
            translated_matrix = np.matrix(np.apply_along_axis(self.translateCoordinates, 1, new_points))
            original_matrix = np.matrix(np.apply_along_axis(self.translateCoordinates, 1, old_points))
            inverse_original_matrix = np.linalg.pinv(original_matrix)
            transformation_matrix = np.asarray(inverse_original_matrix * translated_matrix)
            transposed_transformation_matrix = np.asarray(np.transpose(transformation_matrix))
            return True, transposed_transformation_matrix
        except:
            print "ERR %.f|MATR- Old Points- %.f,  New Points- %.f" % \
                  (self.count, len(new_points), len(old_points))
            ### TODO maybe replace count with timestamp??
            return False, None## TODO


    def calculateDisplacement(self, transformation_matrix):
        x_displacement = transformation_matrix[0][2]
        y_displacement = transformation_matrix[1][2]
        cosine = (transformation_matrix[0][0] + transformation_matrix[1][1])/2
        sine = (transformation_matrix[1][0] - transformation_matrix[0][1])/2
        rotational_displacement_radians = np.arctan2(sine, cosine)
        rotational_displacement_degrees = np.degrees(rotational_displacement_radians)
        return x_displacement, y_displacement, rotational_displacement_degrees


    def errorChecking(self, transformation_matrix, attempted_points, good_points):
        ### Error checking ## TODO need error handling
        opfl_eigenval_error = False
        if len(good_points) < len(attempted_points):
            print "ERR %.f|OPFL/EIGEN- " \
                  "Points: %.f,  Tracked: %.f,  Threshold: %g" \
                  % (self.count,
                     len(attempted_points),
                     len(good_points),
                     self.MIN_EIGENVAL)
            opfl_eigenval_error = True

        opfl_points_error = False
        if len(good_points) < self.MINIMUM_POINTS:
            print "ERR %.f|OPFL/POINTS- " \
                  "Points: %.f" \
                  % (self.count, len(good_points))
            opfl_points_error = True

        opfl_corner_error = False
        if np.abs(transformation_matrix[0][2]) > 0.05 \
                or np.abs(transformation_matrix[1][2]) > 0.05 \
                or np.abs(transformation_matrix[2][2] - 1) > 0.05:
            print "ERR %.f|OPFL/BR- " \
                  "Bottom row: %.5f, %.5f, %.5f.  Points: %.f" \
                  % (self.count, transformation_matrix[0][2].round(5),
                     transformation_matrix[1][2].round(5),
                     transformation_matrix[2][2].round(5),
                     len(good_points))
            opfl_corner_error = True

        opfl_cos_error = False
        if np.abs(transformation_matrix[1][1] - transformation_matrix[0][0]) > 0.075:
            print "ERR %.f|OPFL/COS- " \
                  "Cosine values: %.5f, %.5f.  Points: %.f" \
                  % (self.count,
                     transformation_matrix[0][0].round(5),
                     transformation_matrix[1][1].round(5),
                     len(good_points))
            opfl_cos_error = True

        opfl_sin_error = False
        if np.abs(transformation_matrix[0][1] - transformation_matrix[1][0]) > 0.075:
            print "ERR %.f|OPFL/SIN- " \
                  "Sine values: %.5f, %.5f.  Points: %.f" \
                  % (self.count,
                     transformation_matrix[0][1].round(5),
                     transformation_matrix[1][0].round(5),
                     len(good_points))
            opfl_sin_error = True

        opfl_matrix_error = opfl_corner_error or opfl_cos_error or opfl_points_error
        return opfl_matrix_error, 0


    def process(self, count, old_frame, new_frame, p0):
        ## calculate optical flow
        p1, st, err = cv2.calcOpticalFlowPyrLK(old_frame, new_frame, p0, None, **self.LK_PARAMS)

        ## Replace "good_new_points" with new points
        good_new_points = p1[st == 1]
        good_old_points = p0[st == 1]

        ### TODO Need a way to maintain points if tracking fails

        matrix_success, t = self.calculateMatrix(good_new_points, good_old_points, self.test)
        if (matrix_success):
            abort, score = self.errorChecking(t, p1, good_new_points)

            x, y, r = self.calculateDisplacement(t)

            score = 0

            if (self.test):
                print t.round(4)
                print x.round(4), y.round(4), r.round(4)

                ## draw the tracks
                if abort:
                    for i, (new, old) in enumerate(zip(good_new_points, good_old_points)):
                        a, b = new.ravel()
                        c, d = old.ravel()
                        self.mask = cv2.line(self.mask, (a, b), (c, d), (0, 0, 255), 2)
                        self.resize = cv2.circle(self.resize, (a, b), 5, self.color[i].tolist(), -1)
                else:
                    for i, (new, old) in enumerate(zip(good_new_points, good_old_points)):
                        a, b = new.ravel()
                        c, d = old.ravel()
                        self.mask = cv2.line(self.mask, (a, b), (c, d), (0, 255, 0), 1)
                        self.resize = cv2.circle(self.resize, (a, b), 5, (255, 0, 255), -1)
                self.img = cv2.add(self.resize, self.mask)

            ## Now update the previous frame and previous points
            return p1, score, x, y, r
        else:
            ### TODO figure out what to do if matrix fails yaknow
            ## Now update the previous frame and previous points
            return p1, 0, 0, 0, 0


    def run(self):
        cap = cv2.VideoCapture(self.CHANNEL)
        ret, frame = cap.read()
        # cap.release() ## Releases no next frame will have to be fresh ## Need to try stuff out

        old_frame = cv2.cvtColor(self.reshape(frame), cv2.COLOR_BGR2GRAY)
        ## Take first frame and find corners in it
        p0 = cv2.goodFeaturesToTrack(old_frame, mask=None, **self.FEATURE_PARAMS)

        if (self.test):
            self.resize = self.reshape(frame) ## Create a mask image for drawing purposes
            self.mask = np.zeros_like(self.resize)
            self.color = np.random.randint(0, 120, (100, 3)) ## Create some random colors

        while True:
            ## Counter- resets at a certain count
            self.count += 1
            if self.count == 0:
                None
            else:
                None

            # cap = cv2.VideoCapture(self.CHANNEL)
            ret, frame = cap.read()
            # cap.release()  ## Releases no next frame will have to be fresh ## Need to try stuff out
            if frame is None:
                break
            else:
                new_frame =  cv2.cvtColor(self.reshape(frame), cv2.COLOR_BGR2GRAY)
            if (self.test):
                self.resize = self.reshape(frame)
            p1, score, x, y, r = self.process(self.count, old_frame, new_frame, p0)

            if self.count % self.POINTS_RESET == 0:
                p0 = cv2.goodFeaturesToTrack(new_frame, mask=None, **self.FEATURE_PARAMS)
            else:
                p0 = p1

            if True:
                old_frame = new_frame
            else:
                old_frame = None

            if (self.test):
                cv2.imshow("Optical Flow", self.img)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        ## End program output
        print "======================================================="
        print "Frame Count- %.f,  Point Reset- %.f,  Eigen Val Threshold- %g" % \
              (self.count, self.POINTS_RESET, self.MIN_EIGENVAL)
        print "Image Height- %.f, Image Width- %.f, Pixels- %.f" % \
              (self.HEIGHT, self.WIDTH, self.HEIGHT * self.WIDTH)

        if (self.test):
            cv2.imshow("Optical Flow", img)
            cv2.waitKey(0)
            cv2.destroyAllWindows()


if __name__ == '__main__':
    # visual_odometry = VisualOdometry(channel=1,debug=False)
    # print "Visual_Odometry Running"
    # visual_odometry.run()

    cap = cv2.VideoCapture(1)
    count = 0

    while True:
        count += 1
        # cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        # print cap.get(cv2.CAP_PROP_FPS)
        ret, frame = cap.read()
        # cap.release()
        # if not(count < 0):
        #     cv2.imshow(" ", frame)
        #     count += 1
        #     if cv2.waitKey(1) & 0xFF == ord('q'):
        #         break
        #     if cv2.waitKey(0) & 0xFF == ord('w'):
        #         continue
        # else:
        #     continue
        cv2.imshow("fuck", frame)


        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    print count
    cap.release()

