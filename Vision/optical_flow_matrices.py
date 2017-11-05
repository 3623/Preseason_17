import sys
import cv2
import numpy as np
import math



if __name__ == '__main__':

    cap = cv2.VideoCapture('drop_tile.mp4')
    ## Create some random colors
    color = np.random.randint(0, 120, (100, 3))
    ## Take first frame and find corners in it
    ret, old_frame = cap.read()
    resize = cv2.resize(old_frame, (0, 0), fx=0.5, fy=0.5)
    ## Create a mask image for drawing purposes
    mask = np.zeros_like(resize)

    RESET = 1
    MINIMUM_POINTS = 3
    MIN_EIGENVAL = 0.05

    height, width = resize.shape[:2]
    minDistanceP = (width + height) / 2 / 40
    ## params for ShiTomasi corner detection
    FEATURE_PARAMS_1_1_1 = dict(maxCorners=5,
                          qualityLevel=0.2,
                          minDistance =minDistanceP,
                          blockSize=7)
    ## Parameters for lucas kanade optical flow
    LK_PARAMS = dict(winSize=(15, 15),
                     maxLevel=2,
                     criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03),
                     minEigThreshold=MIN_EIGENVAL)
    ### TODO mineigenvals

    def transCoord(a):
        return([a[0]-(0*width/2), (0*height/2)-a[1], 1])

    # def selectPoints()
    old_gray = cv2.cvtColor(resize, cv2.COLOR_BGR2GRAY)
    p0 = cv2.goodFeaturesToTrack(old_gray, mask=None, **FEATURE_PARAMS_1_1_1)

    count = 0
    totalR = 0
    totalX = 0
    totalY = 0
    while (1):
        ## Counter- resets at a certain count
        count += 1
        # if count == 60:
        #     None
        # else:
        #     None

        ## Read Video
        ret, frame = cap.read()
        if frame is None:
            break
        resize = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5)
        frame_gray = cv2.cvtColor(resize, cv2.COLOR_BGR2GRAY)

        ## calculate optical flow
        p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, frame_gray, p0, None, **LK_PARAMS)
        ## Replace "good_new" with new points


        good_new = p1[st == 1]
        good_old = p0[st == 1]
        ### TODO Need a way to maintain points if tracking fails

        try:
            x_prime = np.matrix(np.apply_along_axis(transCoord, 1, good_new))
            x = np.matrix(np.apply_along_axis(transCoord, 1, good_old))
        except:
            print good_new.shape, good_old.shape
        # print x_prime
        # print x
        x_inv = np.linalg.pinv(x)
        t = np.asarray(x_inv * x_prime)
        # transpose = np.asarray(np.transpose(t))
#        print "========================="
#        print transpose.round(4)
        # print x_inv.shape, x_prime.shape
        # print t.round(4)
        # print "========================="
        # check = x * t
        # print check.round(3)

        opfl_eigenval_error = False
        if len(good_new) < len(p1):
            print "ERR %.f|OPFL/EIGEN- " \
                  "Points: %.f,  Tracked: %.f,  Threshold: %g" \
                  %(count, len(p1), len(good_new), MIN_EIGENVAL)
            opfl_eigenval_error = True

        opfl_points_error = False
        if len(good_new) < MINIMUM_POINTS:
            print "ERR %.f|OPFL/POINTS- " \
                  "Points: %.f" \
                  %(count, len(good_new))
            opfl_points_error = True

        opfl_corner_error = False
        if np.abs(t[0][2]) > 0.05 or np.abs(t[1][2]) > 0.05 or np.abs(t[2][2]-1) > 0.05:
            print "ERR %.f|OPFL/BR- " \
                  "Bottom row: %.5f, %.5f, %.5f.  Points: %.f" \
                  %(count, t[0][2].round(5), t[1][2].round(5), t[2][2].round(5), len(good_new))
            opfl_corner_error = True

        opfl_cos_error = False
        if np.abs(t[1][1] - t[0][0]) > 0.075:
            print "ERR %.f|OPFL/COS- " \
                  "Cosine values: %.5f, %.5f.  Points: %.f" \
                  %(count, t[0][0].round(5), t[1][1].round(5), len(good_new))
            opfl_cos_error = True

        opfl_sin_error = False
        if np.abs(t[0][1] - t[1][0]) > 0.075:
            print "ERR %.f|OPFL/SIN- " \
                  "Sine values: %.5f, %.5f.  Points: %.f" \
                  %(count, t[0][1].round(5), t[1][0].round(5), len(good_new))
            opfl_sin_error = True
            print t.round(4)

        opfl_matrix_error = opfl_corner_error or opfl_cos_error or opfl_points_error

        ## draw the tracks
        terms = 0
        if opfl_matrix_error:
            for i, (new, old) in enumerate(zip(good_new, good_old)):
                a, b = new.ravel()
                c, d = old.ravel()
                mask = cv2.line(mask, (a, b), (c, d), (0, 0 , 255), 2)
                resize = cv2.circle(resize, (a, b), 5, color[i].tolist(), -1)
        else:
            for i, (new, old) in enumerate(zip(good_new, good_old)):
                a, b = new.ravel()
                c, d = old.ravel()
                mask = cv2.line(mask, (a, b), (c, d), (0,255,0), 1)
                resize = cv2.circle(resize, (a, b), 5, (255,0,255), -1)
            # print t.round(4)

        no_terms_error = False

        old_gray = frame_gray.copy()
        # if opfl_matrix_error == True:
        #     None
        # el
        if count%RESET == 0: #or error == True:
            p0 = cv2.goodFeaturesToTrack(frame_gray, mask=None, **FEATURE_PARAMS_1_1_1)
        else:
            p0 = p1

        img = cv2.add(resize, mask)
        cv2.imshow("Optical Flow", img)
        ## Now update the previous frame and previous points
        # if (opfl_corner_error or opfl_cos_error):
        #     cv2.waitKey()
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    print "======================================================="
    print "Frame Count- %.f,  Point Reset- %.f,  Eigen Val Threshold- %g" %(count, RESET, MIN_EIGENVAL)
    print "Displacement X: %.2f,  Displacement Y: %.2f" % (round(totalX, 2), round(totalY, 2))
    cv2.imshow("Optical Flow", img)
    cv2.waitKey(0)
    cap.release()
    cv2.destroyAllWindows()
