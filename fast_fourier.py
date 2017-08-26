import numpy as np
import cv2
from matplotlib import pyplot as plt
import matplotlib.animation as animation
# img = cv2.imread('ceiling1.jpg',0)
#
# dft = cv2.dft(np.float32(img),flags = cv2.DFT_COMPLEX_OUTPUT)
# dft_shift = np.fft.fftshift(dft)
#
# magnitude_spectrum = 20*np.log(cv2.magnitude(dft_shift[:,:,0],dft_shift[:,:,1]))

# plt.subplot(121),plt.imshow(img, cmap = 'gray')
# plt.title('Input Image'), plt.xticks([]), plt.yticks([])
# plt.subplot(122),plt.imshow(magnitude_spectrum, cmap = 'gray')
# plt.title('Magnitude Spectrum'), plt.xticks([]), plt.yticks([])
# plt.show()

cap = cv2.VideoCapture('drop_tile.mp4')
count = False
while (cap.isOpened()):
    ret, frame = cap.read()
    resize = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5)
    gray = cv2.cvtColor(resize, cv2.COLOR_BGR2GRAY)

    dft = cv2.dft(np.float32(gray), flags=cv2.DFT_COMPLEX_OUTPUT)
    dft_shift = np.fft.fftshift(dft)

    magnitude_spectrum = (20 * np.log(cv2.magnitude(dft_shift[:, :, 0], dft_shift[:, :, 1])))/256
    cv2.imshow("FFT", magnitude_spectrum)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()