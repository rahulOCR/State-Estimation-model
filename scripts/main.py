from kalmanfilter import KalmanFilter
import cv2

# Kalman Filter
kf = KalmanFilter()

img = cv2.imread("bck.jpg")

ball1_positions = [(50, 100), (100, 100), (150, 100), (200, 100), (250, 100), (300, 100), (350, 100), (400, 100), (450, 100)]

ball2_positions = [(4, 300), (61, 256), (116, 214), (170, 180), (225, 148), (279, 120), (332, 97),
         (383, 80), (434, 66), (484, 55), (535, 49), (586, 49), (634, 50),
         (683, 58), (731, 69), (778, 82), (824, 101), (870, 124), (917, 148),
         (962, 169), (1006, 212), (1051, 249), (1093, 290)]

x = [(2, 2), (3, 4), (4, 8), (5, 16), (6, 32), (7, 64), (8, 128), (9, 256), (10, 512), (11, 1024)]

for pt in ball2_positions:
    cv2.circle(img, pt, 15, (0, 20, 220), -1)
    predicted = kf.predict(pt[0], pt[1])

    cv2.circle(img, predicted, 15, (20, 220, 0), 4)

for i in range(10):
    predicted = kf.predict(predicted[0], predicted[1])
    cv2.circle(img, predicted, 15, (20, 220, 0), 2)

    print(predicted)


cv2.imshow("Img", img)
cv2.waitKey(0)
