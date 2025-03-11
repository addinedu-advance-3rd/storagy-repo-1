import cv2
import numpy as np
import os

# ArUco 딕셔너리 선택
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

# 큰 ArUco 마커 생성 (ID 10)
big_marker_size = 400  # 400x400 px 크기
big_marker = np.zeros((big_marker_size, big_marker_size, 1), dtype=np.uint8)
cv2.aruco.drawMarker(aruco_dict, 10, big_marker_size, big_marker, 1)

# 작은 ArUco 마커 생성 (ID 20)
small_marker_size = 100  # 작은 마커 크기
small_marker = np.zeros((small_marker_size, small_marker_size, 1), dtype=np.uint8)
cv2.aruco.drawMarker(aruco_dict, 20, small_marker_size, small_marker, 1)

# 큰 마커 중앙에 작은 마커 삽입
x_offset = big_marker_size // 2 - small_marker_size // 2
y_offset = big_marker_size // 2 - small_marker_size // 2
big_marker[y_offset:y_offset + small_marker_size, x_offset:x_offset + small_marker_size] = small_marker

# 결과 출력 및 저장
cv2.imshow("Nested ArUco Marker", big_marker)
cv2.imwrite("nested_aruco_marker.png", big_marker)
# save path
save_path = os.path.join(os.path.dirname(__file__), "nested_aruco_marker.png")
cv2.imwrite(save_path, big_marker)
cv2.waitKey(0)
cv2.destroyAllWindows()
