import cv2
import pyautogui

# 현재 마우스 위치 찾기
current_mouse_position = pyautogui.position()

# 현재 마우스 위치 출력
print(f"Current mouse position: {current_mouse_position}")

pyautogui.moveTo(500, 200, 1)
pyautogui.click(button='right')

# 현재 마우스 위치 찾기
current_mouse_position = pyautogui.position()

# 현재 마우스 위치 출력
print(f"Current mouse position: {current_mouse_position}")