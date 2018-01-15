import pygame
import time
pygame.mixer.init()

pygame.mixer.music.load("/home/pi/project/codey_test/images/CodeDownloadSuc.wav")
pygame.mixer.music.play()
time.sleep(2)
pygame.mixer.music.load("/home/pi/project/codey_test/images/readyWork.wav")
pygame.mixer.music.play()
time.sleep(2)
pygame.mixer.music.load("/home/pi/project/codey_test/images/startWork.wav")
pygame.mixer.music.play()
time.sleep(2)