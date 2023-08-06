from time import sleep
from pynput.keyboard import Controller, Key

keyboard = Controller()

sleep(3)

keyboard.press(Key.ctrl)
keyboard.press('c')
keyboard.release('c')
keyboard.release(Key.ctrl)