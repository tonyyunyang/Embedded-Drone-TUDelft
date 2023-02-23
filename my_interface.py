# python code to interface the logitech joystick and Keyboard
from curses.ascii import isalpha
from multiprocessing.connection import Listener
import struct
import this
import pygame
import time
import sys
import os
import subprocess
from pynput import keyboard
# Initialize the joysticks
pygame.init()
pygame.joystick.init()

# Get ready to print
joysticks= pygame.joystick.get_count()
print ("Number of joysticks: ", joysticks)
if joysticks >0:
    joystick = pygame.joystick.Joystick(0)
    joystick.init() # Initialize the first joystick
    print ("Joystick name: ", joystick.get_name())
    axes=joystick.get_numaxes()
    print ("Number of axes: ", axes)
    buttons=joystick.get_numbuttons()
    print ("Number of buttons: ", buttons)
    print ("Number of hats: ", joystick.get_numhats())

     

# Loop until the user clicks the close button.
done = False

# Used to manage how fast the screen updates
clock = pygame.time.Clock()

#create global variables to store the joystick values and initialize them to zero
global pitch
global yaw 
global lift 
global roll 
pitch =0; yaw =0; lift =0; roll =0


#make a function for joystick data
def get_joystick_data():
     done=False
     for event in pygame.event.get():
        if event.type == pygame.JOYBUTTONDOWN:
                    print("Button:",event.dict['button'],"pressed")
                    if event.dict['button'] == 0:
                        done = True
                        break
        if event.type == pygame.JOYAXISMOTION:
                for i in range(axes):
                    pitch = joystick.get_axis(1)
                    # print("Pitch:" , pitch)
                    yaw = joystick.get_axis(2)
                    # print("Yaw:" , yaw)
                    lift = joystick.get_axis(3)
                    # print("Lift:" , lift)
                    roll = joystick.get_axis(0)
                    # print("Roll:" , roll)
                    continue
                return done
#make a function for getting keyboard data
def get_keyboard_data():
        done=False
        global pitch, yaw, lift, roll
        with keyboard.Events() as events:
            print(keyboard.KeyCode(char="a"))
            for event in events:
                # print(type(event.key.char))
                if event.key == keyboard.Key.esc or event.key == keyboard.Key.space:
                 done=True
                 break
                if  event.key == keyboard.Key.up or event.key== keyboard.Key.down or event.key== keyboard.Key.left or event.key== keyboard.Key.right:
                    if event.key == keyboard.Key.up and type(event) == keyboard.Events.Press:
                        pitch = pitch + 0.1
                        print("Pitch: ", pitch)
                    elif event.key == keyboard.Key.down and type(event) == keyboard.Events.Press:
                        pitch = pitch - 0.1
                        print("Pitch: ", pitch)
                    elif event.key == keyboard.Key.left and type(event) == keyboard.Events.Press:
                        roll = roll + 0.1
                        print("Roll: ", roll)
                    elif event.key == keyboard.Key.right and type(event) == keyboard.Events.Press:
                        roll = roll - 0.1
                        print("Roll: ", roll) 
                else:
                     if event.key == keyboard.KeyCode(char="a") and type(event) == keyboard.Events.Press:
                            lift = lift + 0.1
                            print("Thrust: ", lift)  
                     elif event.key == keyboard.KeyCode(char="z") and type(event) == keyboard.Events.Press:
                            lift = lift - 0.1
                            print("Thrust: ", lift)     
                     elif event.key == keyboard.KeyCode(char="q") and type(event) == keyboard.Events.Press:
                            yaw = yaw + 0.1
                            print("Yaw: ", yaw)
                     elif event.key == keyboard.KeyCode(char="w") and type(event) == keyboard.Events.Press:
                            yaw = yaw - 0.1
                            print("Yaw: ", yaw)
        return done
        #need to add for yaw control and pitch control ??aaa
#function to get the data from the joystick or keyboard
def get_data():
     return pitch, yaw, lift, roll


# -------- Main Program Loop -----------
while done==False:
        print("in python code")
        if(joysticks > 0):
            done=get_joystick_data()
            print(get_data())
        else:
            done=get_keyboard_data()
            print(get_data())
