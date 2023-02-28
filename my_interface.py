# python code to interface the logitech joystick and Keyboard
from curses.ascii import isalpha
from multiprocessing.connection import Listener
import struct
# import this
import pygame
import time
import sys
import os
import subprocess
import threading
from pynput import keyboard
from tomlkit import key
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
global done 
done= False

# Used to manage how fast the screen updates
clock = pygame.time.Clock()

#create global variables to store the joystick values and initialize them to zero
global pitch
global yaw 
global lift 
global roll 
pitch =0.0; yaw =0.0; lift =0.0; roll =0.0


#make a function for joystick data
def get_joystick_data():
    #  done=False
    #  while (done!=True):
     for event in pygame.event.get():
        print(event)
        if event.type == pygame.JOYBUTTONDOWN:
                    print("Button:",event.dict['button'],"pressed")
                    if event.dict['button'] == 0:
                        global done
                        done = True
                        # return done
                        break
        elif event.type == pygame.JOYAXISMOTION:
                for i in range(axes):
                    global pitch 
                    pitch = joystick.get_axis(1)
                    print("Pitch:" , pitch)
                    global yaw
                    yaw = joystick.get_axis(2)
                    print("Yaw:" , yaw)
                    global lift
                    lift = joystick.get_axis(3)
                    print("Lift:" , lift)
                    global roll
                    roll = joystick.get_axis(0)
                    print("Roll:" , roll)
                    # continue
                # return done
#make a function for getting keyboard data
def get_keyboard_data():
        # done=False
        global pitch, yaw, lift, roll
        with keyboard.Events() as events:
            # print(keyboard.KeyCode(char="a"))
            for event in events:
                # print(type(event.key.char))
                if event.key == keyboard.Key.esc or event.key == keyboard.Key.space:
                 global done
                 done=True
                #  return done
                 break
                if  event.key == keyboard.Key.up or event.key== keyboard.Key.down or event.key== keyboard.Key.left or event.key== keyboard.Key.right:
                    if event.key == keyboard.Key.up and type(event) == keyboard.Events.Press:
                        if pitch <= 0.9:
                         pitch = pitch + 0.1
                        print("Pitch: ", pitch)
                    elif event.key == keyboard.Key.down and type(event) == keyboard.Events.Press:
                        if pitch >= -0.9:
                         pitch = pitch - 0.1
                        print("Pitch: ", pitch)
                    elif event.key == keyboard.Key.left and type(event) == keyboard.Events.Press:
                        if roll <= 0.9:
                         roll = roll + 0.1
                        print("Roll: ", roll)
                    elif event.key == keyboard.Key.right and type(event) == keyboard.Events.Press:
                        if roll >=-0.9:
                         roll = roll - 0.1
                        print("Roll: ", roll) 
                else:
                     if event.key == keyboard.KeyCode(char="a") and type(event) == keyboard.Events.Press:
                            if lift <= 0.9:
                             lift = lift + 0.1
                            print("Thrust: ", lift)  
                     elif event.key == keyboard.KeyCode(char="z") and type(event) == keyboard.Events.Press:
                            if lift >= -0.9:
                             lift = lift - 0.1
                            print("Thrust: ", lift)     
                     elif event.key == keyboard.KeyCode(char="q") and type(event) == keyboard.Events.Press:
                            if yaw <= 0.9:
                             yaw = yaw + 0.1
                            print("Yaw: ", yaw)
                     elif event.key == keyboard.KeyCode(char="w") and type(event) == keyboard.Events.Press:
                            if yaw >= -0.9:
                             yaw = yaw - 0.1 
                            print("Yaw: ", yaw)
                     elif event.key == keyboard.KeyCode(char ='1') and type(event) == keyboard.Events.Press:
                         print( "Mode1")
                     elif event.key == keyboard.KeyCode(char ='0') and type(event) == keyboard.Events.Press:
                         print( "Mode0")
                     else:
                         break
        # return done
        #need to add for yaw control and pitch control ??aaa
#function to get the data from the joystick or keyboard
def get_data():
     return pitch, yaw, lift, roll


# -------- Main Program Loop -----------
# while done!=True:
        # print("in python code")
if (joysticks > 0):
 thread1=threading.Thread(target=get_joystick_data)
 thread1.start()
thread2=threading.Thread(target=get_keyboard_data)
thread2.start()
        # if(joysticks > 0):
            # done=get_joystick_data()
            # print(done)
            # print(get_data())
        # else:
            # done=get_keyboard_data()
            # print(get_data())
# if (done==True):
    
thread1.join()
thread2.join()
print(get_data()) 