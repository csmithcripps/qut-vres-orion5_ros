
from time import sleep
from random import randint


'''

Used to build a moveit interface without the controller attached.
Also enables developing with the controller on something other than a raspberry pi.

Call get_controls to randomly assign control positions.



'''


class ps2ctrltest:

    def __init__(self):



        # states for buttons, initialised in off position (active high)
        self.btn_states = [0]*14

        self.TRIANGLE = 0
        self.CROSS = 1
        self.DPAD_UP = 2
        self.DPAD_DOWN = 3
        self.DPAD_LEFT = 4
        self.DPAD_RIGHT = 5
        self.SQUARE = 6
        self.CIRCLE = 7
        self.SELECT = 8
        self.START= 9
        self.L1 = 10
        self.L2 = 11
        self.R1 = 12
        self.R2 = 13

        # states for analog sticks, initialised in neutral position
        self.stick_states = [128, 128, 128, 128]
        self.LEFT_X = 0
        self.LEFT_Y = 1
        self.RIGHT_X = 2
        self.RIGHT_Y = 3


        # Actually it doesn't run this fast. I think the pi is limited to about 6kHz.
        # Would be faster if this were written in C with a python wrapper I guess.
        # Project for a different time maybe.
        

    # Transmit one byte down the CMD (MOSI) line.
    # Pre: CMD, CLK high. ATT has been high for a few clock cycles.
    # Post: same.
    


    # The main function of this class.
    # Initiates a full exchagne with the controller and returns the payload from the controller.
    # It will update the STATE attributes of the ps2ctrl object.
    def get_controls(self):

        # get a random payload
        payload = 6*[0]

        for i in range(6):
            payload[i] = randint(1,255)
            
        # Access the individual button states using bit shifting and a bit mask.
        # Flip the result because the controller sends active low signals, but our button state info is active high.
        self.btn_states[self.DPAD_UP] = not ((payload[0] >> 4) & 1)
        self.btn_states[self.DPAD_DOWN] = not ((payload[0] >> 6) & 1)
        self.btn_states[self.TRIANGLE] = not ((payload[1]  >> 4) & 1)
 
        self.stick_states[self.RIGHT_X]= payload[4]
	self.stick_states[self.RIGHT_Y]= payload[5]
        
        return payload


if __name__ == "__main__":

    
    pass
