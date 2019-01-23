import RPi.GPIO as GPIO
from time import sleep
from os import remove

'''

Jenna Riseley 2017


A ps2ctrl object allows a raspberry pi to communicate with a playstation 2 controller.
The protocol is basically the same as SPI, but it has a few quirks which make off-the-shelf SPI tools unsuitable:
    - bytes are sent LSB first
    - clock, MOSI and MISO lines idle at HIGH
    - additional ACK line... but actually we can ignore acks and it still works.

The playstation controller pins must be connected to the following pins of a raspberry pi.

1 DATA - GPIO22 open collector line, the controller can only drive it low so it needs to idle high. Use a pullup resistor to tie this line to 3.3V.
2 CMD - GPIO27
3 rumble control - not used
4  GND - any ground pin
5  VCC - any 3.3V pin
6  ATT - GPIO17
7  CLK - GPIO4
8  ??? not used
9 ACK - not used

Protocol and pinout information thanks to http://pinouts.ru/Game/playstation_9_pinout.shtml


'''


class ps2ctrl:

    def __init__(self):

        GPIO.setmode(GPIO.BCM)

        self.DATA = 22 # MISO - white
        self.CMD =27 # MOSI - oramge
        self.ATT = 17 # CHS0 - purple
        self.CLK = 4 # SCLK - yellow

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
        self.frequency = 250000.0
        self.clock_delay = 1.0/(2*self.frequency) # time to wait after clock edge
        self.DELAY = 0.0006 # delay between bursts

        self.payload_types = [65, 115]

        GPIO.setup(self.DATA, GPIO.IN)
        GPIO.setup(self.CMD, GPIO.OUT)
        GPIO.setup(self.ATT, GPIO.OUT)
        GPIO.setup(self.CLK, GPIO.OUT)

        GPIO.output(self.ATT, GPIO.HIGH) # attention command idles at HIGH
        GPIO.output(self.CLK, GPIO.HIGH) # clock idles at HIGH
        GPIO.output(self.CMD, GPIO.HIGH) # command idles at HIGH


    # Transmit one byte down the CMD (MOSI) line.
    # Pre: CMD, CLK high. ATT has been high for a few clock cycles.
    # Post: same.
    def tx(self, byte):
        for i in byte:
            # load new data and send clock low
            GPIO.output(self.CLK, GPIO.LOW) # Pull clock low
            GPIO.output(self.CMD, i) # load next bit for transmission
            sleep(self.clock_delay) # wait half a clock cycle
            GPIO.output(self.CLK, GPIO.HIGH) # rising clock edge should trigger controller to read bit
            sleep(self.clock_delay) # wait

        GPIO.output(ps2ctrl.CMD, GPIO.HIGH)

    # Receive a byte or bytes rom the controller from the DATA (MISO) line.
    def rx(self, num_bytes = 1):

        rx_bytes = []

        for ith_byte in range(num_bytes):

            received_byte = 0

            for i in range(8):
                GPIO.output(self.CLK, GPIO.LOW) # Pull clock low
                sleep(self.clock_delay) # wait half a clock cycle
                GPIO.output(self.CLK, GPIO.HIGH) # rising clock edge should trigger controller to write a bit
                sleep(self.clock_delay) # wait
                received_byte = received_byte + (2**i)*GPIO.input(self.DATA)

            rx_bytes.append(received_byte)
            # wait a bit
            sleep(ps2ctrl.DELAY)

        if num_bytes==1:
            return rx_bytes[0]
        else:
            return rx_bytes


    # Transmit (MOSI) and receive (MISO) one byte simultaneously.
    def xfer(self, byte):

        received_byte = 0
        j = 0
        for i in byte:
            # load new data and send clock low
            GPIO.output(self.CLK, GPIO.LOW) # Pull clock low
            GPIO.output(self.CMD, i) # load next bit for transmission
            sleep(self.clock_delay) # wait half a clock cycle
            GPIO.output(self.CLK, GPIO.HIGH) # rising clock edge should trigger controller to read bit
            sleep(self.clock_delay) # wait
            received_byte = received_byte + 2**j*(GPIO.input(self.DATA))
            j = j+1
        GPIO.output(ps2ctrl.CMD, GPIO.HIGH)
        return received_byte


    # The main function of this class.
    # Initiates a full exchagne with the controller and returns the payload from the controller.
    # It will update the STATE attributes of the ps2ctrl object.
    def get_controls(self):

        # Pull ATT (chip select) low then wait a few clock cycles.
        # This signals the start of a transmission to the controller.
        GPIO.output(ps2ctrl.ATT, GPIO.LOW)
        sleep(10*ps2ctrl.clock_delay)

        # Send 0x01  least significant bit first.
        # This signals the start of a transmission to the controller.
        ps2ctrl.tx([1, 0, 0, 0, 0, 0, 0, 0])

        # Wait a few clock cycles for the controller to ACK.
        # Actually we aren't listening to the ACK, but we assume it occurs.
        sleep(ps2ctrl.DELAY)

        # Send the next command byte, 0x42 (LSB first).
        # At the same time, receive the payload type from the controller.

        # 0x41 = digital control (2 byte payload)
        # 0x73 = analogue controller in red mode (6 byte payload)

        payload_type = ps2ctrl.xfer([0, 1, 0, 0, 0, 0, 1, 0])

        if payload_type == 65:
            payload_size = 2
        elif payload_type == 115:
            payload_size = 6
        else:
            print("Payload type error")

        # wait a bit
        sleep(ps2ctrl.DELAY)

        # Send some clock pulses and wait.
        # The controller should send 0x5A to indicate that the payload is about to start.
        if ps2ctrl.rx() is not 90:
            print("Payload not ready. Controller didn't send 0x5a")

        # Start receiving the payload.
        payload = ps2ctrl.rx(payload_size)

        # Pull ATT (chip select) high again to signal to the controller that the transmission has ended.
        GPIO.output(ps2ctrl.ATT, GPIO.HIGH)

        # Next update the button state information.
        # It's not ideal to do this in the same thread as the thingy receiving and transmitting...
        # Ideally the rx/tx method would update some kind of buffer and signal that there is new information,
        # while another thread decodes the information into button states.

        # Access the individual button states using bit shifting and a bit mask.
        # Flip the result because the controller sends active low signals, but our button state info is active high.
        self.btn_states[self.DPAD_UP] = not ((payload[0] >> 4) & 1)
        self.btn_states[self.DPAD_DOWN] = not ((payload[0] >> 6) & 1)
        self.btn_states[self.TRIANGLE] = not ((payload[1]  >> 4) & 1)

        self.stick_states[self.RIGHT_X]= payload[4]
	self.stick_states[self.RIGHT_Y]= payload[5]

        return payload


if __name__ == "__main__":

    ps2ctrl = ps2ctrl()
    #ps2ctrl.interrupt_thread.start()
    sleep(4*ps2ctrl.clock_delay)


    #GPIO.cleanup()
