import serial
import time
import math

class io_expansion_board:
    '''
    LED1 = green LED
    LED2 = yellow LED
    LED3 = red LED
    If automatic_update is set to True, then each time a change is made to an LED or PWM the state is sent to the microcontroller.
    '''
    def __init__(self, automatic_update, PWM1_min = -1, PWM1_max = 1, PWM2_min = -1, PWM2_max = 1, LED1 = 0, LED2 = 0, LED3 = 0, PWM1 = 0, PWM2 = 0, com_port = 0, baud_rate = 115200):
        if not(automatic_update in [True, False]):
            raise ValueError("automatic_update can only be True of False")

        self.automatic_update = automatic_update
        self.ser = serial.Serial("/dev/ttyUSB" + str(com_port), baud_rate)
        self.LED1 = 0
        self.LED2 = 0
        self.LED3 = 0
        self.PWM1 = 0
        self.PWM2 = 0
        self.PWM1_min = PWM1_min
        self.PWM1_max = PWM1_max
        self.PWM2_min = PWM2_min
        self.PWM2_max = PWM2_max
        self.LED_state_set(LED1=LED1, LED2=LED2, LED3=LED3)
        self.PWM_value_set(PWM1=PWM1, PWM2=PWM2)
        self.update_state()
        self.ser.reset_input_buffer()

 
    def send_data(self, data):
        '''
        'data' is a list containing only data that can be cast to byte type.
        If the above condition is not met a TypeError is raised.
        
        Data given is formatted as such:
        First 2 bytes: 0x55 and 0xAA for checks on the microcontroller
        Next 4 bytes are divided for the 2 PWMs which have to have values between 1000 and 2000(otherwise a ValueError is raised):
            -first 2 bytes of these 4 are for the 1st PWM
            -next 2 for the 2nd PWM
            -the value of the PWM is calculated as follows (1st byte) * 0x100 + (2nd byte) 
        '''
        if type(data) != list:
            raise TypeError("'data' isn't of list type.")

        if not(2000 >= data[2] * 0x100 + data[3] >= 1000):
            raise ValueError(f"PWM1 outside desired range([1000, 2000]). Value given is {data[2] * 0x100 + data[3]}.")

        if not(2000 >= data[4] * 0x100 + data[5] >= 1000):
            raise ValueError(f"PWM2 outside desired range([1000, 2000]). Value given is {data[4] * 0x100 + data[5]}.")

        data = bytearray(data)
        self.ser.write(data)
    
    
    def update_state(self):
        to_send = [0x55, 0xAA]

        PWM1  = int((self.PWM1 / 2 + 0.5) * 1000 + 1000)
        PWM11 = PWM1 // 0x100
        PWM12 = PWM1 %  0x100
        to_send.append(PWM11)
        to_send.append(PWM12)

        PWM2  = int((self.PWM2 / 2 + 0.5) * 1000 + 1000)
        PWM21 = PWM2 // 0x100
        PWM22 = PWM2 %  0x100
        to_send.append(PWM21)
        to_send.append(PWM22)
        
        all_LEDs = self.LED1 * (2 ** 6) + self.LED2 * (2 ** 4) + self.LED3 * (2 ** 2)
        to_send.append(all_LEDs)

        self.send_data(to_send)


    def LED_state_set(self, LED1=None, LED2=None, LED3=None):
        '''
        'LED1', 'LED2' or 'LED3' can be:
            -0 = OFF
            -1 = ON
            -2 = BLINKING
        If value given is anything else ValueError is raised.
        '''
        if LED1 == None:
            LED1 = self.LED1
        if LED2 == None:
            LED2 = self.LED2
        if LED3 == None:
            LED3 = self.LED3

        for index, state in enumerate([LED1, LED2, LED3]):
            if not(state in [0, 1, 2]):
                raise ValueError(f"LED{index + 1} has an invalid value. It should be 1, 2 or 3, but {state} was provided.")

        self.LED1 = LED1
        self.LED2 = LED2
        self.LED3 = LED3
        if self.automatic_update:
            self.update_state()


    def PWM_value_set(self, PWM1 = None, PWM2 = None):
        '''
        'PWM1' or 'PWM2' can be only in the [-1, 1] interval. Anything else raises a ValueError.
        '''
        if PWM1 == None:
            PWM1 = self.PWM1
        if PWM2 == None:
            PWM2 = self.PWM2
        
        limits = [[self.PWM1_min, self.PWM1_max], [self.PWM2_min, self.PWM2_max]]

        for index, value in enumerate([PWM1, PWM2]):
            if not(limits[index][0] <= value <= limits[index][1]):
                pass
                #print(f"PWM{index + 1} must be in the [{limits[index][0]}, {limits[index][1]}] interval. Value given is {value}.")
        PWM1 = max(min(self.PWM1_max, PWM1), self.PWM1_min)
        PWM2 = max(min(self.PWM2_max, PWM2), self.PWM2_min)


        self.PWM1 = PWM1
        self.PWM2 = PWM2
        if self.automatic_update:
            self.update_state()


    class LED_state:  #kind of an enum
        OFF   = 0
        ON    = 1
        BLINK = 2


    def read_buttons(self):
        button1 = 0
        button2 = 0

        self.ser.timeout=0.01
        read_raw = self.ser.read()
        self.ser.timeout=None

        i = 0
        while read_raw != b'':
            read = int.from_bytes(read_raw, byteorder='little')
            if read != 0x55:
                self.ser.reset_input_buffer()
                raise RuntimeError(f"Recieved data is corrupted: hex->{hex(read)}, raw->{read_raw}. Flushed input buffer.1 iteration {i}")

            read_raw = self.ser.read()
            read = int.from_bytes(read_raw, byteorder='little')
            if read != 0xAA:
                self.ser.reset_input_buffer()
                raise RuntimeError(f"Recieved data is corrupted: hex->{hex(read)}, raw->{read_raw}. Flushed input buffer.1 iteration {i}")

            read_raw = self.ser.read()
            print(read_raw)
            read = int.from_bytes(read_raw, byteorder='little')
            if read // 0x10 == 0xF:
                button1 += 1

            if read % 0x10 == 0xF:
                button2 += 1

            self.ser.timeout=0.01
            read_raw = self.ser.read()
            self.ser.timeout=None
            i += 1

        if i != 0:
            print(button1, button2)
        return (button1, button2)


    def __del__(self):
        self.ser.close()


if __name__ == "__main__":
    comm = io_expansion_board(False, PWM2_min=-0.8)

    start = time.time()
    while(1):
        now = time.time()

        sine1 = math.sin(now * 2 - start)
        sine2 = math.sin(now * 2.6 - start)

        P1 = float(input("PWM1: "))
        P2 = float(input("PWM2: "))
        print()

        comm.PWM_value_set(PWM1=P1)
        comm.PWM_value_set(PWM2=P2)

        button1, button2 = comm.read_buttons()

        if button1 % 2 == 1:
            comm.LED_state_set(LED1 = int(not comm.LED1))

        if button2 % 2 == 1:
            comm.LED_state_set(LED2 = int(not comm.LED2))

        comm.LED_state_set(LED3 = 2)
        
        comm.update_state()
