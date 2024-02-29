

# ----------------------------------------------------------------------
# tofdbeam.py MIDI controller inspired by Roland's D-Beam
# special thanks to: 
# Kevin McAleer for providing the library for VL53LOX
# https://github.com/kevinmcaleer/vl53l0x
# Craig Barnes for BigMidiInteger routine from Github
# https://github.com/cjbarnes18/micropython-midi
# 
# The measured data coming from the vl5310x can vary per sensor. 
# Please check and correct in the code, see also remarks further below (in the while loop)
# This tofdbeam_FINAL.py working with CAD diagram tofdbeam.kicad_sch Rev: Versie 1.4
#
# Frank van der Borg
# Netherlands
# February 2024
# ----------------------------------------------------------------------

from machine import Pin, I2C, DAC, PWM
from vl5310x import VL53L0X
from time import sleep
from machine import UART
from machine import TouchPad
import machine
import utime


from ssd1306 import SSD1306_I2C
import ubinascii
import uos
import ujson



#timer1 = machine.Timer(0)
#timer2 = machine.Timer(1)
#timer3 = machine.Timer(2)

clock_ticks = 0
clock_ticks_analog = 0

data_saved = True       # flag to check if parameters are saved, start with assumption that values are saved in file.
seq_runs = False        # flag to check if sequencer (drumcomputer, etc) is running

# (zero-based channel counting, actual midi channel = 1), give volume a starting level, from NVM later on
# starting with some default (only needed for first run) values:
# #1 = Modulation wheel
# #7 = Volume

volume= 60
bpm = 120
channel = 0
cc1 = 1 
cc2 = 7

# Define initial state of output variables for switches
key1=False
key2=False
key3=False
key4=False
key5=False



# LEDs indicating which switch is pressed:

led1 = Pin(32, Pin.OUT)       # green LED

led2 = Pin(33, Pin.OUT)       # blue LED
led3 = Pin(27, Pin.OUT)       # yellow LED
led4 = Pin(2, Pin.OUT)        # red LED

led5 = Pin(19, Pin.OUT)       # White EDIT LED

led = Pin(5, Pin.OUT)         # green LED indicate distance is within range (set by range value) and MIDI is send


led.value(0)
led1.value(0)
led2.value(0)
led3.value(0)
led4.value(0)
led5.value(0)

# setting up I2C 
# print("setting up i2c")
i2c=I2C(scl=Pin(22), sda=Pin(21))

oled_width=128
oled_height=64
oled = SSD1306_I2C(oled_width, oled_height, i2c)

lin_hight = 9
col_width = 8

def text_write(text,lin, col):
  oled.text(text,col*col_width,lin*lin_hight)

oled.fill(0)
text_write("Time-Of-Flight", 1, 1)
text_write("- D-Beam -", 3, 3)
text_write("NL-Frank", 5, 4)

# oled.rect(5, 5, 116, 52, 1)
oled.show()

def display_parameters(bpm, channel, cc1, cc2):
    oled.fill(0)


    lin_height = 9

    bpm_string = 'BPM  :%7.0f' % bpm
    channel_string = 'MIDI :%7.0f' % (channel+1)
    cc1_string = 'CC1  :%7.0f' % cc1
    cc2_string = 'CC2  :%7.0f' % cc2
    
    oled.text(bpm_string, 0, lin_height * 1)
    oled.text(channel_string, 0, lin_height * 2)
    oled.text(cc1_string, 0, lin_height * 3)
    oled.text(cc2_string, 0, lin_height * 4)
    
    oled.show()


# -------------------------------------------------------------------------------------------------------------------------------------
# ------------------------------------Routine for storing parameters in eeprom on ESP32------------------------------------------------
# -------------------------------------------------------------------------------------------------------------------------------------
# Define the address for storing the parameter values in the flash memory
PARAM_FILE = "/params.json"

def save_parameters(bpm, channel, cc1, cc2):
    # Create a dictionary with the parameter values
    params = {
        'bpm': bpm,
        'channel': channel,
        'cc1': cc1,
        'cc2': cc2
    }

    # Serialize the dictionary to JSON
    param_json = ujson.dumps(params)

    # Open the file in write mode and write the JSON data
    with open(PARAM_FILE, "w") as f:
        f.write(param_json)

def load_parameters():
    try:

        # Open the file in read mode
        with open(PARAM_FILE, "r") as f:
            # Read the JSON data from the file

            param_json = f.read()

        # Deserialize the JSON to a dictionary
        params = ujson.loads(param_json)

        # Extract the parameter values from the dictionary
        bpm = params.get('bpm')
        channel = params.get('channel')
        cc1 = params.get('cc1')
        cc2 = params.get('cc2')

        # Return the parameter values
        return bpm, channel, cc1, cc2



    except OSError:
        # Return None if the file does not exist or an error occurs
        return None

# Load the parameters from the file system
loaded_params = load_parameters()


# Initialize the variables with loaded values or defaults
bpm, channel, cc1, cc2 = loaded_params or (120, 0, 1, 7)

# Example usage
print("Loaded parameters:")
print("BPM:", bpm)
print("Channel:", channel)
print("CC1:", cc1)
print("CC2:", cc2)





# Define the pin to use for the PWM output, enable_clk is the output enable (active low) for the analog clock out (High puts output in high Z)


# -------------------------------------------------------------------------------------------------------------------------------------
# ------------------------------------Bigmidi integer routine to split into MSB and LSB------------------------------------------------
# -------------------------------------------------------------------------------------------------------------------------------------
#

class BigMidiInteger:
    """Some messages use 14 bit values, these need to be spit down to
    msb and lsb before being sent."""
    def __init__(self, value):
        if 0 <= value <= 2 ** 14:
            self.msb = value // 2 ** 7
            self.lsb = value % 2 ** 7
        else:
            raise ValueError(
                'Invalid midi data value: {}'.format(value),
                'A midi datavalue must be an integer between0'
                ' and {}'.format(2 ** 14))

    def __repr__(self):
        return '<BigMidiInteger: lsb={}, msb={}>'.format(self.lsb, self.msb)

        
 

# -------------------------------------------------------------------------------------------------------------------------------------
# ----------------------------------------- interrupt routine to scan switches --------------------------------------------------------
# ------------------------------------------------------------------------------------------------------------------------------------- 
 

# Define GPIO pins for switches
switch_pins = [14, 12, 13, 15, 23]


# Debounce function, the bouncing effect depends on the switches used, this corrects this effect to some extent.

def debounce(pin):
    current_state = pin.value()
    start_time = utime.ticks_ms()

    while True:
        if current_state != pin.value():


            start_time = utime.ticks_ms()
            current_state = pin.value()
        elif utime.ticks_diff(utime.ticks_ms(), start_time) >= 75:
            return current_state

        utime.sleep_ms(25)



def handle_interrupt(pin):

  
  global key1, key2, key3, key4, key5
  
  # Debounce the pin
  debounced_state = debounce(pin)

  if debounced_state == 1:


  
    if pin == Pin(14): 
      key1 = not key1
      led1.value(not led1.value())
    elif pin == Pin(12):
      key2 = not key2
      led2.value(not led2.value())
    elif pin == Pin(13):
      key3 = not key3
      led3.value(not led3.value())
    elif pin == Pin(15):
      key4 = not key4
      led4.value(not led4.value())
    elif pin == Pin(23):
      key5 = not key5
      led5.value(not led5.value())
 
 
for pin in switch_pins:
    switch = Pin(pin, Pin.IN, Pin.PULL_DOWN)
    switch.irq(trigger=Pin.IRQ_RISING, handler=handle_interrupt)





# ---------------------------------------------------------------------------------------------------------------------------------
#--------------------------------------------------Initializing UART --------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------------------

# Initializing UART: 31.250 Hz Baudrate,  the number of bits, parity, and stop bits are already set to their default values, 
# which are typically 8 data bits, no parity, and 1 stop bit. Exactly what we need.

uart = UART(1, 31250, tx=10, rx=9)


# -------------------------------------------------------------------------------------------------------------------------------------
# ------------------------------------timer based interrupt routine for MIDI clock--------------------------------------------------
# ------------------------------------------------------------------------------------------------------------------------------------- 
# Timing between analog and midi clock tested on Arturia Drumbrute
# difficult to create analog clock out function, unfortunately pwm is not working below 500Hz, the frequncy is too low, therefor we toggle the output in an ISR
# 24 MIDI Clocks in every quarter note
# 120 BPM -> 24 * 120 MIDI Clocks per minute
# For example: 120 BPM = 120/60 = 2 BPS
# 1 BPS = 24 MIDI clks/sec -> 2X24 = 48 midi clks/sec  
# 48 – 120 gives 120/2,5 = 48 = number of midiclocks per sec
# so the ratio is: (BPM/2,5) = MIDI clock out freq, 

# we need to correct the analog clock out:  this is divided to create the same clockfrequency as by MIDI (UART output).
# Drumbrute default is: 1step (one pulse per quarter note , or ppqn) hence the division by 4 (clock_ticks % 3)
# Both outputs Midi clock and analog clock as input give the same BPM on the display of the Drumbrute.
# There are variations in timing because of the principle used here, but it works good enough for most cases, I think.


tim = machine.Timer(1)
analog_clk= Pin(9, Pin.OUT)
enable_clk = Pin(18, Pin.OUT)
tmp = int(bpm/2.5)
interval = int((1/tmp)*1000)
output_enabled = False

def tick():
    global clock_ticks
    if output_enabled == True:
      uart.write(bytes([0xF8]))
      if clock_ticks % 3 == 0:
        analog_clk(not analog_clk())
    clock_ticks += 1


#tim.init(period=100, mode=machine.Timer.PERIODIC, callback=tick)
tim.init(period=interval, mode=machine.Timer.PERIODIC, callback=lambda t: tick())


# -------------------------------------------------------------------------------------------------------------------------------------
# ------------------------------------Setting trigger out ----------------------------------------------------------
# -------------------------------------------------------------------------------------------------------------------------------------

# define trigger out and set to 5V, trigger output is buffered and level shifted to 5V by 74HCT125N to generate trigger/gate pulse
# my setup requires a 5V-> 0V pulse, a falling edge.

trigger=Pin(4, Pin.OUT)
trigger.value(1)


# setting up DAC for analog out for modular synths :


dac1=DAC(Pin(26))
dac2=DAC(Pin(25))

 

# -------------------------------------------------------------------------------------------------------------------------------------
# ------------------------------------routine for MIDI Time-of-Flight sensor VL53L0X--------------------------------------------------
# ------------------------------------------------------------------------------------------------------------------------------------- 
# check I2C connection to vl5310x
# print(i2c.scan())


# print("creating vl53lox object")
# Create a VL53L0X object
tof = VL53L0X(i2c)

# Pre: 12 to 18 (initialized to 14 by default)

# Final: 8 to 14 (initialized to 10 by default)

# the measuting_timing_budget is a value in ms, the longer the budget, the more accurate the reading. 
budget = tof.measurement_timing_budget_us
# print("Budget was:", budget)
tof.set_measurement_timing_budget(20000)

# Sets the VCSEL (vertical cavity surface emitting laser) pulse period for the 
# given period type (VL53L0X::VcselPeriodPreRange or VL53L0X::VcselPeriodFinalRange) 
# to the given value (in PCLKs). Longer periods increase the potential range of the sensor. 
# Valid values are (even numbers only):

# tof.set_Vcsel_pulse_period(tof.vcsel_period_type[0], 18)
tof.set_Vcsel_pulse_period(tof.vcsel_period_type[0], 12)

# tof.set_Vcsel_pulse_period(tof.vcsel_period_type[1], 14)
tof.set_Vcsel_pulse_period(tof.vcsel_period_type[1], 6)

# set range for measuring distance from 0 to 400 mm (or even further)
range = 350

# Take care! the value from the vl5310x needed callibration, it had an offset of 50 mm
# this seems to be varying per individual sensor, the first needed 50, the second I used needs a correction of 69
callibration = 69

mid_pos = 120  # the mid position of the analog value of the mod wheel
dac1_val = mid_pos

# -------------------------------------------------------------------------------------------------------------------------------------
# ------------------------------------Main program while loop--------------------------------------------------------------------------
# ------------------------------------------------------------------------------------------------------------------------------------- 


while True:
  # Start ranging, read the distance and subtract callibration value to get correct physical distance
  distance=tof.ping()-callibration
  

  # print(distance)
  
  # if the sensor sees no object it will give the value of 8140
  # for the Pitch bend this means the wheel is not touched and should output an analog value of 0 Volt (circuit diagram VC PB)
  # due to tolerances in the analog circuit it needs a bit of callibration, in my case dac_val = 120 (iso 127) is closest to 0 V 
  # PBval= 8192 is the "middle of the PB wheel" this is not further used

  

   
  if distance==8121:      # no distance measured, no hand above sensor, TAKE CARE also this value can change per sensor, measure and correct it!
    
    if dac1_val > mid_pos: # so everytime the program passes it lowers of raises the Pitchbend value, like a nutural moving back of the wheel. 
      dac1_val -= 1        # we do this in steps of 1, stepsize of "1" defines the speed.
    else:
      dac1_val = mid_pos

    if dac1_val < mid_pos:
      dac1_val += 1
    else:
      dac1_val = mid_pos
    dac1.write(dac1_val)
    
  else:
     dac_val=int((abs(distance)/range)*255) & 0xFF
     dac1.write(dac_val) # write output to create analog output on DAC1 pin26
     dac2.write(dac_val) # write output to create analog output on DAC2 pin25
     dac1_val = dac_val  # we need this variable to move to mid or zero of analog out when hand is gone (Pitchbend wheel moves to middle position) 
 
  

  PBval = int((abs(distance)/range)*16383) # calulate the BitchBend value from the distance, twice a 7 bits value see below as well.
  if PBval>16383:
    PBval=16383
    

  
  MODrange=int((abs(distance)/range)*127) # calculate and scale MODrange from distance to full range
  
  # if key3 is True, it will generate notes with a value equal to the MODrange (0-127)

  note=MODrange 
  
  
  # if hand is within the measuring range the green LED will be on
  if MODrange>127: 
    led.value(0) 
    MODrange = 127
    trigger.value(1)
    output_enabled = False    # stop MIDI clock
    if seq_runs == True: 
      uart.write(bytes([0xFC])) # Stop sequencer by MIDI
      enable_clk(1)             # disable analog clock out
      seq_runs = False
      
  else:
    led.value(1)
    trigger.value(0)          # generate an external trigger 5V->gnd
    if bpm != 250 : output_enabled = True     # start MIDI clock, 250 = no midi clock out
    if seq_runs == False: 
      uart.write(bytes([0xFA])) # Start sequencer
      enable_clk(0)             # enable analog clock out
      seq_runs = True           # store state sequencer
    # not using elif because multiple keys can be on
    if key1==True:
      uart.write(bytes([0xB0 | channel, cc1, MODrange])) # send CC (#1 would be the Mod wheel) on midi channel with distancevalue expressed in MODrange
    if key1 and key5 == True:
      cc1=MODrange
      sleep(0.1) # slowing down the loop a bit to enable to choose the right value
    if key2==True:
      uart.write(bytes([0xB0 | channel, cc2, MODrange])) # send CC #7 (= Volume) on midi channel with distancevalue expressed in MODrange
    if key2 and key5 == True:
      cc2=MODrange
      sleep(0.1) # slowing down the loop a bit to enable to choose the right value
    if key3==True:

      uart.write(bytes([0x90 | channel, note, volume])) # send note on on midi channel based on distancevalue
    if key3 and key5 == True:
      bpm=(dac_val+1)                # if key5 is on -> edit bpm with range this will give a range from 40 to 250 BPM 
      if bpm <= 20 : bpm = 20         
      if bpm >= 250: bpm = 250        # 250 is the value where midi-clock is disabled
      sleep(0.1) # slowing down the loop a bit to enable to choose the right value
    if key4==True:
      value = BigMidiInteger(PBval)
      uart.write(bytes([0xE0 | channel, value.lsb, value.msb])) # send Pitchbend message 0xE0 on midi channel 1 with 2 times 7 bits PB value
    if key4 and key5 == True:
      channel = int(dac_val/16)
      sleep(0.1) # slowing down the loop a bit to enable to choose the right value
    if key5==True:
      display_parameters(bpm, channel, cc1, cc2)
      data_saved = False
      #sleep(0.1) # slowing down the loop a bit to enable to choose the right value
    if (key5==False and data_saved==False):
      save_parameters(bpm, channel, cc1, cc2)     # when all leds are off, the 4 main parameters are saved in non volatile memory
      tmp = int(bpm/2.5)
      interval = int((1/tmp)*1000)
      tim.deinit()
      tim.init(period=interval, mode=machine.Timer.PERIODIC, callback=lambda t: tick())# Recalculate the MIDI clock ISR new interval based on new BPM 
      data_saved = True
    #if (key1 | key2 | key3 | key4) == False:

     #uart.write(bytes([0xB0 | channel, 123, 0])) # when all switched off: send all notes off message when beam un-interrupted, to avoid chaos 
 
  
# For debugging, check distance measured with no hand above sensor and correct above (in my case 8121) 
  #print('dac1_val:', dac1_val, 'Mid_pos:', mid_pos)
  #print( key1, key2, key3, key4, key5)
  #print('Distance:', distance, 'PBval:', PBval, 'MODrange:', MODrange) 


# See information on MIDI messages from the offcial MIDI association:
# https://www.midi.org/specifications-old/item/table-1-summary-of-midi-message


# Channel Voice Messages:
# Control Change:  0xB0 = 1011nnnn, [nnnn = 0-15 (MIDI Channel Number 1-16)] so [0xB0 | nnnn] will generate the correct first byte
# Pitch Bend :     0xE0 = 1110nnnn, followed by the 2 7bits values.

# MIDI CC List – Most Common Parameters:



 #1 = Modulation wheel
 #2 = Breath Control

 #7 = Volume
 #10 = Pan
 #11 = Expression
 #64 = Sustain Pedal (on/off)
 #65 = Portamento (on/off)
 #71 = Resonance (filter)
 #74 = Frequency Cutoff (filter)





















