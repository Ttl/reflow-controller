import serial
import sys
import argparse
import ast
import time
from collections import deque

import matplotlib.pyplot as plt
import matplotlib.animation as animation

BAUDRATE = 115200
PARITY = False
PWM_MAX = 6250

class AnalogPlot:
    def __init__(self, ser, maxlen):
        self.x = deque([0]*maxlen, maxlen)
        self.y1 = deque([0]*maxlen, maxlen)
        self.y2 = deque([0]*maxlen, maxlen)
        self.maxlen = maxlen
        self.starttime = time.time()
        self.ser = ser

    def add(self, status):
        #Only add datapoint when it's possible to add in
        #both plots
        if 'temp' in status and 'target' in status:
            self.x.appendleft(time.time() - self.starttime)
            self.y1.appendleft(status['temp'])
            self.y2.appendleft(status['target'])

    def update(self, frameNum, p0, p1, ax):
        try:
            status = get_status(self.ser)
            self.add(status)
            print_status(status)
        except TypeError:
            pass
        ax.set_xlim(self.x[-1], max(10,self.x[0]))
        ax.set_ylim((0, max(250,max(self.y2),max(self.y1))))
        p0.set_data(self.x, self.y1)
        p1.set_data(self.x, self.y2)
        return (p0,p1,ax)

#Configures serial port
def configure_serial(serial_port):
    return serial.Serial(
        port=serial_port,
        baudrate=BAUDRATE,
        parity=serial.PARITY_EVEN if PARITY else serial.PARITY_NONE,
        stopbits=serial.STOPBITS_TWO,
        bytesize=serial.EIGHTBITS,
        timeout=0.05
    )

def u16_to_chars(u16):
    """uint16_t to two bytes"""
    if u16>2**16 or u16<0:
        raise ValueError
    a,b = u16>>8,u16&(~(0xff<<8))
    return chr(a),chr(b)

def start_reflow(ser):
    ser.write('!S')

def stop_reflow(ser):
    ser.write('!H')

def pid_debug(ser, enable=True):
    if enable:
        ser.write('!D')
    else:
        ser.write('!d')

def get_profile(ser):
    ser.write('!O')
    while True:
        x = ser.readline()
        if '!' in x:
            break
    pos = x.index('!')+1
    settings = list(ast.literal_eval(x[pos:]))
    return settings

def print_profile(settings):
    for i in (0,1,2,4,6):
        settings[i] /=4.
    print "Start rate",settings[0],'C/s'
    print "Soak temp start",settings[1],'C'
    print "Soak temp end",settings[2],'C'
    print "Soak length",settings[3],'s'
    print "Peak temp",settings[4],'C'
    print "Time to peak",settings[5],'s'
    print "Cool rate",settings[6],'C/s'
    print "PID P",settings[7]
    print "PID I",settings[8]
    print "PID D",settings[9]

def write_profile(ser, settings):
    """Writes a correctly formatted list of settings to device"""
    u16s = map(u16_to_chars,settings)
    #Flatten the list
    u16s = [item for sublist in u16s for item in sublist]
    u16s = ''.join(u16s)
    ser.write('!W'+u16s)

def set_pid(ser, settings):
    if len(settings) == 3:
        current = get_profile(ser)[:7]
        current.extend(settings)
        settings = current
    else:
        raise ValueError("set_pid needs 3 values")
    settings = map(int,settings)
    write_profile(ser, settings)
    print "New profile:"
    print_profile(get_profile(ser))

def set_profile(ser,settings):
	#uint16_t start_rate;
	#uint16_t soak_temp1;
	#uint16_t soak_temp2;
	#uint16_t soak_length;
	#uint16_t peak_temp;
	#uint16_t time_to_peak;
	#uint16_t cool_rate;
	#uint16_t pid_p;
	#uint16_t pid_i;
	#uint16_t pid_d;
    if len(settings) == 7:
        current = get_profile(ser)
        settings.extend(current[7:])
        print "Using previous PID settings"
    if len(settings) != 10:
        raise ValueError("set_profile needs 10 values")
    #Firmware needs units in quarter celsius for temperatures
    for i in (0,1,2,4,6):
        settings[i] *=4
    settings = map(int,settings)
    write_profile(ser, settings)
    print "New profile:"
    print_profile(get_profile(ser))

def get_status(ser):
    states = ['Stop','Preheat','Soak','Peak','Cool']
    s = ser.readline()
    s = s.strip()
    s = s.replace(':',',')
    status = s.split(',')
    if len(status) == 10:
        try:
            temp = float(status[1])/4
            room = float(status[3])/4
            target = float(status[5])/4
            pwm = float(status[7])/PWM_MAX
            state = states[int(status[9])]
            return {'temp':temp, 'room':room, 'target':target, 'pwm':pwm, 'state':state}
        except:
            pass
    return None

def print_status(status):
    try:
        print 'Temp: {temp:3.2f}, Room: {room:3.2f}, Target: {target:3.2f}, PWM: {pwm:1.2f}, State: {state}'.format(**status)
    except (TypeError, KeyError):
        print

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Reflow oven controller program")
    parser.add_argument('-s','--set', nargs=7, metavar=('start_rate', 'soak_temp_start',
        'soak_temp_end', 'soak_length', 'peak_temp', 'time_to_peak', 'cool_rate'), type=float, help='Set temperature profile')
    parser.add_argument('-p','--set_pid', nargs=3, metavar=('P','I','D'), type=float, help='Set PID coefficients')
    parser.add_argument('-g','--get', action='store_true', help='Get the current profile')
    parser.add_argument('-l','--plot', dest='plot', action='store_true', help="Draw plot")
    parser.add_argument('-n','--no-plot', dest='plot', action='store_false', help="Don't draw plot")
    parser.add_argument('port',metavar='Port', help='Serial port')
    parser.set_defaults(get=False, plot=True)
    args = parser.parse_args()

    if len(sys.argv)<2:
        print "Give serial port address as a command line argument."
        exit()
    try:
        ser = configure_serial(args.port)
        if not ser.isOpen():
            raise Exception
    except:
        print 'Opening serial port {} failed.'.format(args.port)
        exit(1)

    stop_reflow(ser)

    if args.set != None:
        set_profile(ser, args.set)

    if args.set_pid != None:
        set_pid(ser, args.set_pid)

    if args.get:
        print_profile(get_profile(ser))

    if args.plot:
        analogPlot = AnalogPlot(ser, 600)

        fig = plt.figure()
        ax = plt.axes(xlim=(0, 100), ylim=(0, 300))
        p0, = ax.plot([], [])
        p1, = ax.plot([], [])

        anim = animation.FuncAnimation(fig, analogPlot.update,
                                         fargs=(p0,p1,ax),
                                         interval=10)
        start_reflow(ser)
        plt.show()
        exit()

    while True:
        try:
            try:
                print_status(get_status(ser))
            except serial.serialutil.SerialException:
                continue
            except OSError:
                continue
            except KeyError:
                continue
        except KeyboardInterrupt:
            ser.close()
            break
