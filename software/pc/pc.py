import serial
import sys
BAUDRATE = 115200
PARITY = False

#Configures serial port
def configure_serial(serial_port):
    return serial.Serial(
        port=serial_port,
        baudrate=BAUDRATE,
        parity=serial.PARITY_EVEN if PARITY else serial.PARITY_NONE,
        stopbits=serial.STOPBITS_TWO,
        bytesize=serial.EIGHTBITS,
    )

def u16_to_chars(u16):
    """uint16_t to two bytes"""
    if u16>2**16 or u16<0:
        raise ValueError
    a,b = u16>>8,u16&(~(0xff<<8))
    return chr(a),chr(b)

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
    if len(settings) != 10:
        raise ValueError("set_profile needs 10 values")
    u16s = map(u16_to_chars,settings)
    #Flatten list
    u16s = [item for sublist in u16s for item in sublist]
    u16s = ''.join(u16s)
    print map(ord,u16s)
    ser.write('!W'+u16s)

if __name__ == "__main__":

    if len(sys.argv)<2:
        print "Give serial port address as a command line argument."
        exit()
    try:
        ser = configure_serial(sys.argv[1])
        if not ser.isOpen():
            raise Exception
    except:
        print 'Opening serial port {} failed.'.format(sys.argv[1])
        raise
        exit()

    while True:
        try:
            try:
                x = ser.readline()
            except serial.serialutil.SerialException:
                continue
            except OSError:
                continue
            print x
        except KeyboardInterrupt:
            ser.close()
            break
