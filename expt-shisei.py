import spidev,time
from json import load
import RPi.GPIO as GPIO

ACCEL_RANGE = 8 #±n[g]
GYRO_RANGE = 2000 #±n[dps]
MAG_RANGE = 4912 #±n[μT]

LED_PIN = 22
BUTTON_PIN = 6

GPIO.setmode(GPIO.BCM)
GPIO.setup(LED_PIN, GPIO.OUT)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

spi = spidev.SpiDev()
count = 0 #for debug

number = input('number?:')
results_file = "/home/pi/program/results-mpu9250-{}.csv".format(number)

#load offsets
offset_file = "/home/pi/program/sensor-offsets.json"
with open(offset_file, 'r') as f:
    offsets = load(f)
OFFSET_X_ACCEL = offsets['offset_x_Accel']
OFFSET_Y_ACCEL = offsets['offset_y_Accel']
OFFSET_Z_ACCEL = offsets['offset_z_Accel']
OFFSET_X_GYRO  = offsets['offset_x_Gyro']
OFFSET_Y_GYRO  = offsets['offset_y_Gyro']
OFFSET_Z_GYRO  = offsets['offset_z_Gyro']
OFFSET_X_MAG  = offsets['offset_x_Mag']
OFFSET_Y_MAG  = offsets['offset_y_Mag']
OFFSET_Z_MAG  = offsets['offset_z_Mag']
SENSITIVITY_X_MAG = offsets['sensitivity_x_Mag']
SENSITIVITY_Y_MAG = offsets['sensitivity_y_Mag']
SENSITIVITY_Z_MAG = offsets['sensitivity_z_Mag']

def StartMpu9250(): #mpu9250 start operation
    spi.open(0, 0)
    spi.mode = 3
    spi.max_speed_hz = 500000 #1MHz max(write)
    time.sleep(0.5)

    ##MPU9250
    dev_id = spi.xfer2([0x75 +0x80,0x80]) #who am i
    if dev_id[1] != 0x71:
        print("MPU9250 may be offline! Continue in 5 seconds.")
        time.sleep(5)

    #reset
    spi.xfer2([0x6b,0x80]) #reset mpu9250
    time.sleep(0.1) #wait for all registers to reset
    spi.xfer2([0x6b,0x00]) #reset pwr mgr
    time.sleep(0.1)

    #settings
    spi.xfer2([0x6b,0x00]) #0x01:auto select clock source 0x00:internal 20Mhz oscillator
    time.sleep(0.2)
    spi.xfer2([0x1a,0x00]) #gyro and thermo config -> set 8khz
    spi.xfer2([0x19,0x00]) #sample rate -> set 1khz(if 0<slpf_cfg<7, this is needless)
    spi.xfer2([0x1b,0x00 | (3 << 0x03)]) #set gyro range(0:250,1:500,2:1000,3:2000[dps])
    spi.xfer2([0x1c,0x00 | (2 << 0x03)]) #set accel range(0:2,1:4,2:8,3:16[g])
    spi.xfer2([0x1d,0x00]) #accel config2 -> set 4khz
    spi.xfer2([0x37,0x10]) #int pin cfg
    spi.xfer2([0x38,0x01]) #enable data ready interrupt
    spi.xfer2([0x23,0x00]) #disable fifo
    time.sleep(0.1)
    spi.xfer2([0x6a,0x20]) #enable i2c master mode
    spi.xfer2([0x24,0x5d]) #delay data ready until data is loaded,i2c config stop after each transaction, master i2c bus at 400khz
    spi.xfer2([0x67,0x81]) #use blocking data retrieval and enable delay for mag sample rate mismatch
    spi.xfer2([0x34,0x01]) #delay mag data retrieval to once every other accel/gyro data sample
    time.sleep(0.1)
    #spi.xfer2([
    #    ##for magnetometer
    #    55,0, #clear int
    #    25,9, #sample rate
    #    26,2, #gyro temp fchoice
    #    29,10, #accel fchoice
    #    108,0, #enable all sensor
    #    36,77 #I2C_MST_CTRL set clock 400khz
    #])

    ##AK8963
    #spi.xfer2([0x6a,0x20]) #enable i2c master mode
    #spi.xfer2([0x24,0x5d]) #i2c clock 400khz
    spi.xfer2([0x25,0x0c | 0x80]) #AK8963 address on read mode
    spi.xfer2([0x26,0x00]) #who am i
    spi.xfer2([0x27,0x81]) #transfer 1 byte
    time.sleep(0.1)
    dev_id = spi.xfer2([0x49 +0x80,0x80])
    if dev_id[1] != 0x48:
        print("AK8963 may be offline! Continue in 5 seconds.")
        time.sleep(5)

    #reset
    spi.xfer2([0x25,0x0c]) #AK8963 address on write mode
    spi.xfer2([0x26,0x0b]) #CNTL2
    spi.xfer2([0x63,0x01]) #reset AK8963
    spi.xfer2([0x27,0x81]) #transfer 1 byte
    time.sleep(0.1)
    spi.xfer2([0x25,0x0c]) #AK8963 address on write mode
    spi.xfer2([0x26,0x0a]) #CNTL1
    spi.xfer2([0x63,0x00]) #power down magnetometer
    spi.xfer2([0x27,0x81]) #transfer 1 byte
    time.sleep(0.05)
    spi.xfer2([0x25,0x0c]) #AK8963 address on write mode
    spi.xfer2([0x26,0x0a]) #CNTL1
    spi.xfer2([0x63,0x16]) #16bit,100hz sample rate
    spi.xfer2([0x27,0x81]) #transfer 1 byte
    time.sleep(0.05)
    spi.xfer2([0x25,0x0c | 0x80]) #AK8963 address on read mode
    spi.xfer2([0x26,0x03]) #HXL
    spi.xfer2([0x27,0x87]) #transfer 7 byte
    time.sleep(0.05)


def FinishMpu9250():
    spi.xfer2([0x25,0x0c | 0x80]) #AK8963 address on read mode
    spi.xfer2([0x26,0x09]) #ST2
    spi.xfer2([0x27,0x81]) #transfer 1 byte
    time.sleep(0.1)
    spi.xfer2([0x25,0x0c]) #AK8963 address on write mode
    spi.xfer2([0x26,0x0a]) #CNTL1
    spi.xfer2([0x63,0x00]) #power down magnetometer
    spi.xfer2([0x27,0x81]) #transfer 1 byte
    time.sleep(0.05)


def ReadDataMpu9250():
    seed = [
        59 +128,60 +128, #xAccel
        61 +128,62 +128, #yAccel
        63 +128,64 +128, #zAccel
        65 +128,66 +128, #temp
        67 +128,68 +128, #xGyro
        69 +128,70 +128, #yGyro
        71 +128,72 +128, #zGyro
        73 +128,74 +128, #xMag
        75 +128,76 +128, #yMag
        77 +128,78 +128, #zMag
        79 +128, #AK8963 states
        128 #trash
        ]
    data_list = spi.xfer2(seed)
    xAccel = u2s((data_list[1] << 8  | data_list[2]))  * (ACCEL_RANGE / 0x8000) * -1 - OFFSET_X_ACCEL
    yAccel = u2s((data_list[3] << 8  | data_list[4]))  * (ACCEL_RANGE / 0x8000) * -1 - OFFSET_Y_ACCEL
    zAccel = u2s((data_list[5] << 8  | data_list[6]))  * (ACCEL_RANGE / 0x8000) * -1 - OFFSET_Z_ACCEL
    xGyro  = u2s((data_list[9] << 8  | data_list[10])) * (GYRO_RANGE  / 0x8000) - OFFSET_X_GYRO 
    yGyro  = u2s((data_list[11] << 8 | data_list[12])) * (GYRO_RANGE  / 0x8000) - OFFSET_Y_GYRO
    zGyro  = u2s((data_list[13] << 8 | data_list[14])) * (GYRO_RANGE  / 0x8000) - OFFSET_Z_GYRO 
    yMag   = u2s((data_list[16] << 8 | data_list[15])) * ((((SENSITIVITY_Y_MAG - 128) * 0.5) / 128) + 1) * (MAG_RANGE / 0x8000) - OFFSET_Y_MAG #Gyro(x,y,z)=Accel(x,y,z)=Mag(y,x,-z)
    xMag   = u2s((data_list[18] << 8 | data_list[17])) * ((((SENSITIVITY_X_MAG - 128) * 0.5) / 128) + 1) * (MAG_RANGE / 0x8000) - OFFSET_X_MAG #Gyro(x,y,z)=Accel(x,y,z)=Mag(y,x,-z)
    zMag   = u2s((data_list[20] << 8 | data_list[19])) * ((((SENSITIVITY_Z_MAG - 128) * 0.5) / 128) + 1) * (MAG_RANGE / 0x8000)
    zMag   = (zMag * -1) - OFFSET_Z_MAG #Gyro(x,y,z)=Accel(x,y,z)=Mag(y,x,-z)
    STMag  = data_list[21]

    return xAccel,yAccel,zAccel,xGyro,yGyro,zGyro,xMag,yMag,zMag,STMag


def u2s(unsigneddata): #convert unsigned to signed(only 16bit)
    if unsigneddata & (0x01 << 15) :
        return -1 * ((unsigneddata ^ 0xffff) + 1)
    return unsigneddata

try:
    StartMpu9250()
    spi.max_speed_hz = 10000000 #20MHz max(read)
    results = []
    r_append = results.append
    one_time_append1 = True
    one_time_append2 = True
    while GPIO.input(BUTTON_PIN) == GPIO.LOW:
        time.sleep(0.05)
    GPIO.output(LED_PIN, GPIO.HIGH)
    print('1')
    time.sleep(1)
    GPIO.output(LED_PIN, GPIO.LOW)
    time.sleep(0.2)
    now = time.time()
    print("start" + str(now))
    while True:
        data = ReadDataMpu9250()
        r_append("{},{},{},{},{},{},{},{},{},{},{}".format(time.time(),data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7],data[8],data[9]))
        count += 1
        if count >= 10000:
            with open(results_file,'a') as f:
                f.write("\n".join(results) + "\n")
            results.clear()
            count = 0
        elif (time.time() - now >= 5) and one_time_append1:
            GPIO.output(LED_PIN, GPIO.HIGH)
            one_time_append1 = False
            r_append('{},fall!'.format(time.time()))
        elif (time.time() - now >= 60) and one_time_append2:
            GPIO.output(LED_PIN, GPIO.LOW)
            one_time_append2 = False
            r_append('{},Turn off the accels'.format(time.time()))
        elif GPIO.input(BUTTON_PIN) == GPIO.HIGH:
            break
except KeyboardInterrupt:
    print('keyboardinterrupt!')
finally:
    FinishMpu9250()
    spi.close()
    GPIO.cleanup()
    with open(results_file,'a') as f:
        f.write("\n".join(results))