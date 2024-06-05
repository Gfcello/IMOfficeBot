import rclpy
from rclpy.node import Node
import sys
sys.path.append('./external_libraries/BerryIMUv3/')

import IMU
import datetime
import math

from message_types.msg import NodeStatus, ImuData


class IMUNode(Node):
    # Node communicates with Berry IMU v3 using their library

    # Constants
    RAD_TO_DEG = 57.29578
    M_PI = 3.14159265358979323846
    G_GAIN = 0.070          # [deg/s/LSB]  If you change the dps for gyro, you need to update this value accordingly
    AA =  0.40              # Complementary filter constant
    MAG_LPF_FACTOR = 0.4    # Low pass filter constant magnetometer
    ACC_LPF_FACTOR = 0.4    # Low pass filter constant for accelerometer
    ACC_MEDIANTABLESIZE = 7         # Median filter table size for accelerometer. Higher = smoother but a longer delay
    MAG_MEDIANTABLESIZE = 7         # Median filter table size for magnetometer. Higher = smoother but a longer delay

    # Calibration values (may want to move these to params later)
    magXmin =  -1515
    magYmin =  -2932
    magZmin =  -2714
    magXmax =  2310
    magYmax =  1047
    magZmax =  999

    #Kalman filter variables
    Q_angle = 0.02
    Q_gyro = 0.0015
    R_angle = 0.005
    y_bias = 0.0
    x_bias = 0.0
    XP_00 = 0.0
    XP_01 = 0.0
    XP_10 = 0.0
    XP_11 = 0.0
    YP_00 = 0.0
    YP_01 = 0.0
    YP_10 = 0.0
    YP_11 = 0.0
    KFangleX = 0.0
    KFangleY = 0.0
    gyroXangle = 0.0
    gyroYangle = 0.0
    gyroZangle = 0.0
    CFangleX = 0.0
    CFangleY = 0.0
    CFangleXFiltered = 0.0
    CFangleYFiltered = 0.0
    kalmanX = 0.0
    kalmanY = 0.0
    oldXMagRawValue = 0
    oldYMagRawValue = 0
    oldZMagRawValue = 0
    oldXAccRawValue = 0
    oldYAccRawValue = 0
    oldZAccRawValue = 0

    def __init__(self):
        super().__init__('imu_node')
        self.data_publisher = self.create_publisher(ImuData, 'imu_node/data', 10)
        self.status_publisher = self.create_publisher(NodeStatus, 'bms_node/status', 10)
        self.status_msg = NodeStatus()

        self.init_time = datetime.datetime.now()

        #Setup the tables for the mdeian filter. Fill them all with '1' so we dont get devide by zero error
        self.acc_medianTable1X = [1] * self.ACC_MEDIANTABLESIZE
        self.acc_medianTable1Y = [1] * self.ACC_MEDIANTABLESIZE
        self.acc_medianTable1Z = [1] * self.ACC_MEDIANTABLESIZE
        self.acc_medianTable2X = [1] * self.ACC_MEDIANTABLESIZE
        self.acc_medianTable2Y = [1] * self.ACC_MEDIANTABLESIZE
        self.acc_medianTable2Z = [1] * self.ACC_MEDIANTABLESIZE
        self.mag_medianTable1X = [1] * self.MAG_MEDIANTABLESIZE
        self.mag_medianTable1Y = [1] * self.MAG_MEDIANTABLESIZE
        self.mag_medianTable1Z = [1] * self.MAG_MEDIANTABLESIZE
        self.mag_medianTable2X = [1] * self.MAG_MEDIANTABLESIZE
        self.mag_medianTable2Y = [1] * self.MAG_MEDIANTABLESIZE
        self.mag_medianTable2Z = [1] * self.MAG_MEDIANTABLESIZE

        IMU.detectIMU()     #Detect if BerryIMU is connected.
        if(IMU.BerryIMUversion == 99):
            print(" No BerryIMU found... exiting ")
            sys.exit()
        IMU.initIMU()       #Initialise the accelerometer, gyroscope and compass

        timer_period = 1.0  # seconds
        self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        #Read the accelerometer,gyroscope and magnetometer values
        ACCx = IMU.readACCx()
        ACCy = IMU.readACCy()
        ACCz = IMU.readACCz()
        GYRx = IMU.readGYRx()
        GYRy = IMU.readGYRy()
        GYRz = IMU.readGYRz()
        MAGx = IMU.readMAGx()
        MAGy = IMU.readMAGy()
        MAGz = IMU.readMAGz()


        #Apply compass calibration
        MAGx -= (self.magXmin + self.magXmax) /2
        MAGy -= (self.magYmin + self.magYmax) /2
        MAGz -= (self.magZmin + self.magZmax) /2


        ##Calculate loop Period(LP). How long between Gyro Reads
        b = datetime.datetime.now() - a
        a = datetime.datetime.now()
        LP = b.microseconds/(1000000*1.0) # Loop time


        ###############################################
        #### Apply low pass filter ####
        ###############################################
        MAGx =  MAGx  * self.MAG_LPF_FACTOR + oldXMagRawValue*(1 - self.MAG_LPF_FACTOR)
        MAGy =  MAGy  * self.MAG_LPF_FACTOR + oldYMagRawValue*(1 - self.MAG_LPF_FACTOR)
        MAGz =  MAGz  * self.MAG_LPF_FACTOR + oldZMagRawValue*(1 - self.MAG_LPF_FACTOR)
        ACCx =  ACCx  * self.ACC_LPF_FACTOR + oldXAccRawValue*(1 - self.ACC_LPF_FACTOR)
        ACCy =  ACCy  * self.ACC_LPF_FACTOR + oldYAccRawValue*(1 - self.ACC_LPF_FACTOR)
        ACCz =  ACCz  * self.ACC_LPF_FACTOR + oldZAccRawValue*(1 - self.ACC_LPF_FACTOR)

        oldXMagRawValue = MAGx
        oldYMagRawValue = MAGy
        oldZMagRawValue = MAGz
        oldXAccRawValue = ACCx
        oldYAccRawValue = ACCy
        oldZAccRawValue = ACCz

        #########################################
        #### Median filter for accelerometer ####
        #########################################
        # cycle the table
        for x in range (self.ACC_MEDIANTABLESIZE-1,0,-1 ):
            self.acc_medianTable1X[x] = self.acc_medianTable1X[x-1]
            self.acc_medianTable1Y[x] = self.acc_medianTable1Y[x-1]
            self.acc_medianTable1Z[x] = self.acc_medianTable1Z[x-1]

        # Insert the lates values
        self.acc_medianTable1X[0] = ACCx
        self.acc_medianTable1Y[0] = ACCy
        self.acc_medianTable1Z[0] = ACCz

        # Copy the tables
        self.acc_medianTable2X = self.acc_medianTable1X[:]
        self.acc_medianTable2Y = self.acc_medianTable1Y[:]
        self.acc_medianTable2Z = self.acc_medianTable1Z[:]

        # Sort table 2
        self.acc_medianTable2X.sort()
        self.acc_medianTable2Y.sort()
        self.acc_medianTable2Z.sort()

        # The middle value is the value we are interested in
        ACCx = self.acc_medianTable2X[int(self.ACC_MEDIANTABLESIZE/2)]
        ACCy = self.acc_medianTable2Y[int(self.ACC_MEDIANTABLESIZE/2)]
        ACCz = self.acc_medianTable2Z[int(self.ACC_MEDIANTABLESIZE/2)]



        #########################################
        #### Median filter for magnetometer ####
        #########################################
        # cycle the table
        for x in range (self.MAG_MEDIANTABLESIZE-1,0,-1 ):
            self.mag_medianTable1X[x] = self.mag_medianTable1X[x-1]
            self.mag_medianTable1Y[x] = self.mag_medianTable1Y[x-1]
            self.mag_medianTable1Z[x] = self.mag_medianTable1Z[x-1]

        # Insert the latest values
        self.mag_medianTable1X[0] = MAGx
        self.mag_medianTable1Y[0] = MAGy
        self.mag_medianTable1Z[0] = MAGz

        # Copy the tables
        self.mag_medianTable2X = self.mag_medianTable1X[:]
        self.mag_medianTable2Y = self.mag_medianTable1Y[:]
        self.mag_medianTable2Z = self.mag_medianTable1Z[:]

        # Sort table 2
        self.mag_medianTable2X.sort()
        self.mag_medianTable2Y.sort()
        self.mag_medianTable2Z.sort()

        # The middle value is the value we are interested in
        MAGx = self.mag_medianTable2X[int(self.MAG_MEDIANTABLESIZE/2)]
        MAGy = self.mag_medianTable2Y[int(self.MAG_MEDIANTABLESIZE/2)]
        MAGz = self.mag_medianTable2Z[int(self.MAG_MEDIANTABLESIZE/2)]



        #Convert Gyro raw to degrees per second
        rate_gyr_x =  GYRx * self.G_GAIN
        rate_gyr_y =  GYRy * self.G_GAIN
        rate_gyr_z =  GYRz * self.G_GAIN


        #Calculate the angles from the gyro.
        self.gyroXangle+=rate_gyr_x*LP
        self.gyroYangle+=rate_gyr_y*LP
        self.gyroZangle+=rate_gyr_z*LP

        #Convert Accelerometer values to degrees
        AccXangle =  (math.atan2(ACCy,ACCz)*self.RAD_TO_DEG)
        AccYangle =  (math.atan2(ACCz,ACCx)+self.M_PI)*self.RAD_TO_DEG


        #Change the rotation value of the accelerometer to -/+ 180 and
        #move the Y axis '0' point to up.  This makes it easier to read.
        if AccYangle > 90:
            AccYangle -= 270.0
        else:
            AccYangle += 90.0

        #Complementary filter used to combine the accelerometer and gyro values.
        CFangleX=self.AA*(CFangleX+rate_gyr_x*LP) +(1 - self.AA) * AccXangle
        CFangleY=self.AA*(CFangleY+rate_gyr_y*LP) +(1 - self.AA) * AccYangle

        #Kalman filter used to combine the accelerometer and gyro values.
        kalmanY = self.kalmanFilterY(AccYangle, rate_gyr_y,LP)
        kalmanX = self.kalmanFilterX(AccXangle, rate_gyr_x,LP)

        #Calculate heading
        heading = 180 * math.atan2(MAGy,MAGx)/self.M_PI

        #Only have our heading between 0 and 360
        if heading < 0:
            heading += 360

        ####################################################################
        ###################Tilt compensated heading#########################
        ####################################################################
        #Normalize accelerometer raw values.
        accXnorm = ACCx/math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)
        accYnorm = ACCy/math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)


        #Calculate pitch and roll
        pitch = math.asin(accXnorm)
        roll = -math.asin(accYnorm/math.cos(pitch))


        #Calculate the new tilt compensated values
        #The compass and accelerometer are orientated differently on the the BerryIMUv1, v2 and v3.
        #This needs to be taken into consideration when performing the calculations

        #X compensation
        if(IMU.BerryIMUversion == 1 or IMU.BerryIMUversion == 3):            #LSM9DS0 and (LSM6DSL & LIS2MDL)
            magXcomp = MAGx*math.cos(pitch)+MAGz*math.sin(pitch)
        else:                                                                #LSM9DS1
            magXcomp = MAGx*math.cos(pitch)-MAGz*math.sin(pitch)

        #Y compensation
        if(IMU.BerryIMUversion == 1 or IMU.BerryIMUversion == 3):            #LSM9DS0 and (LSM6DSL & LIS2MDL)
            magYcomp = MAGx*math.sin(roll)*math.sin(pitch)+MAGy*math.cos(roll)-MAGz*math.sin(roll)*math.cos(pitch)
        else:                                                                #LSM9DS1
            magYcomp = MAGx*math.sin(roll)*math.sin(pitch)+MAGy*math.cos(roll)+MAGz*math.sin(roll)*math.cos(pitch)


        #Calculate tilt compensated heading
        tiltCompensatedHeading = 180 * math.atan2(magYcomp,magXcomp)/self.M_PI

        if tiltCompensatedHeading < 0:
            tiltCompensatedHeading += 360
        
        imu_msg = ImuData()
        imu_msg.heading = tiltCompensatedHeading
        imu_msg.ang_vel = rate_gyr_z
        self.data_publisher.publish(imu_msg)

    def kalmanFilterY (self, accAngle, gyroRate, DT):
        y=0.0
        S=0.0

        KFangleY = KFangleY + DT * (gyroRate - self.y_bias)

        self.YP_00 = self.YP_00 + ( - DT * (self.YP_10 + self.YP_01) + self.Q_angle * DT )
        self.YP_01 = self.YP_01 + ( - DT * self.YP_11 )
        self.YP_10 = self.YP_10 + ( - DT * self.YP_11 )
        self.YP_11 = self.YP_11 + ( + self.Q_gyro * DT )

        y = accAngle - KFangleY
        S = self.YP_00 + self.R_angle
        K_0 = self.YP_00 / S
        K_1 = self.YP_10 / S

        KFangleY = KFangleY + ( K_0 * y )
        self.y_bias = self.y_bias + ( K_1 * y )

        self.YP_00 = self.YP_00 - ( K_0 * self.YP_00 )
        self.YP_01 = self.YP_01 - ( K_0 * self.YP_01 )
        self.YP_10 = self.YP_10 - ( K_1 * self.YP_00 )
        self.YP_11 = self.YP_11 - ( K_1 * self.YP_01 )

        return KFangleY

    def kalmanFilterX (self, accAngle, gyroRate, DT):
        x=0.0
        S=0.0


        KFangleX = KFangleX + DT * (gyroRate - self.x_bias)

        self.XP_00 = self.XP_00 + ( - DT * (self.XP_10 + self.XP_01) + self.Q_angle * DT )
        self.XP_01 = self.XP_01 + ( - DT * self.XP_11 )
        self.XP_10 = self.XP_10 + ( - DT * self.XP_11 )
        self.XP_11 = self.XP_11 + ( + self.Q_gyro * DT )

        x = accAngle - KFangleX
        S = self.XP_00 + self.R_angle
        K_0 = self.XP_00 / S
        K_1 = self.XP_10 / S

        KFangleX = KFangleX + ( K_0 * x )
        self.x_bias = self.x_bias + ( K_1 * x )

        self.XP_00 = self.XP_00 - ( K_0 * self.XP_00 )
        self.XP_01 = self.XP_01 - ( K_0 * self.XP_01 )
        self.XP_10 = self.XP_10 - ( K_1 * self.XP_00 )
        self.XP_11 = self.XP_11 - ( K_1 * self.XP_01 )

        return KFangleX


def main(args=None):
    rclpy.init(args=args)

    imu_node = IMUNode()

    rclpy.spin(imu_node)


if __name__ == '__main__':
    main()
