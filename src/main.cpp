#include <Arduino.h>

/*
  Arduino LSM9DS1 - Simple Magnetometer
  Extended with library V2.0 function calls

  This example reads the magnetic field values from the LSM9DS1
  sensor and continuously prints them to the Serial Monitor
  or Serial Plotter.

  The circuit:
  - Arduino Nano 33 BLE Sense
  - or Arduino connected to LSM9DS1 breakout board

  created 10 Jul 2019
  by Riccardo Rizzo

  Modified by Femme Verbeek 
  10 July 2020

  This example code is in the public domain.
*/

#include <ros.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_broadcaster.h>
#include <Arduino_LSM9DS1.h>
#include <sensor_msgs/Imu.h>
#include <ArduinoBLE.h>

//----------------------------------------------------------------------------------------------------------------------
// BLE UUIDs
//----------------------------------------------------------------------------------------------------------------------

#define BLE_UUID_LOCATION_AND_NAVIGATION_SERVICE  "1819"
#define BLE_UUID_ROLL                             "6a21e139-0a8f-4c4b-9c1e-4c2bdf28e10b"
#define BLE_UUID_PITCH                            "9ebff1d3-de91-487a-ab47-2d88cace03ab"
#define BLE_UUID_YAW                              "650d828d-b771-44d9-9efd-872f3daeab01"

//----------------------------------------------------------------------------------------------------------------------
// BLE
//----------------------------------------------------------------------------------------------------------------------

#define BLE_DEVICE_NAME                           "Arduino Nano 33 BLE"
#define BLE_LOCAL_NAME                            "Arduino Nano 33 BLE"

BLEService IMU_Service( BLE_UUID_LOCATION_AND_NAVIGATION_SERVICE );
BLEShortCharacteristic RollCharacteristic( BLE_UUID_ROLL, BLERead | BLENotify );
BLEShortCharacteristic PitchCharacteristic( BLE_UUID_PITCH, BLERead | BLENotify );
BLEUnsignedShortCharacteristic YawCharacteristic( BLE_UUID_YAW, BLERead | BLENotify );

#define BLE_LED_PIN                               LED_BUILTIN

const unsigned long WDT = 20;              // Watch Dog Timer max time seconds.

boolean viewInSerialPlotter=false;      // true optimises for serial plotter, false for serial monitor

int     Mag_x_offset = -28,      Mag_y_offset = -3,     Mag_z_offset = +30;   // Hard-iron offsets

ros::NodeHandle nh;
geometry_msgs::TransformStamped t;
geometry_msgs::Vector3 orient;
tf::TransformBroadcaster broadcaster;
sensor_msgs::Imu imu_msg;

ros::Publisher imu_pub("imu_data", &orient);
ros::Publisher imu_msg_pub("imu/data_raw", &imu_msg);

char frameid[] = "/base_link";
char child[] = "/imu_frame";

float accl_scale, gyro_scale;
float yawDEG, pitchDEG, rollDEG;

const uint32_t BLE_UPDATE_INTERVAL = 100;
uint32_t previousMillis = 0;
uint32_t currentMillis = 0;

struct Quaternion
{
    double w, x, y, z;
};
Quaternion myQuaternion;

Quaternion ToQuaternion(double roll, double pitch, double yaw) // roll (x), pitch (y), yaw (z), angles are in radians
{
    // Abbreviations for the various angular functions

    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);

    Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}

void raw_N_Accel(uint16_t N, float& averX, float& averY, float& averZ) 
{    float x, y, z;
     int j = 1;
     averX=0; averY =0;averZ =0;
     for (int i=1;i<=N;i++)
     {  if (IMU.accelAvailable())
        {
          if (IMU.readRawAccel(x, y, z))
          {
            averX += x;    averY += y;     averZ += z;
            j = j+1;
          }
          else
          {
            Serial.println("readRawAccel failed");
            IMU.resetAccel();
            IMU.setAccelFS(3);           
            IMU.setAccelODR(5);           // 
            // IMU.setAccelOffset(0, 0, 0);  //   uncalibrated
            // IMU.setAccelSlope (1, 1, 1);  //   uncalibrated
          /***********************************************************************************************************************************
          *******  FS  Full Scale         range 0:±2g | 1:±24g | 2: ±4g | 3: ±8g  (default=2)                                           ******
          *******  ODR Output Data Rate   range 0:off | 1:10Hz | 2:50Hz | 3:119Hz | 4:238Hz | 5:476Hz, (default=3)(not working 6:952Hz) ******
          ************************************************************************************************************************************/
            IMU.accelUnit=  GRAVITY;    // or  METERPERSECOND2    
          }
          
        }
     } 
     averX /= j;    averY /= j;     averZ /= j;
     // digitalWrite(LED_BUILTIN,0);                          // led off
}

void bleTask()
{
  BLEDevice central = BLE.central();
  
  if (central.connected())
  {
    int16_t BLE_Roll = round(rollDEG);
    RollCharacteristic.writeValue( BLE_Roll );
    delay(5);
    int16_t BLE_Pitch = round(pitchDEG);
    PitchCharacteristic.writeValue(BLE_Pitch);
    delay(5);
    uint16_t BLE_Yaw = round((int)(-yawDEG + 360) % 360);
    YawCharacteristic.writeValue(BLE_Yaw);
    delay(5);
  }
}


void blePeripheralConnectHandler( BLEDevice central )
{
  digitalWrite( BLE_LED_PIN, HIGH );
  Serial.print( F ( "Connected to central: " ) );
  Serial.println( central.address() );
}


void blePeripheralDisconnectHandler( BLEDevice central )
{
  digitalWrite( BLE_LED_PIN, LOW );
  Serial.print( F( "Disconnected from central: " ) );
  Serial.println( central.address() );
}

bool setupBleMode()
{
  if ( !BLE.begin() )
  {
    return false;
  }

  // set advertised local name and service UUID
  BLE.setDeviceName( BLE_DEVICE_NAME );
  BLE.setLocalName( BLE_LOCAL_NAME );
  BLE.setAdvertisedService( IMU_Service );

  // BLE add characteristics
  IMU_Service.addCharacteristic( RollCharacteristic );
  IMU_Service.addCharacteristic( PitchCharacteristic );
  IMU_Service.addCharacteristic( YawCharacteristic );

  // add service
  BLE.addService( IMU_Service );

  // set the initial value for the characeristic
  RollCharacteristic.writeValue( 0 );
  PitchCharacteristic.writeValue( 0 );
  YawCharacteristic.writeValue( 0 );

  // set BLE event handlers
  BLE.setEventHandler( BLEConnected, blePeripheralConnectHandler );
  BLE.setEventHandler( BLEDisconnected, blePeripheralDisconnectHandler );

  // start advertising
  BLE.advertise();

  return true;
}

void setup() 
{ 
  Serial.begin(115200);

  //Configure WDT.
  NRF_WDT->CONFIG         = 0x01;     // Configure WDT to run when CPU is asleep
  NRF_WDT->CRV            = WDT*32769;    // CRV = timeout * 32768 + 1
  NRF_WDT->RREN           = 0x01;     // Enable the RR[0] reload register
  NRF_WDT->TASKS_START    = 1;        // Start WDT       

  // Initialize ROS communication
  nh.initNode();
  broadcaster.init(nh);
  nh.advertise(imu_pub);
  nh.advertise(imu_msg_pub);


  if (!IMU.begin()) 
  {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

/*******************    For an improved accuracy run the DIY_Calibration_Accelerometer sketch first.     ****************
********************         Copy/Replace the lines below by the code output of the program              ****************/
   IMU.setAccelFS(3);           
   IMU.setAccelODR(5);           // 
   IMU.setAccelOffset(0, 0, 0);  //   uncalibrated
   IMU.setAccelSlope (1, 1, 1);  //   uncalibrated
/***********************************************************************************************************************************
*******  FS  Full Scale         range 0:±2g | 1:±24g | 2: ±4g | 3: ±8g  (default=2)                                           ******
*******  ODR Output Data Rate   range 0:off | 1:10Hz | 2:50Hz | 3:119Hz | 4:238Hz | 5:476Hz, (default=3)(not working 6:952Hz) ******
************************************************************************************************************************************/
   IMU.accelUnit=  GRAVITY;    // or  METERPERSECOND2    
/*****************   For a proper functioning of the magnetometer it needs to be calibrated            ********************
*****************   Replace the lines below by the output of the DIY_Calibration_Magnetometer sketch   ********************/
   IMU.setMagnetFS(0);  
   IMU.setMagnetODR(8); 
   IMU.setMagnetOffset(28,3,0);  //  uncalibrated
   IMU.setMagnetSlope (1,1,1);  //  uncalibrated

   IMU.setMagnetFS(0);
   IMU.setMagnetODR(8);
   IMU.setMagnetOffset(29.921265, 1.913452, 10.876465);
   IMU.setMagnetSlope (1.189762, 1.136859, 1.187317);
/******************************************************************************************************************************     
****  FS  Full Scale        range (0=±400 | 1=±800 | 2=±1200 | 3=±1600  (µT)                                              *****     
****  ODR Output Data Rate  range (6,7,8)=(40,80,400)Hz | not available on all chips (0..5): (0.625,1.25,2.5,5.0,10,20)Hz *****
*******************************************************************************************************************************/     
   IMU.magnetUnit = MICROTESLA;  //   GAUSS   MICROTESLA   NANOTESLA

   if (!viewInSerialPlotter)
   {  Serial.println("Magnetic Field in µT");
      Serial.print("Magnetometer Full Scale = ±");
      Serial.print(IMU.getMagnetFS());
      Serial.println ("µT");
      Serial.print("Magnetic field sample rate = ");
      Serial.print(IMU.getMagnetODR());        // alias IMU.magneticFieldSampleRate in library version 1.01
      Serial.println(" Hz");
      delay(4000);
   }
   Serial.println("X \t Y\t Z");

  // The slowest ODR determines the sensor rate, Accel and Gyro share their ODR
  // float sensorRate = min(IMU.getGyroODR(),IMU.getMagnetODR());

  if ( !setupBleMode() )
  {
    Serial.println( "Failed to initialize BLE!" );
    while ( 1 );
  }
  else
  {
    Serial.println( "BLE initialized. Waiting for clients to connect." );
  }
}

void loop() {
  // Reload the WDTs RR[0] reload register
  NRF_WDT->RR[0] = WDT_RR_RR_Reload;

  float mx, my, mz;
  float m_hx, m_hy;
  float ax, ay, az;
  float yaw = 0.0;
  float pitch, roll;
  float r;
  int tb = 0;

  Serial.print(previousMillis);
  Serial.print("    ");

  if (IMU.magnetAvailable())                   // alias IMU.magneticFieldAvailable in library version 1.01
  { 
    if (!IMU.readMagnet(mx, my, mz))          // alias IMU.readMagneticField in library version 1.01
    {
      Serial.println("readMagnet hangs");
    }
    

    raw_N_Accel(50 ,ax, ay, az);
    
    r = sqrt(ax*ax+ay*ay+az*az);
    pitch  = asin(ax/r);
    roll   = -asin(ay/r);

    m_hx = -mx*cos(pitch) + my*sin(roll)*sin(pitch) - mz*cos(roll)*sin(pitch);
    m_hy = my*cos(roll) + mz*sin(roll);

    if (m_hx!=0) 
    {
      yaw = atan2(m_hy, m_hx);
      tb = (int)(-yaw * 180/PI + 360) % 360;
    }

    Serial.print(mx);
    Serial.print('\t');
    Serial.print(my);
    Serial.print('\t');
    Serial.print(mz);
    Serial.print('\t');

    Serial.print(ax);
    Serial.print('\t');
    Serial.print(ay);
    Serial.print('\t');
    Serial.print(az);

    Serial.print("     ");
    Serial.print(tb);
    Serial.print("     ");
    Serial.print(pitch * RAD_TO_DEG);
    Serial.print("     ");
    Serial.println(roll * RAD_TO_DEG);

    rollDEG = roll * RAD_TO_DEG;
    pitchDEG = pitch * RAD_TO_DEG;
    yawDEG = yaw * RAD_TO_DEG;

    orient.x = rollDEG;
    orient.y = pitchDEG;
    orient.z = yawDEG;

    myQuaternion = ToQuaternion( roll, -pitch, yaw );  // Create this quaternion from roll/pitch/yaw (in radians)
    
    t.header.frame_id = frameid;
    t.child_frame_id = child;
    t.transform.translation.x = 1.0;
    t.transform.rotation.x = myQuaternion.x;
    t.transform.rotation.y = myQuaternion.y;
    t.transform.rotation.z = myQuaternion.z;
    t.transform.rotation.w = myQuaternion.w;
    t.header.stamp = nh.now();
    

    imu_msg.header.stamp = nh.now();
    imu_msg.header.frame_id = child;

    // angular velocities from gyroscope
    imu_msg.angular_velocity.x = 0;
    imu_msg.angular_velocity.y = 0;
    imu_msg.angular_velocity.z = 0;

    // linear accelerations from accelerometer
    imu_msg.linear_acceleration.x = ax;
    imu_msg.linear_acceleration.y = ay;
    imu_msg.linear_acceleration.z = az;

    // orientation
    imu_msg.orientation.x = myQuaternion.x;
    imu_msg.orientation.y = myQuaternion.y;
    imu_msg.orientation.z = myQuaternion.z;
    imu_msg.orientation.w = myQuaternion.w;


    imu_msg_pub.publish(&imu_msg);
    imu_pub.publish(&orient);
    broadcaster.sendTransform(t);
    nh.spinOnce();
    delay(1);
    currentMillis = millis();
    if ( currentMillis - previousMillis >= BLE_UPDATE_INTERVAL )
    {
      previousMillis = currentMillis;
      bleTask();
    }
  }
  else
  {
    Serial.println("Magnet not available");
  }
}