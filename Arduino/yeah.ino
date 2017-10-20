<<<<<<< HEAD
//MIT License
//
//Copyright (c) 2017 Nikolaus Floden
//
//Permission is hereby granted, free of charge, to any person obtaining a copy
//of this software and associated documentation files (the "Software"), to deal
//in the Software without restriction, including without limitation the rights
//to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//copies of the Software, and to permit persons to whom the Software is
//furnished to do so, subject to the following conditions:
//
//The above copyright notice and this permission notice shall be included in all
//copies or substantial portions of the Software.
//
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//SOFTWARE.


#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>
#include <bluefruit.h>


/* Assign a unique ID to this sensor at the same time */
Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);
Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);

float velocity = 0;
float maxvelo = 0;
float maxtilt = 0;
float distance,currAccel;
float speshdistance = 0;
float totdistance = speshdistance / 2;
float prevAccel = 0;
unsigned long prevTime = millis();
unsigned long currTime;

BLEClientDis  clientDis;
BLEClientUart clientUart;

void displaySensorDetails(void)
{
  sensor_t sensor;
  gyro.getSensor(&sensor);
  sensor_t accel, mag;
  accelmag.getSensor(&accel, &mag);
  Serial.println("------------------------------------");
  Serial.println("GYROSCOPE");
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    0x"); Serial.println(sensor.sensor_id, HEX);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" rad/s");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" rad/s");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" rad/s");
  Serial.println("------------------------------------");
  Serial.println("");
  Serial.println("------------------------------------");
  Serial.println("ACCELEROMETER");
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(accel.name);
  Serial.print  ("Driver Ver:   "); Serial.println(accel.version);
  Serial.print  ("Unique ID:    0x"); Serial.println(accel.sensor_id, HEX);
  Serial.print  ("Min Delay:    "); Serial.print(accel.min_delay); Serial.println(" s");
  Serial.print  ("Max Value:    "); Serial.print(accel.max_value, 4); Serial.println(" m/s^2");
  Serial.print  ("Min Value:    "); Serial.print(accel.min_value, 4); Serial.println(" m/s^2");
  Serial.print  ("Resolution:   "); Serial.print(accel.resolution, 8); Serial.println(" m/s^2");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(1000);
}

void setup(void)
{
  Serial.begin(115200);

  /* Wait for the Serial Monitor */
  while (!Serial) {
    delay(1);
  }

  Serial.println("Gyroscope Test"); Serial.println("");

  /* Initialise the sensor */
  if(!gyro.begin())
  {
    /* There was a problem detecting the FXAS21002C ... check your connections */
    Serial.println("Ooops, no FXAS21002C detected ... Check your wiring!");
    while(1);
  }else if (!accelmag.begin(ACCEL_RANGE_4G))
  {
    Serial.println("Oooops, no FXOS8700 detected ... CHEK YO WIRING!");
  }

  /* Display some basic information on this sensor */
  displaySensorDetails();
  
  Serial.println("Bluefruit52 Central BLEUART Example");
  Serial.println("-----------------------------------\n");
  
  // Enable both peripheral and central
  Bluefruit.begin(true, true);
  Bluefruit.setName("Bluefruit52 Central");

  // Configure DIS client
  clientDis.begin();

  // Init BLE Central Uart Serivce
  clientUart.begin();
  clientUart.setRxCallback(bleuart_rx_callback);

  // Increase Blink rate to different from PrPh advertising mode
  Bluefruit.setConnLedInterval(250);

  // Callbacks for Central
  Bluefruit.Central.setConnectCallback(connect_callback);
  Bluefruit.Central.setDisconnectCallback(disconnect_callback);

  /* Start Central Scanning
   * - Enable auto scan if disconnected
   * - Interval = 100 ms, window = 80 ms
   * - Don't use active scan
   * - Start(timeout) with timeout = 0 will scan forever (until connected)
   */
  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.restartOnDisconnect(true);
  Bluefruit.Scanner.setInterval(160, 80); // in unit of 0.625 ms
  Bluefruit.Scanner.useActiveScan(false);
  Bluefruit.Scanner.start(0);                   // // 0 = Don't stop scanning after n 
}

/**
 * Callback invoked when scanner pick up an advertising data
 * @param report Structural advertising data
 */
void scan_callback(ble_gap_evt_adv_report_t* report)
{
  // Check if advertising contain BleUart service
  if ( Bluefruit.Scanner.checkReportForService(report, clientUart) )
  {
    Serial.print("BLE UART service detected. Connecting ... ");

    // Connect to device with bleuart service in advertising
    Bluefruit.Central.connect(report);
  }
}

/**
 * Callback invoked when an connection is established
 * @param conn_handle
 */
void connect_callback(uint16_t conn_handle){
  Serial.println("Connected");

  Serial.print("Dicovering DIS ... ");
  if ( clientDis.discover(conn_handle) )
  {
    Serial.println("Found it");
    char buffer[32+1];
    
    // read and print out Manufacturer
    memset(buffer, 0, sizeof(buffer));
    if ( clientDis.getManufacturer(buffer, sizeof(buffer)) )
    {
      Serial.print("Manufacturer: ");
      Serial.println(buffer);
    }

    // read and print out Model Number
    memset(buffer, 0, sizeof(buffer));
    if ( clientDis.getModel(buffer, sizeof(buffer)) )
    {
      Serial.print("Model: ");
      Serial.println(buffer);
    }

    Serial.println();
  }  

  Serial.print("Discovering BLE Uart Service ... ");

  if ( clientUart.discover(conn_handle) )
  {
    Serial.println("Found it");

    Serial.println("Enable TXD's notify");
    clientUart.enableTXD();

    Serial.println("Ready to receive from peripheral");
  }else
  {
    Serial.println("Found NONE");
    
    // disconect since we couldn't find bleuart service
    Bluefruit.Central.disconnect(conn_handle);
  }  
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle
 * @param reason
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;
  
  Serial.println("Disconnected");
}

/**
 * Callback invoked when uart received data
 * @param uart_svc Reference object to the service where the data 
 * arrived. In this example it is clientUart
 */
void bleuart_rx_callback(BLEClientUart& uart_svc)
{
  Serial.print("[RX]: ");
  
  while ( uart_svc.available() )
  {
    Serial.print( (char) uart_svc.read() );
  }

  Serial.println();
}


void loop()
{
  if ( Bluefruit.Central.connected() )
  {
    // Not discovered yet
    if ( clientUart.discovered() )
    {
      // Discovered means in working state
      // Get Serial input and send to Peripheral
      if ( Serial.available() )
      {
        /* Get a new sensor event */
        sensors_event_t event, aevent, mevent;
        gyro.getEvent(&event);
        accelmag.getEvent(&aevent, &mevent);
        currAccel = aevent.acceleration.x;
        
        /* Display the results (speed is measured in rad/s) */
        Serial.print("X: "); Serial.print(event.gyro.x); Serial.print("  ");
        Serial.print("Y: "); Serial.print(event.gyro.y); Serial.print("  ");
        Serial.print("Z: "); Serial.print(event.gyro.z); Serial.print("  ");
        Serial.println("rad/s ");
        
        if(event.gyro.x > maxtilt){
          maxtilt = event.gyro.x;
          }
          
        /* Display the accel results (acceleration is measured in m/s^2) */
        Serial.print("A ");
        Serial.print("X: "); Serial.print(aevent.acceleration.x, 4); Serial.print("  ");
        Serial.print("Y: "); Serial.print(aevent.acceleration.y, 4); Serial.print("  ");
        Serial.print("Z: "); Serial.print(aevent.acceleration.z, 4); Serial.print("  ");
        Serial.println("m/s^2");
        
        //  this is code for making the velocity, however this suffers from drift, so fuck you.
        currTime = millis();
        velocity += (currAccel + prevAccel)/2*(currTime - prevTime)/1000; //1000 added to get the same units
        if(velocity > maxvelo){
          maxvelo = velocity;
        }
        speshdistance += velocity; 
        prevAccel = currAccel;
        prevTime = currTime;
        Serial.print("Velocity is "); Serial.print(velocity, 4); Serial.print("\n");
        delay(500);
      }
    }
  }
}

>>>>>>> 738c0a4018d7c6167be1d2296911fba439d330f4
