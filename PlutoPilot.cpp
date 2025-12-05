// Do not remove the include below
#include "PlutoPilot.h"
#include "API/Debugging.h"

// --- ADD THESE INCLUDES ---
#include "drivers/ranging_vl53l1x.h" // To see XVision
#include "sensors/barometer.h"       // To see BaroAlt
#include "flight/altitudehold.h"     // To see EstAlt (Final estimate)

extern LaserSensor_L1 XVision;       // Tell compiler XVision exists elsewhere
extern float BaroAlt;              // Tell compiler BaroAlt exists
extern int32_t MyEstAlt;               // Final Estimated Altitude

// In PlutoPilot.cpp
extern int16_t debug_flow_x;
extern int16_t debug_flow_y;
extern int16_t debug_flow_squal;

extern float gyroCompX;
extern float gyroCompY;

extern int16_t accSmooth [ XYZ_AXIS_COUNT ];
extern int32_t accSum [ XYZ_AXIS_COUNT ];
extern int32_t accSumXYZ [ XYZ_AXIS_COUNT ];

extern int accSumCountXYZ;

// Add Externs
extern int16_t debug_raw_flow_x;
extern int16_t debug_raw_flow_y;
extern int16_t debug_gyro_roll;
extern int16_t debug_gyro_pitch;

extern int16_t only_imu_pos_X;
extern int16_t only_imu_pos_Y;
extern int16_t only_imu_vel_X;
extern int16_t only_imu_vel_Y;

extern float accel_EF[2];

extern int32_t total_flow_x_counts;
extern bool integration_active;
extern float accVelScale;

// Add these lines at the top with your other externs
// extern int32_t debug_checkReading_count;
// extern int16_t debug_last_raw_mm;
// extern int8_t  debug_startRanging_result;

/**
 * Configures Pluto's receiver to use PPM or default ESP mode; activate the line matching your setup.
 * AUX channel configurations is only for PPM recievers if no custom configureMode function is called this are the default setup
 * ARM mode : Rx_AUX2, range 1300 to 2100
 * ANGLE mode : Rx_AUX2, range 900 to 2100
 * BARO mode : Rx_AUX3, range 1300 to 2100
 * MAG mode : Rx_AUX1, range 900 to 1300
 * HEADFREE mode : Rx_AUX1, range 1300 to 1700
 * DEV mode : Rx_AUX4, range 1500 to 2100
 */

unsigned long lastPrintTime = 0;

void plutoRxConfig ( void ) {
  // Receiver mode: Uncomment one line for ESP or CAM or PPM setup.
  Receiver_Mode ( Rx_ESP );    // Onboard ESP
  // Receiver_Mode ( Rx_CAM );    // WiFi CAMERA
  // Receiver_Mode ( Rx_PPM );    // PPM based
}

// The setup function is called once at Pluto's hardware startup
void plutoInit ( void ) {
  // Add your hardware initialization code here
  setUserLoopFrequency(100);
}

// The function is called once before plutoLoop when you activate Developer Mode
void onLoopStart ( void ) {
  // do your one time stuffs here
}

// The loop function is called in an endless loop
void plutoLoop ( void ) {
unsigned long now = millis();

// if (now - lastPrintTime >= 100) {
//         lastPrintTime = now;

//         // Print Group A: Flow X vs Gyro Roll/Pitch
//         Monitor_Println("FlowX:", debug_raw_flow_x);
//         Monitor_Println("FlowY:", debug_raw_flow_y);

//         Monitor_Println("gyro_PITCH:", (int16_t) gyroCompY);
//         Monitor_Println("gyro_ROLL:", (int16_t) gyroCompX);
//         Monitor_Println("SQUAL:", debug_flow_squal);
        
//         // Verify Motion
//         Monitor_Println("Cleaned_FlowX:", debug_flow_x);
//         Monitor_Println("Cleaned_FlowY:", debug_flow_y);
// }

// CONTROL LOGIC: Using Headfree Mode Switch (AUX1)
    static bool test_running = false;

    // Trigger: Roll Stick Hard Right (> 1900)
    // HOLD the stick right to start the test "Ready State"
    if (rcData[ROLL] > 1900 && !test_running) {
        
        total_flow_x_counts = 0;   // Reset Counter
        integration_active = true; // Enable Counting
        test_running = true;       // Set State
        
        Monitor_Println("STATUS: Recording! Rotate 90 deg, then release stick.");
    }

    // Stop Trigger: Roll Stick Released (Centered < 1600)
    if (rcData[ROLL] < 1600 && test_running) {
        
        integration_active = false; // Stop Counting
        test_running = false;       // Reset State
        
        // Print Results
        Monitor_Println("Total Counts:", total_flow_x_counts);
        
        // Calculate (90 deg = 1.5708 rad)
        if (abs(total_flow_x_counts) > 0) {
            float focal_length = (float)abs(total_flow_x_counts) / 1.5708f;
            Monitor_Println("Focal Length:", (int)focal_length);
        } else {
            Monitor_Println("Error: 0 Counts. Check SQUAL or Lens Cap.");
        }
    }

    // if (now - lastPrintTime >= 100) {
    //         lastPrintTime = now;

    //         Monitor_Println("AccX:", (float)accSumCountXYZ);
    //         Monitor_Println("AccY:", accSumCountXYZ);
            
    //         Monitor_Println("VelX:", (((float)accSum[0] / (float)accSumCountXYZ)*accVelScale), 4);
    //         Monitor_Println("VelY:", only_imu_vel_Y);
    // }

    // if (now - lastPrintTime >= 100) { // 10Hz Refresh Rate
    //     lastPrintTime = now;

    //     // Print the Heartbeat (Should increase constantly)
    //     Monitor_Print("TaskCount:", debug_checkReading_count);

    //     // Print the Boolean Result (0 or 1)
    //     Monitor_Print(", IsReady:", debug_startRanging_result);

    //     // Print the Raw Data
    //     Monitor_Println(", RawMM:", debug_last_raw_mm);
    // }


    //     static uint32_t lastPrint = 0;

          // if (now - lastPrintTime >= 100) {
          //       lastPrintTime = now;

          //       // Verify Quality first (Should be > 0, typically 50-100 on carpet)
          //       Monitor_Println("SQUAL:", debug_flow_squal);
                
          //       // Verify Motion
          //       Monitor_Println("FlowX:", debug_flow_x);
          //       Monitor_Println("FlowY:", debug_flow_y);
          //   }

    //     if (millis() - lastPrint > 100) { // Limit to 10Hz
    //         lastPrint = millis();
    //         // Read the global variable directly to see if the pipeline is delivering data
    //         Monitor_Println("Debug Range:", XVision.getLaserRange()); 
    //     }



    // if (now - lastPrintTime >= 100) { // 10Hz Printing
    //     lastPrintTime = now;

    //     // 1. Get Barometer Height (usually in cm)
    //     float baro_cm = BaroAlt; 

    //     // 2. Get Laser Height (using the new API)
    //     // Note: getLaserRange returns mm, so divide by 10 for cm to match Baro
    //     float laser_mm = XVision.getLaserRange();
    //     float laser_cm = laser_mm / 10.0f;

    //     // 3. Get the Final Fused Estimate (what the drone actually thinks its height is)
    //     float fused_cm = MyEstAlt;

    //     // Print for Teleplot (Graph format)
    //     // Format: "GRAPH:Label:Value"
        
    //     Monitor_Println("Baro_cm:", baro_cm);
    //     Monitor_Println("Laser_cm:", laser_cm, 1);
    //     Monitor_Println("Fused_cm:", fused_cm);
    // }
        // Use this if you just want to read text values
        /*
        Monitor_Print("Baro: ", baro_cm);
        Monitor_Print(" | Laser: ", laser_cm, 1);
        Monitor_Println(" | Fused: ", fused_cm);
        */
    //}
}

// The function is called once after plutoLoop when you deactivate Developer Mode
void onLoopFinish ( void ) {
  // do your cleanup stuffs here
}
