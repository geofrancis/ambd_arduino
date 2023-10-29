/*

  Example guide:
  https://www.amebaiot.com/en/amebapro2-amb82-mini-arduino-neuralnework-object-detection/

  For recommended setting to achieve better video quality, please refer to our Ameba FAQ: https://forum.amebaiot.com/t/ameba-faq/1220

  NN Model Selection
  Select Neural Network(NN) task and models using .modelSelect(nntask, objdetmodel, facedetmodel, facerecogmodel).
  Replace with NA_MODEL if they are not necessary for your selected NN Task.

  NN task
  =======
  OBJECT_DETECTION/ FACE_DETECTION/ FACE_RECOGNITION

  Models
  =======
  YOLOv3 model         DEFAULT_YOLOV3TINY   / CUSTOMIZED_YOLOV3TINY
  YOLOv4 model         DEFAULT_YOLOV4TINY   / CUSTOMIZED_YOLOV4TINY
  YOLOv7 model         DEFAULT_YOLOV7TINY   / CUSTOMIZED_YOLOV7TINY
  SCRFD model          DEFAULT_SCRFD        / CUSTOMIZED_SCRFD
  MobileFaceNet model  DEFAULT_MOBILEFACENET/ CUSTOMIZED_MOBILEFACENET
  No model             NA_MODEL
*/

#include <Arduino.h>

#include "mavlink/common/mavlink.h"
unsigned long previousMillis = 0;        // will store last time LED was updated
const long interval = 3000;           // interval at which to blink (milliseconds)

char numberArray[20];
uint32_t number = 35000; // 5 digits








///////////////////////////////////////////////
////////////////////
#include "WiFi.h"
#include "StreamIO.h"
#include "VideoStream.h"
#include "RTSP.h"
#include "NNObjectDetection.h"
#include "VideoStreamOverlay.h"
#include "ObjectClassList.h"

#define CHANNEL 0
#define CHANNELNN 3

// Lower resolution for NN processing
#define NNWIDTH 576
#define NNHEIGHT 320

VideoSetting config(VIDEO_FHD, 30, VIDEO_H264, 0);
VideoSetting configNN(NNWIDTH, NNHEIGHT, 10, VIDEO_RGB, 0);
NNObjectDetection ObjDet;
RTSP rtsp;
StreamIO videoStreamer(1, 1);
StreamIO videoStreamerNN(1, 1);

char ssid[] = "2.4GN";    // your network SSID (name)
char pass[] = "passw0rded";       // your network password
int status = WL_IDLE_STATUS;

IPAddress ip;
int rtsp_portnum;

void setup() {

  Serial3.begin(115200); //RXTX from Pixhawk (Port 19,18 Arduino Mega)
  Serial.begin(115200); //Main serial port for console output

  // attempt to connect to Wifi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);

    // wait 2 seconds for connection:
    delay(2000);
  }
  ip = WiFi.localIP();

  // Configure camera video channels with video format information
  // Adjust the bitrate based on your WiFi network quality
  config.setBitrate(2 * 1024 * 1024);     // Recommend to use 2Mbps for RTSP streaming to prevent network congestion
  Camera.configVideoChannel(CHANNEL, config);
  Camera.configVideoChannel(CHANNELNN, configNN);
  Camera.videoInit();

  // Configure RTSP with corresponding video format information
  rtsp.configVideo(config);
  rtsp.begin();
  rtsp_portnum = rtsp.getPort();

  // Configure object detection with corresponding video format information
  // Select Neural Network(NN) task and models
  ObjDet.configVideo(configNN);
  ObjDet.setResultCallback(ODPostProcess);
  ObjDet.modelSelect(OBJECT_DETECTION, DEFAULT_YOLOV7TINY, NA_MODEL, NA_MODEL);
  ObjDet.begin();

  // Configure StreamIO object to stream data from video channel to RTSP
  videoStreamer.registerInput(Camera.getStream(CHANNEL));
  videoStreamer.registerOutput(rtsp);
  if (videoStreamer.begin() != 0) {
    Serial.println("StreamIO link start failed");
  }

  // Start data stream from video channel
  Camera.channelBegin(CHANNEL);

  // Configure StreamIO object to stream data from RGB video channel to object detection
  videoStreamerNN.registerInput(Camera.getStream(CHANNELNN));
  videoStreamerNN.setStackSize();
  videoStreamerNN.setTaskPriority();
  videoStreamerNN.registerOutput(ObjDet);
  if (videoStreamerNN.begin() != 0) {
    Serial.println("StreamIO link start failed");
  }

  // Start video channel for NN
  Camera.channelBegin(CHANNELNN);

  // Start OSD drawing on RTSP video channel
  OSD.configVideo(CHANNEL, config);
  OSD.begin();


  request_datastream();
  delay(5000);



  uint8_t system_id = 1;
  uint8_t component_id = 2;
  uint8_t severity = 1;
  uint16_t id = 0;
  uint8_t chunk_seq = 0;

  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_statustext_pack(system_id, component_id, &msg, 0, "CAMERA ONLINE", id, chunk_seq);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial3.write(buf, len);

  //send mavlink message

}

void loop() {



  MavLink_receive();
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    //setmode_Auto();
  }



}

// User callback function for post processing of object detection results
void ODPostProcess(std::vector<ObjectDetectionResult> results) {
  uint16_t im_h = config.height();
  uint16_t im_w = config.width();

  Serial.print("Network URL for RTSP Streaming: ");
  Serial.print("rtsp://");
  Serial.print(ip);
  Serial.print(":");
  Serial.println(rtsp_portnum);
  Serial.println(" ");

  printf("Total number of objects detected = %d\r\n", ObjDet.getResultCount());
  OSD.createBitmap(CHANNEL);

  if (ObjDet.getResultCount() > 0) {
    for (uint32_t i = 0; i < ObjDet.getResultCount(); i++) {
      int obj_type = results[i].type();
      if (itemList[obj_type].filter) {    // check if item should be ignored

        ObjectDetectionResult item = results[i];
        // Result coordinates are floats ranging from 0.00 to 1.00
        // Multiply with RTSP resolution to get coordinates in pixels
        int xmin = (int)(item.xMin() * im_w);
        int xmax = (int)(item.xMax() * im_w);
        int ymin = (int)(item.yMin() * im_h);
        int ymax = (int)(item.yMax() * im_h);

        // Draw boundary box
        printf("Item %d %s:\t%d %d %d %d\n\r", i, itemList[obj_type].objectName, xmin, xmax, ymin, ymax);
        OSD.drawRect(CHANNEL, xmin, ymin, xmax, ymax, 3, OSD_COLOR_WHITE);

        // Print identification text
        char text_str[20];
        snprintf(text_str, sizeof(text_str), "%s %d", itemList[obj_type].objectName, item.score());
        OSD.drawText(CHANNEL, xmin, ymin - OSD.getTextHeight(CHANNEL), text_str, OSD_COLOR_CYAN);


        uint8_t system_id = 1;
        uint8_t component_id = 2;
        uint8_t severity = 1;
        uint16_t id = 0;
        uint8_t chunk_seq = 0;

        mavlink_message_t msg;
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        mavlink_msg_statustext_pack(system_id, component_id, &msg, 0, text_str, id, chunk_seq);
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        Serial3.write(buf, len);

        //send mavlink message
      }
    }
  }
  OSD.update(CHANNEL);
}














void MavLink_receive()
{
  mavlink_message_t msg;
  mavlink_status_t status;

  while (Serial3.available())
  {
    uint8_t c = Serial3.read();

    //Get new message
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
    {

      //Handle new message from autopilot
      switch (msg.msgid)
      {

          /*    case MAVLINK_MSG_ID_GPS_RAW_INT:
                {
                  mavlink_gps_raw_int_t packet;
                  mavlink_msg_gps_raw_int_decode(&msg, &packet);

            mavlink_gps_raw_int_t datagps;
                  mavlink_msg_gps_raw_int_decode (&msg, &datagps);
                  //Serial.println("PX HB");
                 Serial.println("GPS Data ");
                  Serial.print("time usec: ");
                  Serial.println(datagps.time_usec);
                  Serial.print("lat: ");
                  Serial.println(datagps.lat);
                  Serial.print("lon: ");
                  Serial.println(datagps.lon);
                  Serial.print("alt: ");
                  Serial.println(datagps.alt);
                  Serial.print("Sattelite visible: ");
                  Serial.println(datagps.satellites_visible);
                  //Serial.println(datagps.eph);
                  //Serial.println(datagps.epv);
          */
          //  }
          break;

        case MAVLINK_MSG_ID_SYS_STATUS:  // #1: SYS_STATUS
          {
            //mavlink_message_t* msg;
            mavlink_sys_status_t sys_status;
            mavlink_msg_sys_status_decode(&msg, &sys_status);
            Serial.print("PX SYS STATUS: ");
            Serial.print("[Bat (V): ");
            Serial.print(sys_status.voltage_battery);
            utoa(number, numberArray, 10);            
            Serial.print("], [Bat (A): ");
            Serial.print(sys_status.current_battery);
            Serial.print("], [Comms loss (%): ");
            Serial.print(sys_status.drop_rate_comm);
            Serial.println("]");
          }


          break;
          /*
                  case MAVLINK_MSG_ID_ATTITUDE:  // #30
                    {

                      mavlink_attitude_t attitude;
                      mavlink_msg_attitude_decode(&msg, &attitude);
                      Serial.println("PX ATTITUDE");
                      Serial.println(attitude.roll);
                      //if (attitude.roll > 1) leds_modo = 0;
                      //else if (attitude.roll < -1) leds_modo = 2;
                      //else leds_modo = 1;
                    }
          */
          break;
          /*
                  case MAVLINK_MSG_ID_RC_CHANNELS_RAW:  // #35
                    {
                      mavlink_rc_channels_raw_t chs;
                      mavlink_msg_rc_channels_raw_decode(&msg, &chs);
                      Serial.print("Chanel 6 (3-Kanal Schalter): ");
                      Serial.println(chs.chan6_raw);
                    }
          */
          break;
          /*
            case MAVLINK_MSG_ID_RC_CHANNELS_SCALED:  // #35
            {
            mavlink_rc_channels_scaled_t RCCHANNEL;
            mavlink_msg_rc_channels_scaled_decode(&msg, &RCCHANNEL);
            Serial.print("Chanel 6 (3-Kanal Schalter): ");
            int RAW_SERVO = RCCHANNEL.chan6_scaled;
            Serial.println(RAW_SERVO);
            Serial.print("Chanel 5 (Schub): ");
            Serial.println(RCCHANNEL.chan5_scaled);
            Serial.print("Drei Kanal: ");
            Serial.println(mavlink_msg_rc_channels_scaled_get_chan6_scaled(&msg));
            Serial.print("Schub: ");
            Serial.println(mavlink_msg_rc_channels_scaled_get_chan5_scaled(&msg));
            }
          */
          break;
          /*
            case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:  // #30
            {
            mavlink_global_position_int_t Position;
            //mavlink_msg_attitude_decode(&msg, &attitude);
            mavlink_msg_global_position_int_decode(&msg, &Position);
          */
          break;

        case MAVLINK_MSG_ID_HEARTBEAT:  // #0: Heartbeat
          {

            mavlink_heartbeat_t hb;
            mavlink_msg_heartbeat_decode(&msg, &hb);

            Serial.print("\nFlight Mode: (10 ");
            Serial.println(hb.custom_mode);
            Serial.print("Type: ");
            Serial.println(hb.type);
            Serial.print("Autopilot: ");
            Serial.println(hb.autopilot);
            Serial.print("Base Mode: ");
            Serial.println(hb.base_mode);
            Serial.print("System Status: ");
            Serial.println(hb.system_status);
            Serial.print("Mavlink Version: ");
            Serial.println(hb.mavlink_version);
            Serial.println();
          }
          break;

          /* case MAVLINK_MSG_ID_STATUSTEXT: //  #253  https://mavlink.io/en/messages/common.html#STATUSTEXT
             {
               mavlink_statustext_t packet;
               mavlink_msg_statustext_decode(&msg, &packet);
               Serial.print("=STATUSTEXT");
               Serial.print(" severity:");
               Serial.print(packet.severity);
               Serial.print(" text:");
               Serial.print(packet.text);
          */
          break;
      }
    }
  }
}









void request_datastream() {
  //Request Data from Pixhawk
  uint8_t _system_id = 255; // id of computer which is sending the command (ground control software has id of 255)
  uint8_t _component_id = 2; // seems like it can be any # except the number of what Pixhawk sys_id is
  uint8_t _target_system = 1; // Id # of Pixhawk (should be 1)
  uint8_t _target_component = 0; // Target component, 0 = all (seems to work with 0 or 1
  uint8_t _req_stream_id = MAV_DATA_STREAM_ALL;
  uint16_t _req_message_rate = 0x01; //number of times per second to request the data in hex
  uint8_t _start_stop = 1; //1 = start, 0 = stop

  // STREAMS that can be requested
  /*
     Definitions are in common.h: enum MAV_DATA_STREAM and more importantly at:
     https://mavlink.io/en/messages/common.html#MAV_DATA_STREAM

     MAV_DATA_STREAM_ALL=0, // Enable all data streams
     MAV_DATA_STREAM_RAW_SENSORS=1, /* Enable IMU_RAW, GPS_RAW, GPS_STATUS packets.
     MAV_DATA_STREAM_EXTENDED_STATUS=2, /* Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS
     MAV_DATA_STREAM_RC_CHANNELS=3, /* Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW
     MAV_DATA_STREAM_RAW_CONTROLLER=4, /* Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT.
     MAV_DATA_STREAM_POSITION=6, /* Enable LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT messages.
     MAV_DATA_STREAM_EXTRA1=10, /* Dependent on the autopilot
     MAV_DATA_STREAM_EXTRA2=11, /* Dependent on the autopilot
     MAV_DATA_STREAM_EXTRA3=12, /* Dependent on the autopilot
     MAV_DATA_STREAM_ENUM_END=13,

     Data in PixHawk available in:
      - Battery, amperage and voltage (SYS_STATUS) in MAV_DATA_STREAM_EXTENDED_STATUS
      - Gyro info (IMU_SCALED) in MAV_DATA_STREAM_EXTRA1
  */

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  mavlink_msg_request_data_stream_pack(_system_id, _component_id, &msg, _target_system, _target_component, _req_stream_id, _req_message_rate, _start_stop);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);  // Send the message (.write sends as bytes)

  //Serial3.write(buf, len); //Write data to serial port
}


/*
  void setmode_Auto() {
  //Set message variables
  uint8_t _system_id = 255; // id of computer which is sending the command (ground control software has id of 255)
  uint8_t _component_id = 2; // seems like it can be any # except the number of what Pixhawk sys_id is
  uint8_t _target_system = 1; // Id # of Pixhawk (should be 1)
  uint8_t _base_mode = 1;
   _custom_mode = 10; //10 = auto mode

  /*
    Flight / Driving Modes (change custom mode above)
    0 - Manual
    1 - Acro
    3 - Steering
    4 - Hold
    5 - Loiter
    6 - Follow
    7 - Simple
    10  Auto
    11  RTL
    12  SmartRTL
    15  Guided


  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  mavlink_msg_set_mode_pack(_system_id, _component_id, &msg, _target_system, _base_mode, _custom_mode);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);  // Send the message (.write sends as bytes)
  Serial.print("\nsending set mode command...");
  Serial3.write(buf, len); //Write data to serial port
  }

*/


/*void sendtext() {


  uint8_t system_id = 1;
  uint8_t component_id = 2;
  uint8_t severity = 1;

  uint16_t id = 0;
  uint8_t chunk_seq = 0;



  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_statustext_pack(system_id, component_id, &msg, 0, "TEST", id, chunk_seq);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  // Serial3.write(buf, len);
  }
*/
