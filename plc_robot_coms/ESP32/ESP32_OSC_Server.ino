// This ESP32 example (Stack Chan) says messages from the OSC Client and changes faces depending on OSC messages
//
// AquesTalk-ESP32 is a text-to-speech library for ESP32.
// download it from here https://www.a-quest.com/download.html#a-etc
//
#include <M5Stack.h>
#include <ArduinoOSC.h>
#include <AquesTalkTTS.h>
#include <faces/FaceTemplates.hpp>
#include <M5Unified.h>
#include <Avatar.h>
#include <tasks/LipSync.h>
#include <WiFi.h>
#include <WiFiUDP.h>

using namespace m5avatar;

// function definitions
void messageCb(OSCMessage&);
//String UDPrecv(void) uncomment if using raw udp message

// for talk
const char* AQUESTALK_KEY = "XXXX-XXXX-XXXX-XXXX";
// for wifi udp and osc server
const char *ssid = "";    // WiFi ssid
const char *password = ""; // WiFi Password
WiFiUDP udp;
ArduinoOSCWiFi osc;
const int recv_port = 10000;

// if you want to use a UDP port raw for control uncomment these
// String s;
// const int u_port = 50100; // port

// for faces
Avatar avatar;
Face* faces[6];
const int num_faces = sizeof(faces) / sizeof(Face*);
int face_idx = 0;  // face index

// an example of customizing a face
class MyCustomFace : public Face {
   public:
    MyCustomFace()
        : Face(new UShapeMouth(44, 44, 0, 16), new BoundingRect(222, 160),
               // right eye, second eye arg is center position of eye
               new EllipseEye(32, 32, false), new BoundingRect(163, 64),
               //  left eye
               new EllipseEye(32, 32, true), new BoundingRect(163, 256),
               // right eyebrow
               // BowEyebrow's origin is the center of bow (arc)
               new BowEyebrow(64, 20, false),
               new BoundingRect(163, 64),  // (y,x)
                                           //  left eyebrow
               new BowEyebrow(64, 20, true), new BoundingRect(163, 256)) {}
};

ColorPalette* color_palettes[5];
const int num_palettes = sizeof(color_palettes) / sizeof(ColorPalette*);
int palette_idx = 0;

const Expression expressions[] = {
    Expression::Angry,
    Expression::Sleepy,
    Expression::Happy,
    Expression::Sad,
    Expression::Doubt,
    Expression::Neutral
};
const int num_expr = 6;
int expr_idx = 0;

// OSC message handler
void messageCb(OSCMessage& m) {
  String str = m.getArgAsString(0);
  int len = str.length(); 
  String cmd = str.substring(0, 4);
  String value = str.substring(5);
  if (cmd == "talk") {
    // Kanji TTS.playK(value.c_str(), 100);
    TTS.play(value.c_str(), 100);
    avatar.setSpeechText(value.c_str());
    avatar.setMouthOpenRatio(0.7);
    delay(200);
    avatar.setMouthOpenRatio(0);
    delay(3000);
  } else if (cmd == "face") {
    if (value == "dog") {
        avatar.setFace(faces[1]);
	} else if (value == "omega") {
        avatar.setFace(faces[2]);
	} else if (value == "girl") {
        avatar.setFace(faces[3]);
	} else if (value == "pink") {
        avatar.setFace(faces[4]);
	} else if (value == "custom") {
        avatar.setFace(faces[5]);
	} else if (value == "native") {
        avatar.setFace(faces[0]);
	}
  } else if (cmd == "blin") {
    avatar.setIsAutoBlink(!avatar.getIsAutoBlink());
  } else if (cmd == "colr") {
    avatar.setColorPalette(*color_palettes[palette_idx]);
    palette_idx = (palette_idx + 1) % num_palettes;
  } else if (cmd == "expr") {
    avatar.setExpression(expressions[expr_idx]);
    expr_idx = (expr_idx + 1) % num_expr;
  }
}

void setup() {
  M5.begin();  
  /// Increasing the sample_rate will improve the sound quality instead of increasing the CPU load.
  auto spk_cfg = M5.Speaker.config();
  spk_cfg.sample_rate = 96000; // default:64000 (64kHz)  e.g. 48000 , 50000 , 80000 , 96000 , 100000 , 128000 , 144000 , 192000 , 200000
  M5.Speaker.config(spk_cfg);
  M5.Speaker.setVolume(128);  
  M5.Speaker.begin();
 
  // Wi-Fi
  setupWiFi();         // if not work use as below  
  // WiFi.disconnect(true, true);  WiFi OFF, eraseAP=true
  // delay(500);
  // WiFi.mode(WIFI_STA);
  // WiFi.begin(ssid, password);
  // Serial.begin(115200);
  // while (WiFi.status() != WL_CONNECTED) {
  //    delay(500);
  //    Serial.print(".");
  // }
  // Serial.println(" CONNECTED");
  // udp.begin(u_port);   <--- use this if you want to use raw udp for messages
  
  // setup an array of faces
  faces[0] = avatar.getFace();  // native face
  faces[1] = new DoggyFace();
  faces[2] = new OmegaFace();
  faces[3] = new GirlyFace();
  faces[4] = new PinkDemonFace();
  faces[5] = new MyCustomFace();

  // color pallets
  color_palettes[0] = new ColorPalette();
  color_palettes[1] = new ColorPalette();
  color_palettes[2] = new ColorPalette();
  color_palettes[3] = new ColorPalette();
  color_palettes[4] = new ColorPalette();
  color_palettes[1]->set(COLOR_PRIMARY,
                           M5.Lcd.color24to16(0x383838));  // eye
  color_palettes[1]->set(COLOR_BACKGROUND,
                           M5.Lcd.color24to16(0xfac2a8));  // skin
  color_palettes[1]->set(COLOR_SECONDARY,
                           TFT_PINK);  // cheek
  color_palettes[2]->set(COLOR_PRIMARY, TFT_YELLOW);
  color_palettes[2]->set(COLOR_BACKGROUND, TFT_DARKCYAN);
  color_palettes[3]->set(COLOR_PRIMARY, TFT_DARKGREY);
  color_palettes[3]->set(COLOR_BACKGROUND, TFT_WHITE);
  color_palettes[4]->set(COLOR_PRIMARY, TFT_RED);
  color_palettes[4]->set(COLOR_BACKGROUND, TFT_PINK);
  
  // OSC
  osc.begin(udp, recv_port);
  osc.addCallback("/msg", &messageCb);
  // For Kanji-to-speech mode (requires dictionary file saved on microSD)
  // ssPin: Specifies the CS pin for SPI communication with the SD card.
  // In the case of M5Stack, it is "GPIO4"
  // SD.begin(GPIO_NUM_4);
  // int iret = TTS.createK(AQUESTALK_KEY);
  int iret = TTS.create(AQUESTALK_KEY);
  avatar.init(8);                                         // start drawing in 8 bit color mode
  avatar.setColorPalette(*color_palettes[0]);
  avatar.setExpression(expressions[2]);
  avatar.addTask(lipSync, "lipSync");
}

void loop() {
  osc.parse();
  // M5Stack Core's button layout:
  // -----------
  // |         |
  // |         |
  // -----------
  // [A] [B] [C]
  // A changes the face from the front panel
  if (M5.BtnA.wasPressed()) {
    avatar.setFace(faces[face_idx]);
    face_idx = (face_idx + 1) % num_faces;  // loop index
  }
  if (M5.BtnB.wasPressed()) {
    // switch right eye
    avatar.setRightEyeOpenRatio(avatar.getRightEyeOpenRatio() > 0.5f ? 0.0f
                                                                     : 1.0f);
  }
  if (M5.BtnC.wasPressed()) {
    // switch left eye
    avatar.setLeftEyeOpenRatio(avatar.getLeftEyeOpenRatio() > 0.5f ? 0.0f
                                                                   : 1.0f);
  }
  // uncomment if you want to use raw udp messages for control 
  //s = UDPrecv();
 
  //if (s.equals("blin") {
  //    avatar.setIsAutoBlink(!avatar.getIsAutoBlink());
  //} else {
  //    TTS.playK(s.c_str(), 100);   <- use if kanji
  //    TTS.play(s.c_str(), 100);    <- otherwise 
  //    avatar.setSpeechText(s.c_str());
  //    avatar.setMouthOpenRatio(0.7);
  //    delay(200);
  //    avatar.setMouthOpenRatio(0);
  //    delay(3000); 
  //}  
}

// uncomment if using raw udp messages
//String UDPrecv(void) {
//    char packetBuffer[1024];
//    int packetSize = udp.parsePacket();
//    if (packetSize)
//    {
//      int len = udp.read(packetBuffer, packetSize);
//      if (len > 0)
//      {
//        packetBuffer[len] = '\0'; // end
//      }
//      s = packetBuffer;
//    }
//    return s;
//}

