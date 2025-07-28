#include "comms.h"

Comms::Comms(PID* p, PID* r, PID* y, PID* ap, PID* ar,Receiver* rx) : pitch(p), roll(r), yaw(y), aPitch(ap), aRoll(ar),rx(rx) {}


void Comms::init(){
  Serial.begin(115200); // begin serial communication

}


void Comms::parseGains(char* message) {
  if (strncmp(message, "TUNE,", 5) == 0) {
    if (rx->isArmed()) {
      Serial.println("ERR");
      return;
    }
    char* args = message + 5;
    char* token = strtok(args, ",");
    while (token != nullptr) {
      char* eq = strchr(token, '=');
      if (eq != nullptr) {
        *eq = '\0';
        const char* key = token;
        float val = atof(eq + 1);

        if (strcmp(key, "pp") == 0) pitch->setTune(val, -1, -1);
        else if (strcmp(key, "pi") == 0) pitch->setTune(-1, val, -1);
        else if (strcmp(key, "pd") == 0) pitch->setTune(-1, -1, val);
        
        else if (strcmp(key, "rp") == 0) roll->setTune(val, -1, -1);
        else if (strcmp(key, "ri") == 0) roll->setTune(-1, val, -1);
        else if (strcmp(key, "rd") == 0) roll->setTune(-1, -1, val);

        else if (strcmp(key, "yp") == 0) yaw->setTune(val, -1, -1);
        else if (strcmp(key, "yi") == 0) yaw->setTune(-1, val, -1);
        else if (strcmp(key, "yd") == 0) yaw->setTune(-1, -1, val);

        else if (strcmp(key, "app") == 0) aPitch->setTune(val, -1, -1);
        else if (strcmp(key, "api") == 0) aPitch->setTune(-1, val, -1);
        else if (strcmp(key, "apd") == 0) aPitch->setTune(-1, -1, val);

        else if (strcmp(key, "arp") == 0) aRoll->setTune(val, -1, -1);
        else if (strcmp(key, "ari") == 0) aRoll->setTune(-1, val, -1);
        else if (strcmp(key, "ard") == 0) aRoll->setTune(-1, -1, val);
      }
      token = strtok(nullptr, ",");
    }
    // Serial.println("ACK"); // send ack once tune is set

    sendTune(); // send updated tune after
  }
}

void Comms::sendTune() {
  String msg = "TUNE";
  PID::Gains g;

  g = this->roll->getTune(); msg += ",rp=" + String(g.p,3) + ",ri=" + String(g.i,3) + ",rd=" + String(g.d,3);
  g = this->pitch->getTune(); msg += ",pp=" + String(g.p,3) + ",pi=" + String(g.i,3) + ",pd=" + String(g.d,3);
  g = this->yaw->getTune(); msg += ",yp=" + String(g.p,3) + ",yi=" + String(g.i,3) + ",yd=" + String(g.d,3);
  g = this->aRoll->getTune(); msg += ",arp=" + String(g.p,3) + ",ari=" + String(g.i,3) + ",ard=" + String(g.d,3);
  g = this->aPitch->getTune(); msg += ",app=" + String(g.p,3) + ",api=" + String(g.i,3) + ",apd=" + String(g.d,3);

  Serial.println(msg); // send back to Portenta
  Serial.flush(); // ensure all data sent
}

void Comms::parseMessage(char* message) {
  if (strncmp(message, "REQUEST,TUNE", 12) == 0) {
    if (!rx->isArmed()) {  // only send if disarmed
      sendTune();
    }
  } else {
    parseGains(message);
  }
}