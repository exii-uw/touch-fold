int state = 0; // 0: waiting for connection ready; 1: waiting for calibration; 2: visualization

import processing.serial.*;
import org.apache.commons.math3.complex.Quaternion;
import peasy.*;
import controlP5.*;

Serial myPort;

boolean getInitState = false;
String initState = "0,0,0,0,0,0,0,0,1.00,0.00,0.00,0.00,1.00,0.00,0.00,-0.00";
String[] currentState;
int[] touchSequence = {6, 7, 5, 4, 1, 0, 3, 2};
int[] touchTask = {2, 4, 1, 3, 6, 8, 5, 7};
String[] touchCoor = {"(1, 1)", "(2, 1)", "(1, 2)", "(2, 2)", "(1, 3)", "(2, 3)", "(1, 4)", "(2, 4)"};

ControlP5 cp5;
Button calibrationBtn;

PFont boldFontLarge, normalFont, boldFontSmall;

Quaternion qTopCali, qBottomCali;
boolean cp5Init = false;

// gesture state machine
int gestureState = 0; // 0: start; 1: fold; 2: touch; 3: touch and fold; 4: end
int prevGestureState = 0;
int foldState = 0; // 0: unfold; 1: fold
float foldAngleMax = 0;
int touchState = 0; // 0: untouched; 1: touched
boolean reverse = false;
IntList touchPoints = new IntList();
String showInteraction = "";
String showResults = "";
int showResultCounter=0;
int sectionWidth = 72;
int sectionHeight = 83;
int sectionThick = 8;


void setup() {
  //size (720, 480, P3D);
  fullScreen(P3D);
  frameRate(60);
  ortho();

  //String[] fontList = PFont.list();
  //printArray(fontList);

  normalFont = createFont("HelveticaNeueLTPro-Roman", 120);
  boldFontLarge = createFont("HelveticaNeueLTPro-Bd", 144);
  boldFontSmall = createFont("HelveticaNeueLTPro-Bd", 120);
  textFont(normalFont);

  myPort = new Serial(this, "COM3", 115200);
  myPort.bufferUntil('\n');

  currentState = split(initState, ',');
  cp5 = new ControlP5(this);

  // create a new button with name 'calibrationBtn'
  calibrationBtn = cp5.addButton("calibration")
    .setLabel("")
    .setValue(255)
    .setPosition(width - 200, height - 100)
    .setSize(100, 50)
    .setColorBackground(0x1C2833)
    ;
  calibrationBtn.show();

  cp5Init = true;
}


void draw() {
  background (0);
  if (state == 1) {
    calibrationBtn.show();
  } else if (state == 2) {
    processState(currentState);
  }
}

void processState(String[] inData) {
  //println(inData);
  // angle
  Quaternion qTop = new Quaternion(float(inData[12]), -float(inData[14]), float(inData[13]), float(inData[15]));
  Quaternion qBottom = new Quaternion(float(inData[8]), -float(inData[10]), float(inData[9]), float(inData[11]));

  // get quaternion difference
  Quaternion rotationDiff = quaternion_diff(qBottomCali.multiply(qBottom), qTopCali.multiply(qTop));
  //println(rotationDiff);
  float angle = map(toEulerAngles(rotationDiff), 0, PI, 0, 180);
  Quaternion topRotation = toQuaternion(toEulerAngles(rotationDiff), 0.0, 0.0);

  if (foldState == 0) {
    // check if it's fold
    if (abs(angle) >= 20) {
      // fold change the state
      foldState = 1;
      foldAngleMax = angle;
      if (gestureState == 0) {
        gestureState = 1;
        prevGestureState = 0;
      } else if (gestureState == 2) {
        gestureState = 3;
        prevGestureState = 2;
      }
    }
  } else {
    // update maximum
    if (abs(foldAngleMax) < abs(angle)) {
      foldAngleMax = angle;
    } else {
      // check if it's unfold
      if (abs(angle - foldAngleMax) > 20) {
        // unfold, classify
        if (gestureState == 1 || gestureState == 3) {
          classify();
        }
      }
    }
  }

  int[] touchPoint = {0, 0, 0, 0, 0, 0, 0, 0};
  int sumTouch = 0;
  for (int i=0; i<8; i++) {
    touchPoint[i] = int(inData[touchSequence[i]]);
    sumTouch = sumTouch + touchPoint[i];
  }
  //println(touchPoint);
  if (touchState == 0) {
    // check if it's touched
    if (sumTouch != 0) {
      touchState = 1;
      for (int i=0; i<8; i++) {
        if (touchPoint[i] == 1 && !touchPoints.hasValue(touchTask[i])) {
          touchPoints.append(touchTask[i]);
        }
      }
      if (gestureState == 0) {
        gestureState = 2;
        prevGestureState = 0;
      } else if (gestureState == 1) {
        gestureState = 3;
        prevGestureState = 1;
      }
    }
  } else {
    // update touch
    for (int i=0; i<8; i++) {
      if (touchPoint[i] == 1 && !touchPoints.hasValue(touchTask[i])) {
        touchPoints.append(touchTask[i]);
      }
    }
    if (sumTouch == 0) {
      // touch up, classify
      if (gestureState == 2 || gestureState == 3) {
        classify();
      }
    }
  }

  if (gestureState == 4 && abs(angle) < 20 && sumTouch == 0) {
    // reset all state
    gestureState = 0;
    prevGestureState = 0;
    foldState = 0;
    foldAngleMax = 0;
    touchState = 0;
    touchPoints.clear();
  }
  
  if(showResultCounter > 0){
    showResultCounter--;
  }
  if(showResultCounter == 0){
    showResults = "";
    showInteraction = "";
  }

  visualization(touchPoint, qBottomCali.multiply(qBottom), topRotation, angle);
}

void visualization(int[] touchPoint, Quaternion bottom, Quaternion top, float angle) {
  pushMatrix();
  translate(3*width/4, height/2);
  scale(7);
  rotateY(PI);

  pushMatrix();
  fill(133);
  strokeWeight(0.3);
  applyQuaternion(bottom);
  translate(0, sectionHeight/2.0, sectionThick / 2.0);
  box(sectionWidth, sectionHeight, sectionThick);
  if(!reverse){
    translate(-sectionWidth/2.0, -sectionHeight/2.0, -sectionThick/2.0 - 2);
    for (int x=0; x<2; x++) {
      for (int y=0; y<2; y++) {
        if (touchPoint[4+2*x+y] == 1) {
          // show touch
          fill(198, 40, 40);
          ellipse(sectionWidth/2.0 + (2*x-1) * sectionWidth/4.0, sectionHeight/2.0  + (2*y-1) * sectionHeight/4.0, 10, 10);
        } else {
          // show untouch
          fill(100);
          ellipse(sectionWidth/2.0 + (2*x-1) * sectionWidth/4.0, sectionHeight/2.0  + (2*y-1) * sectionHeight/4.0, 10, 10);
        }
      }
    }
  }
  else{
    translate(-sectionWidth/2.0, -sectionHeight/2.0, sectionThick/2.0 + 2);
    for (int x=0; x<2; x++) {
      for (int y=0; y<2; y++) {
        if (touchPoint[4+2*x+1-y] == 1) {
          // show touch
          fill(198, 40, 40);
          ellipse(sectionWidth/2.0 + (2*x-1) * sectionWidth/4.0, sectionHeight/2.0 - (2*y-1) * sectionHeight/4.0, 10, 10);
        } else {
          // show untouch
          fill(100);
          ellipse(sectionWidth/2.0 + (2*x-1) * sectionWidth/4.0, sectionHeight/2.0 - (2*y-1) * sectionHeight/4.0, 10, 10);
        }
      }
    }
  }
  popMatrix();

  pushMatrix();
  fill(200);
  strokeWeight(0.3);
  applyQuaternion(bottom.multiply(top));
  translate(0, -sectionHeight/2.0, sectionThick / 2.0);
  box(sectionWidth, sectionHeight, sectionThick);
  if(!reverse){
    translate(-sectionWidth/2.0, -sectionHeight/2.0, -sectionThick/2.0 - 2);
    for (int x=0; x<2; x++) {
      for (int y=0; y<2; y++) {
        if (touchPoint[2*x+y] == 1) {
          // show touch
          fill(198, 40, 40);
          ellipse(sectionWidth/2.0 + (2*x-1) * sectionWidth/4.0, sectionHeight/2.0  + (2*y-1) * sectionHeight/4.0, 10, 10);
        } else {
          // show untouch
          fill(100);
          ellipse(sectionWidth/2.0 + (2*x-1) * sectionWidth/4.0, sectionHeight/2.0  + (2*y-1) * sectionHeight/4.0, 10, 10);
        }
      }
    }
  }
  else{
    translate(-sectionWidth/2.0, -sectionHeight/2.0, sectionThick/2.0 + 2);
    for (int x=0; x<2; x++) {
      for (int y=0; y<2; y++) {
        if (touchPoint[2*x+1-y] == 1) {
          // show touch
          fill(198, 40, 40);
          ellipse(sectionWidth/2.0 + (2*x-1) * sectionWidth/4.0, sectionHeight/2.0  - (2*y-1) * sectionHeight/4.0, 10, 10);
        } else {
          // show untouch
          fill(100);
          ellipse(sectionWidth/2.0 + (2*x-1) * sectionWidth/4.0, sectionHeight/2.0  - (2*y-1) * sectionHeight/4.0, 10, 10);
        }
      }
    }
  }
  popMatrix();
  popMatrix();

  textFont(boldFontLarge);
  textAlign(RIGHT, BOTTOM);
  fill(255);
  String sensor = "";
  if (!reverse) {
    if (angle > 20) {
      sensor = sensor + "inward\n" + abs(int(angle)) + "°\n";
    } else if (angle < -20) {
      sensor = sensor + "outward\n" + abs(int(angle)) + "°\n";
    } else {
      sensor = sensor + "\n" + abs(int(angle)) + "°\n";
    }
    touchPoints.sort();
    for (int i=0; i<touchPoints.size(); i++) {
      sensor = sensor + touchCoor[touchPoints.get(i)-1];
    }
  } else {
    if (angle > 20) {
      sensor = sensor + "outward\n" + abs(int(angle)) + "°\n";
    } else if (angle < -20) {
      sensor = sensor + "inward\n" + abs(int(angle)) + "°\n";
    } else {
      sensor = sensor + "\n" + abs(int(angle)) + "°\n";
    }
    touchPoints.sort();
    for (int i=0; i<touchPoints.size(); i++) {
      int tp = touchPoints.get(i)-1;
      if (tp % 2 == 0) {
        tp = tp + 1;
      } else {
        tp = tp - 1;
      }
      sensor = sensor + touchCoor[tp];
    }
  }
  text(sensor, 20, 50, width/2-20, height/2-50);
  textFont(normalFont);
  text(showInteraction, 20, 3*height/5, width/2-20, 122);
  textFont(boldFontSmall);
  text(showResults, 20, 3*height/5 + 122, width/2-20, 122);
}

void classify() {
  // check the gestureState, maximum angle and touchPoints
  String results = "";
  if (gestureState == 1) {
    showInteraction = "fold";
    if (foldAngleMax > 0) {
      results = results + "inward-";
    } else {
      results = results + "outward-";
    }
    if (abs(foldAngleMax) > 90) {
      results = results + "large";
    } else if (abs(foldAngleMax) > 45) {
      results = results + "medium";
    } else {
      results = results + "small";
    }
  } else if (gestureState == 2) {
  } else if (gestureState == 3) {
    if (prevGestureState == 1) {
      showInteraction = "fold-enhanced touch";
      if (foldAngleMax > 0) {
        results = results + "inward-";
      } else {
        results = results + "outward-";
      }
      if (abs(foldAngleMax) > 90) {
        results = results + "large-";
      } else if (abs(foldAngleMax) > 45) {
        results = results + "medium-";
      } else {
        results = results + "small-";
      }
      touchPoints.sort();
      for (int i=0; i<touchPoints.size(); i++) {
        results = results + touchCoor[touchPoints.get(i)-1];
      }
    } else if (prevGestureState == 2) {
      showInteraction = "touch-enhanced fold";
      if (!reverse) {
        if (foldAngleMax > 0) {
          results = results + "inward-";
        } else {
          results = results + "outward-";
        }
        touchPoints.sort();
        for (int i=0; i<touchPoints.size(); i++) {
          results = results + touchCoor[touchPoints.get(i)-1];
        }
      } else {
        if (foldAngleMax > 0) {
          results = results + "outward-";
        } else {
          results = results + "inward-";
        }
        touchPoints.sort();
        for (int i=0; i<touchPoints.size(); i++) {
          int tp = touchPoints.get(i)-1;
          if (tp % 2 == 0) {
            tp = tp + 1;
          } else {
            tp = tp - 1;
          }
          results = results + touchCoor[tp];
        }
      }
    }
  }

  showResults = results;
  showResultCounter = 120;
  gestureState = 4;
}

Quaternion quaternion_diff(Quaternion a, Quaternion b)
{
  Quaternion inv = a.getInverse();
  return inv.multiply(b);
}

void applyQuaternion(Quaternion a) {
  // Converts this quaternion to a rotation matrix.
  //
  // | 1 - 2(y^2 + z^2) 2(xy + wz) 2(xz - wy) 0 |
  // | 2(xy - wz) 1 - 2(x^2 + z^2) 2(yz + wx) 0 |
  // | 2(xz + wy) 2(yz - wx) 1 - 2(x^2 + y^2) 0 |
  // | 0 0 0 1 |

  float x2 = (float)a.getQ1() + (float)a.getQ1();
  float y2 = (float)a.getQ2() + (float)a.getQ2();
  float z2 = (float)a.getQ3() + (float)a.getQ3();
  float xx = (float)a.getQ1() * x2;
  float xy = (float)a.getQ1() * y2;
  float xz = (float)a.getQ1() * z2;
  float yy = (float)a.getQ2() * y2;
  float yz = (float)a.getQ2() * z2;
  float zz = (float)a.getQ3() * z2;
  float wx = (float)a.getQ0() * x2;
  float wy = (float)a.getQ0() * y2;
  float wz = (float)a.getQ0() * z2;

  applyMatrix(1 - (yy + zz), xy - wz, xz + wy, 0,
    xy + wz, 1 - (xx + zz), yz - wx, 0,
    xz - wy, yz + wx, 1 - (xx + yy), 0,
    0, 0, 0, 1);
}


public void calibration(int theValue) {
  println("calibration: " + currentState);
  qTopCali = new Quaternion(float(currentState[12]), -float(currentState[14]), float(currentState[13]), float(currentState[15])).getInverse();
  qBottomCali = new Quaternion(float(currentState[8]), -float(currentState[10]), float(currentState[9]), float(currentState[11])).getInverse();
  if (cp5Init) {
    state = 2;
  }
}

float toEulerAngles(Quaternion q) {
  // roll (x-axis rotation)
  double sinr_cosp = 2 * (q.getQ0() * q.getQ1() + q.getQ2() * q.getQ3());
  double cosr_cosp = 1 - 2 * (q.getQ1() * q.getQ1() + q.getQ2() * q.getQ2());
  float roll = atan2((float)sinr_cosp, (float)cosr_cosp);

  // pitch (y-axis rotation)
  double sinp = sqrt((float)(1 + 2 * (q.getQ0() * q.getQ2() - q.getQ1() * q.getQ3())));
  double cosp = sqrt((float)(1 - 2 * (q.getQ0() * q.getQ2() - q.getQ1() * q.getQ3())));
  float pitch = 2 * atan2((float)sinp, (float)cosp) - PI / 2;

  // yaw (z-axis rotation)
  double siny_cosp = 2 * (q.getQ0() * q.getQ3() + q.getQ1() * q.getQ2());
  double cosy_cosp = 1 - 2 * (q.getQ2() * q.getQ2() + q.getQ3() * q.getQ3());
  float yaw = atan2((float)siny_cosp, (float)cosy_cosp);

  //println(roll + ", " + yaw + ", " + pitch);
  //println(q.getQ0() + ", " + q.getQ1() + ", " + q.getQ2() + ", " + q.getQ3());

  return roll;
}

Quaternion toQuaternion(float roll, float yaw, float pitch) {
  float qx = sin(roll/2.0) * cos(pitch/2.0) * cos(yaw/2.0) - cos(roll/2.0) * sin(pitch/2.0) * sin(yaw/2.0);
  float qy = cos(roll/2.0) * sin(pitch/2.0) * cos(yaw/2.0) + sin(roll/2.0) * cos(pitch/2.0) * sin(yaw/2.0);
  float qz = cos(roll/2.0) * cos(pitch/2.0) * sin(yaw/2.0) - sin(roll/2.0) * sin(pitch/2.0) * cos(yaw/2.0);
  float qw = cos(roll/2.0) * cos(pitch/2.0) * cos(yaw/2.0) + sin(roll/2.0) * sin(pitch/2.0) * sin(yaw/2.0);
  return new Quaternion(qw, qx, qy, qz);
}


void serialEvent(Serial p) {
  try {
    if (state == 0) {
      // read from IMU
      String inByte = p.readString();
      print(inByte);

      // get connection
      try {
        if (inByte.equals("a\r\n")==true) {
          myPort.write("a");
          // go to calibration state, show button
          state = 1;
        }
      }
      catch(NullPointerException e) {
      }
    } else if (state == 1) {
      // read from IMU
      String inByte = p.readString();
      //print(inByte);

      // render
      if (inByte != null) {
        String[] pieces = split(inByte, ',');
        if (pieces.length == 16) {
          currentState = pieces;
        }
      }
    } else if (state == 2) {
      String inByte = p.readString();
      //print(inByte);

      // render
      if (inByte != null) {
        String[] pieces = split(inByte, ',');
        if (pieces.length == 16) {
          currentState = pieces;
        }
      }
    }
  }
  catch(RuntimeException e) {
    e.printStackTrace();
  }
}

void keyPressed() {
  if (key == CODED) {
    if (keyCode == ESC) {
      // stop program, stop serial and exit
      myPort.clear();
      myPort.stop();
      exit();
    }
  } else {
    if (key == 'r') {
      reverse = !reverse;
      print("reverse");
    }
  }
}
