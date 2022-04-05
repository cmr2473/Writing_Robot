#include "classes.h"

void ultraSonic::measure(void)
{
  delayMicroseconds(200);

  pinMode(UPin, OUTPUT);

  digitalWrite(UPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(UPin, LOW);

  delayMicroseconds(350);
  pinMode(UPin, INPUT);
  int count = 0;
  delayMicroseconds(100);

  //Serial.print("#1:   ");
  //Serial.println(digitalRead(UPin));

  while (!digitalReadFast(UPin))
  {
    //Serial.println(digitalReadFast(US_Pin));
    //Serial.println("wait high");
    //count++;
    delayNanoseconds(750);
  }

  time1 = micros();
  count = 0;

  //Serial.print("#2:   ");
  //Serial.println(digitalRead(UPin));

  while (digitalRead(UPin) && count < (18.5 * 2000))
  {
    delayNanoseconds(750);
    count++;
  }
  time2 = micros();

  //Serial.print("#3:   ");
  //Serial.println(digitalRead(UPin));

  if (count >= (18.5 * 2000))
  {
    time2 = time1;
  }
}
double ultraSonic::readCm(void)
{
  return (double)(time2 - time1) * 0.0174 - 1.7554;
}
ultraSonic::ultraSonic(u_int8_t pin)
{
  time1 = 0;
  time2 = 0;
  UPin = pin;
}

motor::motor(uint32_t nd2, uint32_t dir, uint32_t pwm)
{
  Motor_nD2 = nd2;
  Motor_DIR = dir;
  Motor_PWM = pwm;
  return;
}

void motor::motorEnable()
{
  digitalWrite(Motor_nD2, HIGH);
  return;
}

void motor::motorDisable()
{
  digitalWrite(Motor_nD2, LOW);
  return;
}
void motor::ClosedLoop(double targetVel)
{

  desiredRPMA = targetVel * 60 / (2 * 3.1415 * WHEEL_RADIUS);

  double feedbackPWMA = (map(abs(desiredRPMA), 0, 63, 0, 181) - map((abs(RPMA)), 0, 63, 0, 181));
  double deriPWMA = (feedbackPWMA - oldPWMA) / (PERIOD * 0.000001);
  intPWMA += (feedbackPWMA * PERIOD * 0.000001);

  PWMA = wKP * feedbackPWMA + wKI * intPWMA - wKD * deriPWMA;
  if (PWMA < PWM_MIN && PWMA != 0)
  {
    PWMA = PWM_MIN;
  }
  if (targetVel == 0)
  {
    PWMA = 0;
    intPWMA = 0;
  }
  oldPWMA = PWMA;

  if (PWMA < 0)
  {
    PWMA = 0;
  }
  if (PWMA > PWM_MAX)
  {
    PWMA = PWM_MAX;
  }

  if (desiredRPMA >= 0)
  {
    digitalWrite(Motor_DIR, 1);
  }
  else
  {
    digitalWrite(Motor_DIR, 0);
  }

  expecteddispA += desiredRPMA * (2 * 3.1415 * WHEEL_RADIUS) * PERIOD * 0.000001 / 60;
  analogWrite(Motor_PWM, PWMA);
  //Serial.printf("%f, %f, %f,%d,%f,%f, %f\n", angPosA, RPMA,dispA,PWMA, desiredRPMA,desireddispA,expecteddispA);

  return;
}

void motor::motorGetInfo()
{
  newCntA = Enc.getCount();
  if ((newCntA >= TICKS_PER_REV - TICK_BOUNDS && oldCntA <= TICK_BOUNDS))
  {
    //if(0){
    diffA = (newCntA - oldCntA) - TICKS_PER_REV;
  }
  else if ((oldCntA >= TICKS_PER_REV - TICK_BOUNDS && newCntA <= TICK_BOUNDS))
  {
    diffA = (newCntA - oldCntA) + TICKS_PER_REV;
  }
  else
  {
    diffA = newCntA - oldCntA;
  }

  angPosA = (newCntA * 360) / TICKS_PER_REV;
  RPMA = (diffA * 60 / (PERIOD * 0.000001)) / TICKS_PER_REV;
  dispA += RPMA * (2 * 3.1415 * WHEEL_RADIUS) * PERIOD * 0.000001 / 60;
  VelA = RPMA * (2 * 3.1415 * WHEEL_RADIUS) / 60;
  oldCntA = newCntA;
  return;
}
motor2::motor2(uint32_t nd2, uint32_t dir, uint32_t pwm)
{
  Motor_nD2 = nd2;
  Motor_DIR = dir;
  Motor_PWM = pwm;
  return;
}

void motor2::motorEnable()
{
  digitalWrite(Motor_nD2, HIGH);
  return;
}

void motor2::motorDisable()
{
  digitalWrite(Motor_nD2, LOW);
  return;
}
void motor2::ClosedLoop(double targetVel)
{

  desiredRPMA = targetVel * 60 / (2 * 3.1415 * WHEEL_RADIUS);

  double feedbackPWMA = (map(abs(desiredRPMA), 0, 63, 0, 181) - map((abs(RPMA)), 0, 63, 0, 181));
  double deriPWMA = (feedbackPWMA - oldPWMA) / (PERIOD * 0.000001);
  intPWMA += (feedbackPWMA * PERIOD * 0.000001);

  PWMA = wKP * feedbackPWMA + wKI * intPWMA - wKD * deriPWMA;
  if (PWMA < PWM_MIN && PWMA != 0)
  {
    PWMA = PWM_MIN;
  }
  if (targetVel == 0)
  {
    PWMA = 0;
    intPWMA = 0;
  }
  oldPWMA = PWMA;

  if (PWMA < 0)
  {
    PWMA = 0;
  }
  if (PWMA > PWM_MAX)
  {
    PWMA = PWM_MAX;
  }

  if (desiredRPMA >= 0)
  {
    digitalWrite(Motor_DIR, 1);
  }
  else
  {
    digitalWrite(Motor_DIR, 0);
  }

  expecteddispA += desiredRPMA * (2 * 3.1415 * WHEEL_RADIUS) * PERIOD * 0.000001 / 60;
  analogWrite(Motor_PWM, PWMA);
  //Serial.printf("%f, %f, %f,%d,%f,%f, %f\n", angPosA, RPMA,dispA,PWMA, desiredRPMA,desireddispA,expecteddispA);

  oldCntA = newCntA;
  return;
}

void motor2::motorGetInfo()
{
  newCntA = Enc.getCount();
  if ((newCntA >= TICKS_PER_REV - TICK_BOUNDS && oldCntA <= TICK_BOUNDS))
  {
    //if(0){
    diffA = (newCntA - oldCntA) - TICKS_PER_REV;
  }
  else if ((oldCntA >= TICKS_PER_REV - TICK_BOUNDS && newCntA <= TICK_BOUNDS))
  {
    diffA = (newCntA - oldCntA) + TICKS_PER_REV;
  }
  else
  {
    diffA = newCntA - oldCntA;
  }

  angPosA = (newCntA * 360) / TICKS_PER_REV;
  RPMA = (diffA * 60 / (PERIOD * 0.000001)) / TICKS_PER_REV;
  dispA += RPMA * (2 * 3.1415 * WHEEL_RADIUS) * PERIOD * 0.000001 / 60;
  VelA = RPMA * (2 * 3.1415 * WHEEL_RADIUS) / 60;
  return;
}

void robot::begin()
{
  MotorA.Enc.begin(TICKS_PER_REV);
  MotorB.Enc.begin(TICKS_PER_REV);
  // Pin Assignments
  //pinMode(MotorA_nD2, OUTPUT); // Enable Pin
  pinMode(MotorA_DIR, OUTPUT);  // Direction Pin
  pinMode(MotorA_PWMA, OUTPUT); // PWMA Pin

  //pinMode(MotorB_nD2, OUTPUT); // Enable Pin
  pinMode(MotorB_DIR, OUTPUT);  // Direction Pin
  pinMode(MotorB_PWMB, OUTPUT); // PWMA Pin
  // enable motor controller
  //digitalWrite(MotorA_nD2, HIGH);

  //s.attach(SERVO_PIN);
  //penUp();
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){reflL, reflM, reflR}, 3);
  qtr.setEmitterPin(reflEmitterPin);
  qtr.setTimeout(2500);

  for (uint16_t i = 0; i < 100; i++)
  {
    qtr.calibrate();
  }
  penUp();
  return;
}

void robot::idle(void)
{
  penUp();
  VaTarget = 0;
  VbTarget = 0;
}
double robot::getTheta()
{
  return theta;
}
void robot::Telemetry_before(void)
{

  MotorA.motorGetInfo(); // gets displacement RPM and Velocity info
  MotorB.motorGetInfo();

  Vrobot = (MotorA.VelA + MotorB.VelA) / 2;
  dthetaDt = (MotorA.VelA - MotorB.VelA) / WIDTH;
  theta = theta + dthetaDt * PERIOD * 0.000001;
  theta = modulo(theta, 2 * M_PI);

  Vx = sin(theta) * Vrobot;
  Vy = cos(theta) * Vrobot;

  x = x + Vx * PERIOD * 0.000001;
  y = y + Vy * PERIOD * 0.000001;
  totalDistance = totalDistance + (Vrobot)*PERIOD * 0.000001;

  //qtr.read(sensorValues);
  lineError = qtr.readLineBlack(sensorValues);
  lineError = lineError - 1000;

  Voffset = LINEKP * lineError - LINEKD * (lineError - lastLineError);// /(PERIOD*0.000005);
  lastLineError = lineError;

  if (abs(VaTarget) < DEADZONE && abs(VaTarget) != 0)
  {
    VaTarget = VaTarget * DEADZONE / abs(VaTarget);
  }
  if (abs(VbTarget) < DEADZONE && abs(VbTarget) != 0)
  {
    VbTarget = VbTarget * DEADZONE / abs(VbTarget);
  }
  USfront.measure();
  //USside.measure();
  if (MotorA.PWMA == VaTargetOld && VaTargetOld == PWM_MAX)
  {
    MotorA.intPWMA = 0;
  }
  if (MotorB.PWMA == VbTargetOld && VbTargetOld == PWM_MAX)
  {
    MotorB.intPWMA = 0;
  }

  VaTargetOld = MotorA.PWMA;
  VbTargetOld = MotorB.PWMA;
  return;
}
void robot::printTelemetry()
{
  //Serial.printf("%3f  %3f  %3f  %3f  %3f  %3f %3f  %3f\n",VaTarget, VbTarget, MotorA.VelA,MotorB.VelA,x,y,theta*180/M_PI, totalDistance);
  //Serial5.printf("%3f  %3f  %3f  %3f  %3f  %3f\n",MotorA.VelA,VaTarget,MotorB.VelA,VbTarget,Vrobot,theta*180/M_PI);
  //Serial.printf("%3.4f  %3.4f  %3.4f  %3.4f  %3.4f  %3.4f %3.4f %f %f %d %d %f\n", x, y, theta, MotorA.VelA, MotorB.VelA, VaTarget, VbTarget, lineError, USfront.readCm(), MotorA.PWMA, MotorB.PWMA, totalDistance);
  Serial5.printf("%3.4f  %3.4f  %3.4f  %3.4f  %3.4f  %3.4f %3.4f\n", x, y, theta, MotorA.VelA, MotorB.VelA, VaTarget, VbTarget);
}

void robot::calcVel()
{
  Telemetry_before();
  s.writeMicroseconds(servoHeight);
  MotorA.ClosedLoop(VaTarget);

  MotorB.ClosedLoop(VbTarget);

  return;
}

void robot::arc(double radius, double finalAngle, int LR)
{
  //offset abs offset abs bounding, delta ---- ah yes of course, I never would have thought
  double angleDel = (M_PI - abs(M_PI - abs(modulo(finalAngle, 2 * M_PI) - theta)));
  delayMicroseconds(PERIOD * 1.2);
  while ((M_PI - abs(M_PI - abs(modulo(finalAngle, 2 * M_PI) - theta))) > 0.015)
  { //this mathmatical function defines all things holy, the circle, but this time entire
    double VwheelTarget;

    if ((M_PI - abs(M_PI - abs(modulo(finalAngle, 2 * M_PI) - theta))) < 0.01)
    {
      VwheelTarget = 0;
    }
    else if (abs(M_PI - abs(M_PI - abs(modulo(finalAngle, 2 * M_PI) - theta))) < abs(0.5) / (2 * SLOPE * .25))
    {
      VwheelTarget = V_WHEEL_Target * (sin(((SLOPE * .25 * 3.141592) / (0.5)) * (M_PI - abs(M_PI - abs(modulo(finalAngle, 2 * M_PI) - theta)))));
    }
    else if ((M_PI - abs(M_PI - abs(modulo(finalAngle, 2 * M_PI) - theta))) < (1 / (2 * SLOPE * .25)) * abs(0.5))
    {
      VwheelTarget = -V_WHEEL_Target;
    }
    else
    {
      VwheelTarget = V_WHEEL_Target;
    }

    if (LR == RIGHT)
    {
      VaTarget = VwheelTarget;
      VbTarget = VaTarget * ((radius - WIDTH / 2) / (radius + WIDTH / 2));
    }
    else if (LR == LEFT)
    {
      VbTarget = VwheelTarget;
      VaTarget = VbTarget * ((radius - WIDTH / 2) / (radius + WIDTH / 2));
    }
    //Serial.printf("stuck in arc loop  %f\n", theta);
    delayMicroseconds(PERIOD * 1.2);
  }
  VaTarget = 0;
  VbTarget = 0;
  delay(100);
  return;
}

void robot::line(double angle, double distance)
{

  double distTag = totalDistance;
  distTag = distTag;
  delay(100);
  changeangle(angle);
  delay(100);
  double VwheelTarget = V_WHEEL_Target;

  double desiredDisp = distance + distTag;

  while (totalDistance - desiredDisp < 0)
  {
    if (abs(totalDistance - desiredDisp) < 0)
    {
      VaTarget = 0;
    }
    else if (abs(totalDistance - desiredDisp) < abs(distance) / (2 * SLOPE))
    {
      VaTarget = VwheelTarget * (-sin(((SLOPE * M_PI) / (distance)) * (totalDistance - desiredDisp)));
    }
    else if (totalDistance > (1 + 1 / (2 * SLOPE)) * distance + distTag)
    {
      VaTarget = -VwheelTarget;
    }
    else
    {
      VaTarget = VwheelTarget;
    }
    //Serial.println(totalDistance - desiredDisp);
    VbTarget = VaTarget;
    delayMicroseconds(PERIOD * 1.2);
  }
  VaTarget = 0;
  VbTarget = 0;
  delay(100);
  return;
}

void robot::locate(double xTarget, double yTarget, double finalAngle)
{
  double xChange = xTarget - x;
  double yChange = yTarget - y;

  quickLine(xChange, yChange);
  //int lr = findLR(finalAngle);
  delay(100);
  changeangle(finalAngle);
  delay(100);
  return;
}

void robot::setZero()
{
  theta = 0;
  x = 0;
  y = 0;
  totalDistance = 0;
}

int robot::findLR(double NewAngle)
{
  int LR;
  NewAngle = modulo(NewAngle, 2 * M_PI);
  delayMicroseconds(PERIOD * 1.2);
  if (modulo((NewAngle - theta), 2 * M_PI) - M_PI >= 0)
  {
    LR = LEFT;
  }
  else
  {
    LR = RIGHT;
  }
  //Serial.printf("%3f  %3f  %s\n",NewAngle, NewAngle-theta, LR==LEFT?"left":"right");
  return LR;
}

void robot::changeangle(double fAngle)
{
  int Lr = findLR(fAngle);
  arc(0, fAngle, Lr);
  delay(100);
  return;
}

void robot::lineFollow()
{
  //penUp();
  // Va is left wheel. Vb is right wheel
  // Va > Vb for right turn LR =1
  // sensors are L,M,R
  double VwheelTar = V_WHEEL_Target / 5;

  if (USfront.readCm() < 30)
  {
    VwheelTar = map(USfront.readCm(), 15, 30, SLOW_SPEED / 5, V_WHEEL_Target / 5); // MAY NEED TO ADJUST
  }
  if (USfront.readCm() < 15)
  {
    goAround();
  }

  VaTarget = VwheelTar * (1 + Voffset);
  VbTarget = VwheelTar * (1 - Voffset);

  /*
  if(sensorValues[0] > 750 && sensorValues[1] > 750 && sensorValues[2] < 750 ){//wwb
    turnRight(20,VwheelTar);
    lineFollowLR = RIGHT;
    delay(30);
  } else if(sensorValues[0] < 750 && sensorValues[1] > 750 && sensorValues[2] > 750 ){//bww
    turnLeft(20,VwheelTar);
    lineFollowLR = LEFT;
    delay(30);
  } else if(sensorValues[0] < 750 && sensorValues[1] < 750 && sensorValues[2] < 750 ){//bbb
    VbTarget = VwheelTar;
    VaTarget = VbTarget;
    delay(30);
  } else if(sensorValues[0] > 750 && sensorValues[1] < 750 && sensorValues[2] < 750 ){//wbb
    turnRight(40,VwheelTar);
    lineFollowLR = RIGHT;
    delay(30);
  } else if(sensorValues[0] < 750 && sensorValues[1] < 750 && sensorValues[2] > 750 ){//bbw
    turnLeft(40,VwheelTar);
    lineFollowLR = LEFT;
    delay(30);
  } else if(sensorValues[0] > 750 && sensorValues[1] < 750 && sensorValues[2] > 750 ){//wbw
    VbTarget = VwheelTar;
    VaTarget = VbTarget;
    delay(30);
  } else{//www bwb // if either, remeber old
    if(lineFollowLR == RIGHT){
      findLine(RIGHT);
      delay(30);
    } else{
      findLine(LEFT);
      delay(30);
    }
  }*/
  return;
}

void robot::turnLeft(double radius, double VwheelTarget)
{
  VbTarget = VwheelTarget;
  VaTarget = VbTarget * ((radius - WIDTH / 2) / (radius + WIDTH / 2));
  return;
}

void robot::turnRight(double radius, double VwheelTarget)
{
  VaTarget = VwheelTarget;
  VbTarget = VaTarget * ((radius - WIDTH / 2) / (radius + WIDTH / 2));
  return;
}

void robot::goAround()
{
  changeangle(theta + M_PI / 2);
  arc(30, theta - M_PI / 2, LEFT);
  //line(theta, 10);
  arc(30, theta -  M_PI / 2, LEFT);
  changeangle(theta + M_PI / 2);

  return;
}

void robot::findLine(int believedSide)
{
  while (sensorValues[0] > 750 && sensorValues[1] > 750 && sensorValues[2] > 750)
  { //www
    if (believedSide == RIGHT)
    {
      turnRight(10, SLOW_SPEED);
    }
    if (believedSide == LEFT)
    {
      turnLeft(10, SLOW_SPEED);
    }
    delayMicroseconds(PERIOD * 1.2);
  }
  return;
}

void robot::penUp()
{
  //Serial.println(s.read());
  servoHeight = 550;
  delay(100);
  //Serial.println(s.read());
  return;
}
void robot::penDown()
{
  //Serial.println(s.read());
  servoHeight = 1100;
  delay(100);
  //Serial.println(s.read());
  return;
}

void robot::startWrite()
{
  setZero();
  ylineCnt = 0;
  xletterCnt = 0;
  locate(fontSize, -3 * fontSize, M_PI / 2);
  return;
}

void robot::newline()
{
  ylineCnt++;
  xletterCnt = 0;
  locate(fontSize, -3 * fontSize * (ylineCnt + 1), M_PI / 2);
  return;
}

void robot::writeString(char string[], int stringlength)
{
  for (int i = 0; i < stringlength; i++)
  {
    if (x + 2.5 * fontSize > pageWidth)
    {
      newline();
    }
    if (y < -pageHeight + 1)
    {
      break;
    }

    writeChar(string[i]);
    delay(200);
  }
  return;
}

double modulo(double in, double divisor)
{
  double ret = fmod(in, divisor);

  if (ret < 0)
  {
    ret += divisor;
  }

  return ret;
}

void robot::writeChar(char Character)
{
  switch (Character)
  {
  case 'A':
    letterA();
    break;
  case 'B':
    letterB();
    break;
  case 'C':
    letterC();
    break;
  case 'D':
    letterD();
    break;
  case 'E':
    letterE();
    break;
  case 'F':
    letterF();
    break;
  case 'G':
    letterG();
    break;
  case 'H':
    letterH();
    break;
  case 'I':
    letterI();
    break;
  case 'J':
    letterJ();
    break;
  case 'K':
    letterK();
    break;
  case 'L':
    letterL();
    break;
  case 'M':
    letterM();
    break;
  case 'N':
    letterN();
    break;
  case 'O':
    letterO();
    break;
  case 'P':
    letterP();
    break;
  case 'Q':
    letterQ();
    break;
  case 'R':
    letterR();
    break;
  case 'S':
    letterS();
    break;
  case 'T':
    letterT();
    break;
  case 'U':
    letterU();
    break;
  case 'V':
    letterV();
    break;
  case 'W':
    letterW();
    break;
  case 'X':
    letterX();
    break;
  case 'Y':
    letterY();
    break;
  case 'Z':
    letterZ();
    break;
  case 'a':
    letterA();
    break;
  case 'b':
    letterB();
    break;
  case 'c':
    letterC();
    break;
  case 'd':
    letterD();
    break;
  case 'e':
    letterE();
    break;
  case 'f':
    letterF();
    break;
  case 'g':
    letterG();
    break;
  case 'h':
    letterH();
    break;
  case 'i':
    letterI();
    break;
  case 'j':
    letterJ();
    break;
  case 'k':
    letterK();
    break;
  case 'l':
    letterL();
    break;
  case 'm':
    letterM();
    break;
  case 'n':
    letterN();
    break;
  case 'o':
    letterO();
    break;
  case 'p':
    letterP();
    break;
  case 'q':
    letterQ();
    break;
  case 'r':
    letterR();
    break;
  case 's':
    letterS();
    break;
  case 't':
    letterT();
    break;
  case 'u':
    letterU();
    break;
  case 'v':
    letterV();
    break;
  case 'w':
    letterW();
    break;
  case 'x':
    letterX();
    break;
  case 'y':
    letterY();
    break;
  case 'z':
    letterZ();
    break;
  case ' ':
    space();
    break;
  default:

    break;
  }
  delay(200);
}

double mag(double val1, double val2)
{
  return sqrt(pow(val1, 2) + pow(val2, 2));
}
void robot::quickLine(double Localx, double Localy)
{
  line(atan2(Localx, Localy), mag(Localx, Localy));
}
void robot::letterA()
{

  locate((xletterCnt + 1) * 1.5 * fontSize + 0 * fontSize, -3 * fontSize * (ylineCnt + 1) + 0 * fontSize, M_PI / 2);
  penDown();
  line(atan2(.5, 2), mag(fontSize / 2, fontSize * 2));
  line(atan2(.5, -2), mag(fontSize / 2, fontSize * 2));
  penUp();

  line(atan2(-.75, 1), mag(fontSize * .75, fontSize * 1));
  penDown();
  quickLine(fontSize * .5, 0);

  penUp();
  xletterCnt++;
}
void robot::letterB()
{
  locate((xletterCnt + 1) * 1.5 * fontSize + 0 * fontSize, -3 * fontSize * (ylineCnt + 1) + 0 * fontSize, 2 * M_PI);
  penDown();
  quickLine(0, fontSize * 2);
  changeangle(M_PI / 2);
  arc(fontSize / 2, -M_PI / 2, RIGHT);
  changeangle(M_PI / 2);
  arc(fontSize / 2, -M_PI / 2, RIGHT);
  penUp();
  xletterCnt++;
}
void robot::letterC()
{
  locate((xletterCnt + 1) * 1.5 * fontSize + 1 * fontSize, -3 * fontSize * (ylineCnt + 1) + 0 * fontSize, -M_PI / 2);
  penDown();
  arc(fontSize, M_PI / 2, RIGHT);
  changeangle(M_PI);
  quickLine(0, -.25 * fontSize);
  penUp();
  xletterCnt++;
}
void robot::letterD()
{
  locate((xletterCnt + 1) * 1.5 * fontSize + 0 * fontSize, -3 * fontSize * (ylineCnt + 1) + 0 * fontSize, 0);
  penDown();
  quickLine(0, 2 * fontSize);
  changeangle(M_PI / 2);
  arc(fontSize, -M_PI / 2, RIGHT);
  penUp();
  xletterCnt++;
}
void robot::letterE()
{
  locate((xletterCnt + 1) * 1.5 * fontSize + 0 * fontSize, -3 * fontSize * (ylineCnt + 1) + 0 * fontSize, 0);
  penDown();
  locate((xletterCnt + 1) * 1.5 * fontSize + 0 * fontSize, -3 * fontSize * (ylineCnt + 1) + 2 * fontSize, M_PI / 2);
  locate((xletterCnt + 1) * 1.5 * fontSize + 1 * fontSize, -3 * fontSize * (ylineCnt + 1) + 2 * fontSize, M_PI);
  penUp();
  locate((xletterCnt + 1) * 1.5 * fontSize + .75 * fontSize, -3 * fontSize * (ylineCnt + 1) + 1 * fontSize, 0);
  penDown();
  locate((xletterCnt + 1) * 1.5 * fontSize + 0 * fontSize, -3 * fontSize * (ylineCnt + 1) + 1 * fontSize, 0);
  penUp();
  locate((xletterCnt + 1) * 1.5 * fontSize + 0 * fontSize, -3 * fontSize * (ylineCnt + 1) + 0 * fontSize, M_PI / 2);
  penDown();
  locate((xletterCnt + 1) * 1.5 * fontSize + 1 * fontSize, -3 * fontSize * (ylineCnt + 1) + 0 * fontSize, 0);
  penUp();
  xletterCnt++;
}
void robot::letterF()
{
  locate((xletterCnt + 1) * 1.5 * fontSize + 0 * fontSize, -3 * fontSize * (ylineCnt + 1) + 0 * fontSize, 0);
  penDown();
  locate((xletterCnt + 1) * 1.5 * fontSize + 0 * fontSize, -3 * fontSize * (ylineCnt + 1) + 2 * fontSize, M_PI / 2);
  locate((xletterCnt + 1) * 1.5 * fontSize + 1 * fontSize, -3 * fontSize * (ylineCnt + 1) + 2 * fontSize, M_PI);
  penUp();
  locate((xletterCnt + 1) * 1.5 * fontSize + .75 * fontSize, -3 * fontSize * (ylineCnt + 1) + 1 * fontSize, 0);
  penDown();
  locate((xletterCnt + 1) * 1.5 * fontSize + 0 * fontSize, -3 * fontSize * (ylineCnt + 1) + 1 * fontSize, 0);
  penUp();
  xletterCnt++;
}
void robot::letterG()
{
  locate((xletterCnt + 1) * 1.5 * fontSize + 0.5 * fontSize, -3 * fontSize * (ylineCnt + 1) + 0.5 * fontSize, 0);
  penDown();
  locate((xletterCnt + 1) * 1.5 * fontSize + 1 * fontSize, -3 * fontSize * (ylineCnt + 1) + 0.5 * fontSize, 0);
  locate((xletterCnt + 1) * 1.5 * fontSize + 1 * fontSize, -3 * fontSize * (ylineCnt + 1) + 0 * fontSize, -M_PI / 2);
  arc(fontSize, M_PI / 2, RIGHT);
  changeangle(M_PI);
  locate((xletterCnt + 1) * 1.5 * fontSize + 1 * fontSize, -3 * fontSize * (ylineCnt + 1) + 1.75 * fontSize, M_PI);
  penUp();
  xletterCnt++;
}
void robot::letterH()
{
  penDown();
  locate((xletterCnt + 1) * 1.5 * fontSize + 0 * fontSize, -3 * fontSize * (ylineCnt + 1) + 0 * fontSize, 0);
  locate((xletterCnt + 1) * 1.5 * fontSize + 0 * fontSize, -3 * fontSize * (ylineCnt + 1) + 2 * fontSize, 0);
  penUp();
  locate((xletterCnt + 1) * 1.5 * fontSize + 0 * fontSize, -3 * fontSize * (ylineCnt + 1) + 1 * fontSize, M_PI / 2);
  penDown();
  locate((xletterCnt + 1) * 1.5 * fontSize + 1 * fontSize, -3 * fontSize * (ylineCnt + 1) + 1 * fontSize, 0);
  penUp();
  locate((xletterCnt + 1) * 1.5 * fontSize + 1 * fontSize, -3 * fontSize * (ylineCnt + 1) + 2 * fontSize, M_PI);
  penDown();
  locate((xletterCnt + 1) * 1.5 * fontSize + 1 * fontSize, -3 * fontSize * (ylineCnt + 1) + 0 * fontSize, M_PI);
  penUp();
  xletterCnt++;
}
void robot::letterI()
{
  locate((xletterCnt + 1) * 1.5 * fontSize + 0 * fontSize, -3 * fontSize * (ylineCnt + 1) + 0 * fontSize, M_PI / 2);
  penDown();
  locate((xletterCnt + 1) * 1.5 * fontSize + 1 * fontSize, -3 * fontSize * (ylineCnt + 1) + 0 * fontSize, 0);
  penUp();
  locate((xletterCnt + 1) * 1.5 * fontSize + 1 * fontSize, -3 * fontSize * (ylineCnt + 1) + 2 * fontSize, -M_PI / 2);
  penDown();
  locate((xletterCnt + 1) * 1.5 * fontSize + 0 * fontSize, -3 * fontSize * (ylineCnt + 1) + 2 * fontSize, M_PI / 2);
  penUp();
  locate((xletterCnt + 1) * 1.5 * fontSize + 0.5 * fontSize, -3 * fontSize * (ylineCnt + 1) + 2 * fontSize, M_PI);
  penDown();
  locate((xletterCnt + 1) * 1.5 * fontSize + 0.5 * fontSize, -3 * fontSize * (ylineCnt + 1) + 0 * fontSize, M_PI / 2);
  penUp();
  xletterCnt++;
}
void robot::letterJ()
{
  locate((xletterCnt + 1) * 1.5 * fontSize + 0 * fontSize, -3 * fontSize * (ylineCnt + 1) + 0.5 * fontSize, M_PI);
  penDown();
  arc(fontSize / 2, 0, LEFT);
  locate((xletterCnt + 1) * 1.5 * fontSize + 1 * fontSize, -3 * fontSize * (ylineCnt + 1) + 2 * fontSize, -M_PI / 2);
  locate((xletterCnt + 1) * 1.5 * fontSize + 0 * fontSize, -3 * fontSize * (ylineCnt + 1) + 2 * fontSize, M_PI);
  locate((xletterCnt + 1) * 1.5 * fontSize + 0 * fontSize, -3 * fontSize * (ylineCnt + 1) + 1.75 * fontSize, M_PI);
  penUp();
  xletterCnt++;
}
void robot::letterK()
{
  locate((xletterCnt + 1) * 1.5 * fontSize + 0 * fontSize, -3 * fontSize * (ylineCnt + 1) + 0 * fontSize, 0);
  penDown();
  locate((xletterCnt + 1) * 1.5 * fontSize + 0 * fontSize, -3 * fontSize * (ylineCnt + 1) + 2 * fontSize, 0);
  penUp();
  locate((xletterCnt + 1) * 1.5 * fontSize + 1 * fontSize, -3 * fontSize * (ylineCnt + 1) + 2 * fontSize, M_PI);
  penDown();
  locate((xletterCnt + 1) * 1.5 * fontSize + 0 * fontSize, -3 * fontSize * (ylineCnt + 1) + 1 * fontSize, M_PI);
  locate((xletterCnt + 1) * 1.5 * fontSize + 1 * fontSize, -3 * fontSize * (ylineCnt + 1) + 0 * fontSize, M_PI / 2);
  penUp();
  xletterCnt++;
}
void robot::letterL()
{
  locate((xletterCnt + 1) * 1.5 * fontSize + 0 * fontSize, -3 * fontSize * (ylineCnt + 1) + 2 * fontSize, M_PI);
  penDown();
  locate((xletterCnt + 1) * 1.5 * fontSize + 0 * fontSize, -3 * fontSize * (ylineCnt + 1) + 0 * fontSize, M_PI);
  locate((xletterCnt + 1) * 1.5 * fontSize + 1 * fontSize, -3 * fontSize * (ylineCnt + 1) + 0 * fontSize, M_PI);
  penUp();
  xletterCnt++;
}
void robot::letterM()
{
  locate((xletterCnt + 1) * 1.5 * fontSize + 0 * fontSize, -3 * fontSize * (ylineCnt + 1) + 0 * fontSize, 0);
  penDown();
  locate((xletterCnt + 1) * 1.5 * fontSize + 0 * fontSize, -3 * fontSize * (ylineCnt + 1) + 2 * fontSize, M_PI / 2);
  locate((xletterCnt + 1) * 1.5 * fontSize + 0.5 * fontSize, -3 * fontSize * (ylineCnt + 1) + 1 * fontSize, 0);
  locate((xletterCnt + 1) * 1.5 * fontSize + 1 * fontSize, -3 * fontSize * (ylineCnt + 1) + 2 * fontSize, M_PI);
  locate((xletterCnt + 1) * 1.5 * fontSize + 1 * fontSize, -3 * fontSize * (ylineCnt + 1) + 0 * fontSize, 0);
  penUp();
  xletterCnt++;
}
void robot::letterN()
{
  locate((xletterCnt + 1) * 1.5 * fontSize + 0 * fontSize, -3 * fontSize * (ylineCnt + 1) + 0 * fontSize, 0);
  penDown();
  locate((xletterCnt + 1) * 1.5 * fontSize + 0 * fontSize, -3 * fontSize * (ylineCnt + 1) + 2 * fontSize, 0);
  locate((xletterCnt + 1) * 1.5 * fontSize + 1 * fontSize, -3 * fontSize * (ylineCnt + 1) + 0 * fontSize, 0);
  locate((xletterCnt + 1) * 1.5 * fontSize + 1 * fontSize, -3 * fontSize * (ylineCnt + 1) + 2 * fontSize, 0);
  penUp();
  xletterCnt++;
}
void robot::letterO()
{
  locate((xletterCnt + 1) * 1.5 * fontSize + 0 * fontSize, -3 * fontSize * (ylineCnt + 1) + 1.5 * fontSize, 0);
  penDown();
  arc(fontSize / 2, M_PI, RIGHT);
  locate((xletterCnt + 1) * 1.5 * fontSize + 1 * fontSize, -3 * fontSize * (ylineCnt + 1) + .5 * fontSize, M_PI);
  arc(fontSize / 2, M_PI, RIGHT);
  locate((xletterCnt + 1) * 1.5 * fontSize + 0 * fontSize, -3 * fontSize * (ylineCnt + 1) + 1.5 * fontSize, 0);
  penUp();
  xletterCnt++;
}
void robot::letterP()
{
  locate((xletterCnt + 1) * 1.5 * fontSize + 0 * fontSize, -3 * fontSize * (ylineCnt + 1) + 0 * fontSize, 0);
  penDown();
  locate((xletterCnt + 1) * 1.5 * fontSize + 0 * fontSize, -3 * fontSize * (ylineCnt + 1) + 2 * fontSize, M_PI / 2);
  arc(fontSize / 2, -M_PI / 2, RIGHT);
  penUp();
  xletterCnt++;
}
void robot::letterQ()
{
  locate((xletterCnt + 1) * 1.5 * fontSize + 0 * fontSize, -3 * fontSize * (ylineCnt + 1) + 1.5 * fontSize, 0);
  penDown();
  arc(fontSize / 2, M_PI, RIGHT);
  locate((xletterCnt + 1) * 1.5 * fontSize + 1 * fontSize, -3 * fontSize * (ylineCnt + 1) + .5 * fontSize, M_PI);
  arc(fontSize / 2, M_PI, RIGHT);
  locate((xletterCnt + 1) * 1.5 * fontSize + 0 * fontSize, -3 * fontSize * (ylineCnt + 1) + 1.5 * fontSize, 0);
  penUp();
  locate((xletterCnt + 1) * 1.5 * fontSize + .5 * fontSize, -3 * fontSize * (ylineCnt + 1) + .75 * fontSize, M_PI / 2);
  penDown();
  locate((xletterCnt + 1) * 1.5 * fontSize + 1 * fontSize, -3 * fontSize * (ylineCnt + 1) + 0 * fontSize, M_PI / 2);
  penUp();
  xletterCnt++;
}
void robot::letterR()
{
  locate((xletterCnt + 1) * 1.5 * fontSize + 0 * fontSize, -3 * fontSize * (ylineCnt + 1) + 0 * fontSize, 0);
  penDown();
  locate((xletterCnt + 1) * 1.5 * fontSize + 0 * fontSize, -3 * fontSize * (ylineCnt + 1) + 2 * fontSize, M_PI / 2);
  arc(fontSize / 2, -M_PI / 2, RIGHT);
  locate((xletterCnt + 1) * 1.5 * fontSize + 1 * fontSize, -3 * fontSize * (ylineCnt + 1) + 0 * fontSize, M_PI / 2);
  penUp();
  xletterCnt++;
}
void robot::letterS()
{
  locate((xletterCnt + 1) * 1.5 * fontSize + 0 * fontSize, -3 * fontSize * (ylineCnt + 1) + 0 * fontSize, M_PI / 2);
  penDown();
  locate((xletterCnt + 1) * 1.5 * fontSize + 0.5 * fontSize, -3 * fontSize * (ylineCnt + 1) + 0 * fontSize, 0);
  arc(fontSize / 2, -M_PI / 2, LEFT);
  arc(fontSize / 2, M_PI / 2, RIGHT);
  locate((xletterCnt + 1) * 1.5 * fontSize + 1 * fontSize, -3 * fontSize * (ylineCnt + 1) + 2 * fontSize, M_PI);
  locate((xletterCnt + 1) * 1.5 * fontSize + 1 * fontSize, -3 * fontSize * (ylineCnt + 1) + 1.75 * fontSize, M_PI);
  penUp();
  xletterCnt++;
}
void robot::letterT()
{
  locate((xletterCnt + 1) * 1.5 * fontSize + 0.5 * fontSize, -3 * fontSize * (ylineCnt + 1) + 0 * fontSize, 0);
  penDown();
  locate((xletterCnt + 1) * 1.5 * fontSize + 0.5 * fontSize, -3 * fontSize * (ylineCnt + 1) + 2 * fontSize, -M_PI / 2);
  penUp();
  locate((xletterCnt + 1) * 1.5 * fontSize + 0 * fontSize, -3 * fontSize * (ylineCnt + 1) + 2 * fontSize, M_PI / 2);
  penDown();
  locate((xletterCnt + 1) * 1.5 * fontSize + 1 * fontSize, -3 * fontSize * (ylineCnt + 1) + 2 * fontSize, M_PI / 2);
  penUp();
  xletterCnt++;
}
void robot::letterU()
{
  locate((xletterCnt + 1) * 1.5 * fontSize + 0 * fontSize, -3 * fontSize * (ylineCnt + 1) + 2 * fontSize, M_PI);
  penDown();
  locate((xletterCnt + 1) * 1.5 * fontSize + 0 * fontSize, -3 * fontSize * (ylineCnt + 1) + .5 * fontSize, M_PI);
  arc(0.5 * fontSize, 0, LEFT);
  locate((xletterCnt + 1) * 1.5 * fontSize + 1 * fontSize, -3 * fontSize * (ylineCnt + 1) + 2 * fontSize, 0);
  penUp();
  xletterCnt++;
}
void robot::letterV()
{
  locate((xletterCnt + 1) * 1.5 * fontSize + 0 * fontSize, -3 * fontSize * (ylineCnt + 1) + 2 * fontSize, M_PI / 2);
  penDown();
  locate((xletterCnt + 1) * 1.5 * fontSize + 0.5 * fontSize, -3 * fontSize * (ylineCnt + 1) + 0 * fontSize, M_PI / 2);
  locate((xletterCnt + 1) * 1.5 * fontSize + 1 * fontSize, -3 * fontSize * (ylineCnt + 1) + 2 * fontSize, 0);
  penUp();
}
void robot::letterW()
{
  locate((xletterCnt + 1) * 1.5 * fontSize + 0 * fontSize, -3 * fontSize * (ylineCnt + 1) + 2 * fontSize, M_PI);
  penDown();
  locate((xletterCnt + 1) * 1.5 * fontSize + 0.25 * fontSize, -3 * fontSize * (ylineCnt + 1) + 0 * fontSize, M_PI / 2);
  locate((xletterCnt + 1) * 1.5 * fontSize + 0.5 * fontSize, -3 * fontSize * (ylineCnt + 1) + 1 * fontSize, 0);
  locate((xletterCnt + 1) * 1.5 * fontSize + 0.75 * fontSize, -3 * fontSize * (ylineCnt + 1) + 0 * fontSize, M_PI / 2);
  locate((xletterCnt + 1) * 1.5 * fontSize + 1 * fontSize, -3 * fontSize * (ylineCnt + 1) + 2 * fontSize, 0);
  penUp();
  xletterCnt++;
}
void robot::letterX()
{
  locate((xletterCnt + 1) * 1.5 * fontSize + 0 * fontSize, -3 * fontSize * (ylineCnt + 1) + 0 * fontSize, 0);
  penDown();
  locate((xletterCnt + 1) * 1.5 * fontSize + 1 * fontSize, -3 * fontSize * (ylineCnt + 1) + 2 * fontSize, -M_PI / 2);
  penUp();
  locate((xletterCnt + 1) * 1.5 * fontSize + 0 * fontSize, -3 * fontSize * (ylineCnt + 1) + 2 * fontSize, M_PI);
  penDown();
  locate((xletterCnt + 1) * 1.5 * fontSize + 1 * fontSize, -3 * fontSize * (ylineCnt + 1) + 0 * fontSize, M_PI / 2);
  penUp();
  xletterCnt++;
}
void robot::letterY()
{
  locate((xletterCnt + 1) * 1.5 * fontSize + 0 * fontSize, -3 * fontSize * (ylineCnt + 1) + 2 * fontSize, M_PI / 2);
  penDown();
  locate((xletterCnt + 1) * 1.5 * fontSize + 0.5 * fontSize, -3 * fontSize * (ylineCnt + 1) + 1 * fontSize, M_PI / 2);
  locate((xletterCnt + 1) * 1.5 * fontSize + 1 * fontSize, -3 * fontSize * (ylineCnt + 1) + 2 * fontSize, 0);
  penUp();
  locate((xletterCnt + 1) * 1.5 * fontSize + 0.5 * fontSize, -3 * fontSize * (ylineCnt + 1) + 1 * fontSize, M_PI);
  penDown();
  locate((xletterCnt + 1) * 1.5 * fontSize + 0.5 * fontSize, -3 * fontSize * (ylineCnt + 1) + 0 * fontSize, M_PI / 2);
  penUp();
  xletterCnt++;
}
void robot::letterZ()
{
  locate((xletterCnt + 1) * 1.5 * fontSize + 0 * fontSize, -3 * fontSize * (ylineCnt + 1) + 2 * fontSize, M_PI / 2);
  penDown();
  locate((xletterCnt + 1) * 1.5 * fontSize + 1 * fontSize, -3 * fontSize * (ylineCnt + 1) + 2 * fontSize, M_PI);
  locate((xletterCnt + 1) * 1.5 * fontSize + 0 * fontSize, -3 * fontSize * (ylineCnt + 1) + 0 * fontSize, M_PI / 2);
  locate((xletterCnt + 1) * 1.5 * fontSize + 1 * fontSize, -3 * fontSize * (ylineCnt + 1) + 0 * fontSize, M_PI / 2);
  penUp();
  xletterCnt++;
}
void robot::space()
{
  xletterCnt++;
}