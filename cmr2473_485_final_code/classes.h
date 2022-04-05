#include "QuadDecoder.h"
#include <Arduino.h>
#include <math.h>
#include <QTRSensors.h>
#include <Servo.h>



#define WHEEL_RADIUS 4.02
#define TICK_BOUNDS 300
#define TICKS_PER_REV 4741.44 // Number of ticks/counts per wheel revolution (this will depend on your motor)
#define PERIOD 20000     // How fast velocity is calculated in microseconds

#define RPMat_250PWMA 101

#define wKP 1.3
#define wKD .0005
#define wKI 8

#define LINEKP 0.002    // 0.002
#define LINEKD 0.0001   // 0.0001

#define PWM_MAX 235
#define PWM_MIN 23

#define SF 4
#define US_Pin1 17
#define US_Pin2 19
#define MotorA_nD2 0 // Enable Pin (Not nessecarry to connect if you use a jumper)
#define MotorA_DIR 8 // Direction Pin
#define MotorA_PWMA 7 // PWMA Pin

#define MotorB_nD2 0 // Enable Pin (Not nessecarry to connect if you use a jumper)
#define MotorB_DIR  10// Direction Pin
#define MotorB_PWMB  11// PWMA Pin

#define reflEmitterPin 15
#define reflL 12
#define reflM 13
#define reflR 14
// 20 21 pins xbee
#define SERVO_PIN 22

#define SLOPE 1.1
#define V_WHEEL_Target 25 // max around 60ish
#define SLOW_SPEED 15
#define LEFT -1
#define RIGHT 1
#define DEADZONE 0.1


#define WIDTH 20.5
double modulo(double in, double divisor);


class motor{
public:
    
    motor(uint32_t nd2, uint32_t dir, uint32_t pwm);
    QuadDecoder<1> Enc; //New Quadrature Encoder Object
    volatile int oldCntA = 0; // used to store the count the last time you calculated
    volatile int newCntA = 0; //  new value to use in calculating difference
    volatile int diffA = 0;   // difference between newCntA and oldCntA

    volatile double angPosA = 0.0;
    volatile double oldPWMA = 0.0;
    volatile double intPWMA = 0.0;
    volatile double RPMA = 0.0;
    volatile double VelA = 0.0;
    
    volatile double dispA = 0.0;
    volatile double desireddispA = 0.0;
    volatile double desiredDistA = 0.0;
    volatile double expecteddispA = 0.0;
    volatile double desiredRPMA=0;
    volatile int PWMA = 0;
    uint32_t Motor_nD2; // Enable Pin (Not nessecarry to connect if you use a jumper)
    uint32_t Motor_DIR; // Direction Pin
    uint32_t Motor_PWM;
    
    void ClosedLoop(double targetVel);
    void motorEnable();
    void motorGetInfo();
    void motorDisable();
};

class motor2{
public:
    
    motor2(uint32_t nd2, uint32_t dir, uint32_t pwm);
    QuadDecoder<2> Enc; //New Quadrature Encoder Object
    volatile int oldCntA = 0; // used to store the count the last time you calculated
    volatile int newCntA = 0; //  new value to use in calculating difference
    volatile int diffA = 0;   // difference between newCntA and oldCntA

    volatile double angPosA = 0.0;
    volatile double oldPWMA = 0.0;
    volatile double intPWMA = 0.0;
    volatile double RPMA = 0.0;
    volatile double VelA = 0.0;
    
    volatile double dispA = 0.0;
    volatile double desireddispA = 0.0;
    volatile double desiredDistA = 0.0;
    volatile double expecteddispA = 0.0;
    volatile double desiredRPMA=0;
    volatile int PWMA = 0;
    uint32_t Motor_nD2; // Enable Pin (Not nessecarry to connect if you use a jumper)
    uint32_t Motor_DIR; // Direction Pin
    uint32_t Motor_PWM;
    
    void ClosedLoop(double targetVel);
    void motorEnable();
    void motorGetInfo();
    void motorDisable();
};

class ultraSonic {
  public:
    void measure(void);
    double readCm(void);
    ultraSonic(u_int8_t pin);

  private:
    u_int8_t UPin;
    int time1;
    int time2;
};

class robot{

public:
    Servo s;
    ultraSonic USfront = ultraSonic(US_Pin1);
    ultraSonic USside = ultraSonic(US_Pin2);
private:
    
    
    QTRSensors qtr;

    motor MotorA = motor(MotorA_nD2, MotorA_DIR, MotorA_PWMA);
    motor2 MotorB = motor2(MotorB_nD2, MotorB_DIR, MotorB_PWMB);
    double VWHEELMAX = 70 * M_PI * 2 * WHEEL_RADIUS/60;
    double VWHEELMIN = 13 * M_PI * 2 * WHEEL_RADIUS/60;
    volatile double Vrobot=0;
    double Vtarget=VWHEELMAX;
    volatile double dthetaDt=0;
    
public:
    volatile int servoHeight = 550;
    volatile double theta=0.0;
    volatile double VaTarget=0;
    volatile double VbTarget=0;
    volatile double VaTargetOld=0;
    volatile double VbTargetOld=0;

    double pageWidth = 10000;
    double pageHeight = 75;
    double fontSize = 10;
    volatile double lineError = 0;
    volatile double lastLineError = 0;
    volatile double Voffset = 0;
private:
    int lineFollowLR = RIGHT;
    volatile double Vx;
    volatile double Vy;
    volatile double x=0.0;
    volatile double y=0.0;
    volatile double totalDistance= 0.0;
    volatile uint16_t sensorValues[3] = {0 ,0 ,0};

    
    int xletterCnt = 0;
    int ylineCnt = 0;
    
    
    
    void Telemetry_before(void);
public:    
    void printTelemetry();
    double getTheta(void);
    void calcVel(void);
    void begin(void);
    void idle(void);
    void arc(double radius, double finalAngle, int leftRight);
    void line(double angle, double distance);
    void locate(double xTarget, double yTarget, double finalAngle);
    void setZero();
    void changeangle(double fAngle);
    //movement
    void lineFollow();
    void turnLeft(double rad, double VwheelTarget);
    void turnRight(double rad, double VwheelTarget);
    void penDown();
    void penUp();
    void startWrite();
    void newline();
    void writeChar(char character);
    void writeString(char string[], int stringlength);
    void quickLine(double Localx, double Localy);


    int findLR(double NewAngle);
    void findLine(int believedSide);
    void goAround();

private:
    
    void letterA();
    void letterB();
    void letterC();
    void letterD(); 
    void letterE(); 
    void letterF(); 
    void letterG(); 
    void letterH(); 
    void letterI(); 
    void letterJ(); 
    void letterK(); 
    void letterL(); 
    void letterM(); 
    void letterN(); 
    void letterO(); 
    void letterP(); 
    void letterQ(); 
    void letterR(); 
    void letterS(); 
    void letterT(); 
    void letterU(); 
    void letterV(); 
    void letterW(); 
    void letterX(); 
    void letterY(); 
    void letterZ(); 
    void space();

};
double mag(double val1, double val2);
