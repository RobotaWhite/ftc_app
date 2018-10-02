package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Robota5149 {

    public Robota5149(HardwareMap hwMap){
        hardwareMap = hwMap;
    }
    public HardwareMap hardwareMap;
    public DcMotor motorFrontRight;
    public DcMotor motorFrontLeft;
    public DcMotor motorRearRight;
    public DcMotor motorRearLeft;
    public DcMotor motorArm;
    public DcMotor motorSlide;

    public Motors leftMotors;
    public Motors rightMotors;

    public Servo servoColorSensor;
    public Servo servoSlideRotate;
    public Servo servoSlideGripper;
    public Servo servoGripper1;
    public Servo servoGripper2;

    public double gripperPosition=0;
    public double rotaterPosition=0;
    public double servoGripperPosition=0;
    public double armPosition=0;
    public double slidePosition=0;


    ModernRoboticsI2cGyro gyro;                    // Additional Gyro device
    ModernRoboticsI2cColorSensor colorsensorbottom;
    ModernRoboticsI2cColorSensor colorsensortop;
    public double COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    public double DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    public double WHEEL_DIAMETER_INCHES   = 6.0 ;     // For figuring circumference
    public double gyroOffset =0.0;
    public double COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    private double SteerAngle;


    private double scaleInput(double dVal)  {
        //this should provide a similar stick output to the original scaleInput method but cleans up the code significantly
        double dScaled;
        //dScaled = Math.pow(1.0227*Math.abs(dVal),2);
        dScaled = (1.05*Math.abs(dVal))+(-2.91*Math.pow(Math.abs(dVal),2))+(2.85*Math.pow(Math.abs(dVal),3));
        if (dScaled < 0.05){
            dScaled = 0;
        }
        if (dVal < 0) {
            dScaled = -dScaled;
        }
        return dScaled;
    }

    public void init() {
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorRearRight = hardwareMap.dcMotor.get("motorRearRight");
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorRearLeft = hardwareMap.dcMotor.get("motorRearLeft");

        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftMotors = new Motors( motorFrontLeft,motorRearLeft);
        rightMotors = new Motors(motorFrontRight,motorRearRight);
        colorsensorbottom = hardwareMap.get(ModernRoboticsI2cColorSensor.class,"ColorSensor");
        motorArm  = hardwareMap.dcMotor.get("motorArm");
        motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        servoGripper1 = hardwareMap.get(Servo.class, "servoGripper1");
        servoGripper2 = hardwareMap.get(Servo.class, "servoGripper2");
        servoColorSensor = hardwareMap.get(Servo.class, "BallServo");
        //servoColorSensorCommand(true);
        motorSlide = hardwareMap.dcMotor.get("motorSlide");
        motorSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        servoSlideRotate = hardwareMap.get(Servo.class, "servoSlideRotate");
        //servoSlideRotateCommand(true);
        servoSlideGripper = hardwareMap.get(Servo.class, "servoSlideGripper");
       //servoSlideGripperCommand(true);
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
        gyro.calibrate();

        for (   int idx =0; idx<1000; idx++){
            gyroOffset+=gyro.getAngularVelocity(AngleUnit.DEGREES).zRotationRate;
        }
        gyroOffset=gyroOffset/1000;

        //reverse the left wheels to make everything move forward
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorRearRight.setDirection(DcMotor.Direction.REVERSE);
    }

    public void InitServos() {

        gripperPosition=0;
        rotaterPosition=0;
        servoGripperPosition=0;

        servoGripperCommand(false,false,0);
        servoSlideGripperCommand(false,false,0);
        servoSlideRotateCommand(false,false,0);
        servoColorSensorCommand(true);
    }

    private double conditionStick(double stick) {
        stick = Range.clip(stick, -1, 1);
        stick = (float) scaleInput(stick);
        return stick; //negate stick to make pushing forward on the stick return a positive value
    }

    public void driveTank(double leftStick, double rightStick, boolean precision, int level) {
        if (precision){
            if(Math.abs(leftStick - rightStick)>0.3){
                level=(int)((double)level*1.5);
            }
            leftStick=leftStick*level/100;
            rightStick=rightStick*level/100;
        }
        // write the values to the motors
        leftMotors.setPower(conditionStick(leftStick));
        rightMotors.setPower(conditionStick(rightStick));
    }

    public void servoColorSensorCommand(boolean up){
        if (up){
            servoColorSensor.setPosition(1);
        }
        else {
            servoColorSensor.setPosition(.4);
        }
    }
    public void servoGripperCommand(boolean up, boolean down, double inc){
        if (up){
            servoGripperPosition+=inc;
        }
        else if (down){
            servoGripperPosition-=inc;
        }
        if (servoGripperPosition >0.8) {
            servoGripperPosition = 0.8;
        }
        if (servoGripperPosition <0) {
            servoGripperPosition = 0;
        }
        servoGripper1.setPosition(1-servoGripperPosition);
        servoGripper2.setPosition(servoGripperPosition);
    }

    public void armCommand (boolean up, boolean down, double inc){

        if (down){
            armPosition+=inc;
        }
        else if (up){
            armPosition-=inc;
        }
        if (armPosition >0) {
            armPosition = 0;
        }
        if (armPosition <-1000) {
            armPosition = -1000;
        }
        motorArm.setTargetPosition((int)armPosition);

        motorArm.setPower(1);
    }

    public void slideCommand (boolean out, boolean in, double inc){

        if (in){
            slidePosition+=inc;
        }
        else if (out){
            slidePosition-=inc;
        }
        if (slidePosition >0) {
            slidePosition = 0;
        }
        if (slidePosition <-8000) {
            slidePosition = -8000;
        }
        motorSlide.setTargetPosition((int)slidePosition);

        motorSlide.setPower(1);
    }

    public void servoSlideGripperCommand(boolean up, boolean down, double inc){
        if (up){
            gripperPosition+=inc;
        }
        else if (down){
            gripperPosition-=inc;
        }
        if (gripperPosition >1) {
            gripperPosition = 1;
        }
        if (gripperPosition <0) {
            gripperPosition = 0;
        }
       servoSlideGripper.setPosition(gripperPosition);
    }
    public void servoSlideRotateCommand(boolean up, boolean down, double inc){
        if (up){
            rotaterPosition+=inc;
        }
        else if (down){
            rotaterPosition-=inc;
        }
        if (rotaterPosition >1) {
            rotaterPosition = 1;
        }
        if (rotaterPosition <0) {
            rotaterPosition = 0;
        }
        servoSlideRotate.setPosition(rotaterPosition);
    }
    public void servoSlideGripperCommand(boolean up){
        if (up){
            gripperPosition=1;
        }
        else {
            gripperPosition=0;
        }

        servoSlideGripper.setPosition(gripperPosition);
    }
    public void servoSlideRotateCommand(boolean up){
        if (up){
            rotaterPosition=1;
        }
        else {
            rotaterPosition=1;
        }

        servoSlideRotate.setPosition(rotaterPosition);
    }
    public void driveTankGyro(double leftStick, double rightStick, boolean precision, int level) {


        if (precision){
            leftStick=leftStick*level/100;
            rightStick=rightStick*level/100;
        }

        SteerAngle = SteerAngle +(conditionStick(leftStick) - conditionStick(rightStick));
        double MotorPower = (conditionStick(leftStick) + conditionStick(rightStick)) / 2;
        double error = gyro.getIntegratedZValue() - SteerAngle;  //when error is negative you should turn clockwise to correct
        // write the values to the motors

        double leftpower = MotorPower*0.9 - error*0.1;
        double rightpower = MotorPower*0.9 + error*0.1;
        leftMotors.setPower(leftpower);
        rightMotors.setPower(rightpower);
    }
    public void driveMecanum(double xStick, double yStick, double zStick, double level) {

        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorRearRight.setDirection(DcMotor.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorRearLeft.setDirection(DcMotor.Direction.REVERSE);

        double xyStick= Math.abs(xStick)+Math.abs(yStick);
        if (xyStick==0){
            xyStick=0.0001;
        }

        double zscaled = conditionStick((zStick*level/100.0));
        double distance=(Math.sqrt(Math.pow(yStick,2)+Math.pow(xStick,2)));
        double Denominator = distance+Math.abs(zscaled);
        if (Denominator == 0){
            Denominator = 0.0001;
        }
        double xypower= conditionStick((distance/Denominator)*distance);
        double xcommand=(xStick/xyStick)*xypower;
        double ycommand=(yStick/xyStick)*xypower;
        double zcommand= ((Math.abs(zscaled)/Denominator)* zscaled);
        // write the values to the motors
        motorRearRight.setPower((xcommand + ycommand + zcommand));
        motorRearLeft.setPower((xcommand+(-ycommand)+zcommand));
        motorFrontRight.setPower(((-xcommand)+ycommand+zcommand));
        motorFrontLeft.setPower(((-xcommand)+(-ycommand)+zcommand));

    }
    public void driveMecanumGYRO (double xStick, double yStick, double zStick, double level){
        double angle = 0;
        double P = 0.0004;
        double I = 0.0007;
        double D = 0.0;
        double F = 0.0172;
        MiniPID miniPID;
        miniPID = new MiniPID(P,I,D,F);
        miniPID.setOutputLimits(1);
        double actual=0;
        double output=0;
        angle = zStick*45;

        output = miniPID.getOutput(actual, angle);
        actual = -(gyro.getAngularVelocity(AngleUnit.DEGREES).zRotationRate - gyroOffset);

        //handle the driving
        driveMecanum(xStick,
                yStick,
                output,
                level);
    }
}