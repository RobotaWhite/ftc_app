package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

public class Robot {


    public Robot(LinearOpMode linearOpMode) {
        lop = linearOpMode;
        hardwareMap = lop.hardwareMap;
    }
    public Robot(HardwareMap Hwmap) {
        hardwareMap = Hwmap;
    }

    //set these accessible from outside since we only read from them
    public HardwareMap hardwareMap;
    //public ModernRoboticsI2cRangeSensor rangeSensor
    public ColorSensor colorSensor;
    public ModernRoboticsI2cGyro gyro;
    //public Motors leftMotors;
    //public Motors rightMotors;
    public boolean lastLeftTranslate;
    public boolean lastRightTranslate;
    public boolean onsIntake;


    //set these inaccessible from outside to force using an interface
    private LinearOpMode lop;
    private DcMotor motorRearRight;
    private DcMotor motorRearLeft;
    private DcMotor intakeLeft;
    private DcMotor intakeRight;
    public DcMotor blockDump;
    private DcMotor motor_winch1;
    private DcMotor motor_winch2;
    private DcMotor motorArm;

    private Servo servoColor;
    private Servo servoBall;
    private Servo blockLock;
    private Servo relicServo;

    VuforiaLocalizer vuforia;

    private double servoColorPosition;
    private double relicServoPosition;

    int cameraMonitorViewId;
    VuforiaLocalizer.Parameters parameters;

    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;

    toggle intakeMotorToggle = new toggle();
    toggle hoverToggle = new toggle();

    static final double COUNTS_PER_MOTOR_REV = 560;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 2.25;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    private double scaleInput(double dVal) {
        //this should provide a similar stick output to the original scaleInput method but cleans up the code significantly
        double dScaled;
        dScaled = Math.pow(1.0227 * Math.abs(dVal), 2);
        if (dScaled < 0.05) {
            dScaled = 0;
        }
        if (dVal < 0) {
            dScaled = -dScaled;
        }
        return dScaled;
    }

    private double conditionStick(double stick) {
        stick = Range.clip(stick, -1, 1);
        stick = (float) scaleInput(stick);
        return -stick;
    }

    public void init() {
        motorRearRight = hardwareMap.dcMotor.get("right_drive");
        motorRearLeft = hardwareMap.dcMotor.get("left_drive");
        intakeLeft = hardwareMap.dcMotor.get("intakeLeft");
        intakeRight = hardwareMap.dcMotor.get("intakeRight");
        blockDump = hardwareMap.dcMotor.get("blockDump");
        motor_winch1 = hardwareMap.dcMotor.get("motor_winch1");
        motor_winch2 = hardwareMap.dcMotor.get("motor_winch2");
        motorArm = hardwareMap.dcMotor.get("motorArm");

        servoColor = hardwareMap.servo.get("servoColor");
        servoBall = hardwareMap.servo.get("servoBall");
        blockLock = hardwareMap.servo.get("blockLock");
        relicServo = hardwareMap.servo.get("relicServo");

        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");
        colorSensor.enableLed(true);
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

        motorRearLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        //parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = "ARrdAAv/////AAAAmW9uze+2tUpOrVymH8EdMU4uGNoIh0dtDy1ZLrUB53M5NpXJ1PMdsAe+3M3/pNqcg9nOrY6KjImV1kxpfomVVraihhTTR6s60pnBfop1LAPtHuDFWTtUVJoT68oD4/pX/jbPhWDCAsk3dDsphHIUz8K53ATDNHXLg1bsljuKjm7RDxjgA0ivV/dVLzhM9vZ0w5DBcApqrl585MOtlCQcLbIkkMcdxUYdKGDHEFK/38z+tnuDMQ6PbA7YnhOCtoxJtYhn2fNimkvExG9mNnXTASTVge0w3vHQ7miA3yq1s8U6u2rUyby/6MaPZEFlOta31e87/sp4z+rZQndy5y9hrt1EjGn0YVKZbll5Uect3WU7";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);

        relicTrackables.activate();

        //group and set up the left motors
        /*leftMotors = new Motors(motorFrontLeft, motorRearLeft);
        leftMotors.setDirection(DcMotor.Direction.REVERSE);
        leftMotors.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotors.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //group and set up the right motors
        rightMotors = new Motors(motorFrontRight, motorRearRight);
        rightMotors.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotors.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_grab.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_grab.setMode(DcMotor.RunMode.RUN_TO_POSITION);
*/
        // make sure the gyro is calibrated before continuing
        //gyro.calibrate();

    }
    //gyro tank mode(not used)
    public void GyroTank(double leftStick, double rightStick, int level, boolean precision) {
        double rotationRate = 0;
        double P = 0.0004;
        double I = 0.0007;
        double D = 0.0;
        double F = 0.0172;
        MiniPID miniPID;
        miniPID = new MiniPID(P, I, D, F);
        miniPID.setOutputLimits(1);
        double actual = 0;
        double output = 0;
        double maxRotationRate = 360;
        double stickScalingValue = maxRotationRate / 2;
        double stickDifference = leftStick - rightStick;
        double zStick = stickDifference * stickScalingValue;
        rotationRate = zStick;


        output = miniPID.getOutput(actual, rotationRate);
        actual = -(gyro.getAngularVelocity(AngleUnit.DEGREES).zRotationRate);

        leftStick = leftStick - output;
        rightStick = rightStick + output;

        driveTank(leftStick, rightStick, level, precision);

    }
    //drives straight for what ever distance needed in inches
    public void driveDistance(double distance) {
        //tune f then p then I
        double power = 0.3;
        double rotationRate = 0;
        double P = 0.002; // going to be small bring this up until osculation occurs then back that number off
        double I = 0.00001; // tie string to one side to apply resistance and make sure it corrects itself
        double D = 0.0;
        double F = 0.0;//172; // to tune set long distance and set f to a value that causes osculation then lower it until it doesn't osculate
        MiniPID miniPID;
        miniPID = new MiniPID(P, I, D, F);
        miniPID.setOutputLimits(0.5);
        double actual = 0;
        double output = 0;
        int inches = (int) distance * (int) COUNTS_PER_INCH;

        motorRearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRearRight.setTargetPosition(inches);
        motorRearLeft.setTargetPosition(inches);

        gyro.resetZAxisIntegrator();


        while (motorRearLeft.isBusy() && motorRearRight.isBusy() && lop.opModeIsActive()) {
            output = miniPID.getOutput(actual, rotationRate);
            actual = (gyro.getIntegratedZValue());

            double leftPower = power + output; //if robot goes crazy one way reverse the + & minus
            double rightPower = power - output;

            motorRearRight.setPower(leftPower);
            motorRearLeft.setPower(rightPower);
        }
        motorRearRight.setPower(0);
        motorRearLeft.setPower(0);
        //driveTurn(0, false);
        sleep(500);
    }
    //turns the robot to a certain position relative to the position it is in right now
    public void driveTurn(double angle, boolean reset) {
        rateLimiter angleRamp = new rateLimiter();
        double rate = 60;
        double P = 0.002; // going to be small bring this up until osculation occurs then back that number off
        double I = 0.00001; // tie string to one side to apply resistance and make sure it corrects itself
        double D = 0.0;
        double F = 0.0;//172; // to tune set long distance and set f to a value that causes osculation then lower it until it doesn't osculate
        MiniPID miniPID;
        miniPID = new MiniPID(P, I, D, F);
        miniPID.setOutputLimits(0.5);
        double actual = 0;
        double output = 0;
        motorRearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (reset) {
            gyro.resetZAxisIntegrator();
        }
        while (Math.abs(angle - gyro.getIntegratedZValue()) > 1 && lop.opModeIsActive()) {
            double limitedAngle = angleRamp.ratelimiter(angle, rate/100);

            output = miniPID.getOutput(actual, limitedAngle);
            actual = (gyro.getIntegratedZValue());

            double leftPower = output; //if robot goes crazy one way reverse the + & minus
            double rightPower = -output;

            motorRearRight.setPower(leftPower);
            motorRearLeft.setPower(rightPower);

            sleep(10);
        }
        motorRearLeft.setPower(0);
        motorRearRight.setPower(0);

        sleep(500);
    }

    //returns the value of the present cypher
    public String getCypher() {

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        return vuMark.toString();
    }

    //initilizes the servos so that they are in 18 at the start of the match
    public void initServos() {

    }

    //tank drive function
    public void driveTank(double leftStick, double rightStick, int level, boolean precision) {
        motorRearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // precision and level are used together to reduce the stick output
        if (precision) {
            if (Math.abs(leftStick - rightStick) > 0.3) {
                level = (int) ((double) level * 1.5);
            }
            leftStick = leftStick * level / 100;
            rightStick = rightStick * level / 100;
        }
        // write the values to the motors
        motorRearLeft.setPower(conditionStick(leftStick));
        motorRearRight.setPower(conditionStick(rightStick));
    }

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}