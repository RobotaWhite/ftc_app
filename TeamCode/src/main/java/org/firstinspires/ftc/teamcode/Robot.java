package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaRoverRuckus;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

public class Robot {


    public Robot(LinearOpMode linearOpMode) {
        lop = linearOpMode;
        hardwareMap = lop.hardwareMap;
    }
    public Robot(HardwareMap Hwmap) {
        hardwareMap = Hwmap;
    }

    //gyro object
    BNO055IMU gyro;

    //telemetry states
    Orientation angles;
    Acceleration gravity;

    //set these accessible from outside since we only read from them
    public HardwareMap hardwareMap;
    public ModernRoboticsI2cRangeSensor rangeSensor;
    public ColorSensor colorSensor;
    //public ModernRoboticsI2cGyro gyro;
    //public Motors leftMotors;
    //public Motors rightMotors;

    //set these inaccessible from outside to force using an interface
    private LinearOpMode lop;

    //motors
    private DcMotor motorRearRight; //right motor
    private DcMotor motorRearLeft; //left motor
    private DcMotor dumpWinch1; //winch to actuate the back dumper up and down
    private DcMotor dumpWinch2; //adds strength to the back for climbing
    private DcMotor grabberDump; //dumps the balls that are collected in the front
    private DcMotor grabber; //motor to collect balls
    private DcMotor grabberWinch; //motor to reach out with the grabber
    private DcMotor dumper;

    private Servo latch;
    private Servo pin;

    Orientation lastAngles = new Orientation();
    double globalAngle;

    toggle latcher = new toggle();
    toggle drivers = new toggle();

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final String VUFORIA_KEY = "ARCDmaP/////AAABmQ9Ht4LTf0TeqcQGvl0tJog2e2hdw/j3GpfNy02HmEXHzxWh1DhoVwMGleDp/d58zO69Wh8yPzsJ5WfBFxkNhvtmtvx5eplK+2vTJAte+tu0UEUqxYjLdolHQdcloT3B9J8+MUZZG2TRg/ylavKuBOklS6zTnDANL2Un0ruXolhnXPWnQGd2cbn7vdTGLyv+7+sJr1a0DN/OIyNCt0ocoTdHYNazl9PHmWXqVbSk6G4dRR83Se+esrQlbQJmHJs56e2fQXSt8+NulozDlc1GUGEsI8La1hTr0i1qeLs2ETruoTwOZBY7yecC14JXuCU+hKlbhheXzjeTtDg0thooF+xZbEfVGqlHGKPC1QI9KQuA";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    char Pos = 's';

    int cameraMonitorViewId;
    VuforiaLocalizer.Parameters parameters;

    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;

    //"ARrdAAv/////AAAAmW9uze+2tUpOrVymH8EdMU4uGNoIh0dtDy1ZLrUB53M5NpXJ1PMdsAe+3M3/pNqcg9nOrY6KjImV1kxpfomVVraihhTTR6s60pnBfop1LAPtHuDFWTtUVJoT68oD4/pX/jbPhWDCAsk3dDsphHIUz8K53ATDNHXLg1bsljuKjm7RDxjgA0ivV/dVLzhM9vZ0w5DBcApqrl585MOtlCQcLbIkkMcdxUYdKGDHEFK/38z+tnuDMQ6PbA7YnhOCtoxJtYhn2fNimkvExG9mNnXTASTVge0w3vHQ7miA3yq1s8U6u2rUyby/6MaPZEFlOta31e87/sp4z+rZQndy5y9hrt1EjGn0YVKZbll5Uect3WU7";

    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch;

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = FRONT;

    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;

    List<VuforiaTrackable> allTrackables = new ArrayList<>();

    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder
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
            grabber = hardwareMap.dcMotor.get("grabber");
            grabberWinch = hardwareMap.dcMotor.get("grabber_winch");
            grabberDump = hardwareMap.dcMotor.get("grabberDump");
            dumpWinch1 = hardwareMap.dcMotor.get("dump_winch");
            dumpWinch2 = hardwareMap.dcMotor.get("dump_winch2");
            dumper = hardwareMap.dcMotor.get("dumper");

            latch = hardwareMap.servo.get("latch");
            pin = hardwareMap.servo.get("pin");

            rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range");

            gyro = hardwareMap.get(BNO055IMU.class, "imu");

        //colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");
        //colorSensor.enableLed(true);
        //gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


        //gyro = (BNO055IMU) hardwareMap.gyroSensor.get("gyro");
        gyro.initialize(parameters);

        motorRearRight.setDirection(DcMotorSimple.Direction.FORWARD);

    }

    /*/**
     * Initialize the Vuforia localization engine.
     *
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         *
        try {
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

            parameters.vuforiaLicenseKey = VUFORIA_KEY;
            parameters.cameraName = hardwareMap.get(WebcamName.class, "camera");

            //  Instantiate the Vuforia engine
            vuforia = ClassFactory.getInstance().createVuforia(parameters);

            // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
        }

        catch (Exception g){

        }
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     *
    private void initTfod() {
        try {
            int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                    "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
            tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
            tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
        }
        catch (Exception t){

        }
    }*/


    public void cameraInit(){



            try {
                cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
                VuforiaLocalizer.Parameters parametersCamera = new VuforiaLocalizer.Parameters(cameraMonitorViewId);


                parametersCamera.cameraDirection = CAMERA_CHOICE;
                parametersCamera.vuforiaLicenseKey = VUFORIA_KEY;


                vuforia = ClassFactory.getInstance().createVuforia(parametersCamera);
            }

            catch (Exception v){

            }

            // Load the data sets that for the trackable objects. These particular data
            // sets are stored in the 'assets' part of our application.
            VuforiaTrackables targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
            VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
            blueRover.setName("Blue-Rover");
            VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
            redFootprint.setName("Red-Footprint");
            VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
            frontCraters.setName("Front-Craters");
            VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
            backSpace.setName("Back-Space");

            // For convenience, gather together all the trackable objects in one easily-iterable collection */
            //List<VuforiaTrackable> allTrackables = new ArrayList<>();
            allTrackables.addAll(targetsRoverRuckus);

            //---------------
            //locations
            //---------------

            //blue rover
            OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                    .translation(0, mmFTCFieldWidth, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
            blueRover.setLocation(blueRoverLocationOnField);

            //red footprint
            OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                    .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
            redFootprint.setLocation(redFootprintLocationOnField);

            //front crater
            OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                    .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90));
            frontCraters.setLocation(frontCratersLocationOnField);

            //back space
            OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                    .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
            backSpace.setLocation(backSpaceLocationOnField);

        //phone location
        final int CAMERA_FORWARD_DISPLACEMENT  = 100;   // eg: Camera is 110 mm in front of robot center
        final int CAMERA_VERTICAL_DISPLACEMENT = 200;   // eg: Camera is 200 mm above ground
        final int CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        CAMERA_CHOICE == FRONT ? 90 : -90, 0, 0));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables)
        {
            ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, CAMERA_CHOICE);
        }




        //activate the trackables
        targetsRoverRuckus.activate();

        /*this.vuforia = ClassFactory.createVuforiaLocalizer(parametersCamera);

        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);

        relicTrackables.activate();*/

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
        //actual = -(gyro.getAngularVelocity(AngleUnit.DEGREES).zRotationRate);

        leftStick = leftStick - output;
        rightStick = rightStick + output;

        driveTank(leftStick, rightStick, level, precision);

    }
    //drives straight for what ever distance needed in inches
    public void driveDistance(double distance) {
        //tune f then p then I
        double power = 0.7;
        double rotationRate = 0;
        double P = 0.0820; // going to be small bring this up until osculation occurs then back that number off
        double I = 0.00001; // tie string to one side to apply resistance and make sure it corrects itself
        double D = 0.0;
        double F = 0.0022; // to tune set long distance and set f to a value that causes osculation then lower it until it doesn't osculate
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

        //gyro.resetZAxisIntegrator();
        resetAngle();


        while (motorRearRight.isBusy() /*&& lop.opModeIsActive()*/) {
            output = miniPID.getOutput(actual, rotationRate);
            actual = (/*gyro.getIntegratedZValue()*/ getAngle());

            double leftPower;
            double rightPower;

            if (distance > 0) {
                leftPower = power + output; //if robot goes crazy one way reverse the + & minus
                rightPower = power - output;
            } else {
                leftPower = power - output;
                rightPower = power + output;
            }

            motorRearRight.setPower(leftPower);
            motorRearLeft.setPower(rightPower);
        }
        motorRearRight.setPower(0);
        motorRearLeft.setPower(0);
        //driveTurn(0, false);
        sleep(500);
    }

    //drives straight until the range sensor is so far from the wall
    public void driveRange(double distance) {
        //tune f then p then I
        double power = 0.7;
        double rotationRate = 0;
        double P = 0.0820; // going to be small bring this up until osculation occurs then back that number off
        double I = 0.00001; // tie string to one side to apply resistance and make sure it corrects itself
        double D = 0.0;
        double F = 0.0022; // to tune set long distance and set f to a value that causes osculation then lower it until it doesn't osculate
        MiniPID miniPID;
        miniPID = new MiniPID(P, I, D, F);
        miniPID.setOutputLimits(0.5);
        double actual = 0;
        double output = 0;

        motorRearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        resetAngle();

        while (rangeSensor.rawUltrasonic() > distance) {
            output = miniPID.getOutput(actual, rotationRate);
            actual = (/*gyro.getIntegratedZValue()*/ getAngle());

            double leftPower;
            double rightPower;

            if (distance > 0) {
                leftPower = power + output; //if robot goes crazy one way reverse the + & minus
                rightPower = power - output;
            } else {
                leftPower = power - output;
                rightPower = power + output;
            }

            motorRearRight.setPower(leftPower);
            motorRearLeft.setPower(rightPower);
        }
        motorRearRight.setPower(0);
        motorRearLeft.setPower(0);
        driveTurn(0, false);
        sleep(500);
    }

    public void encoderDrive(double distance, double pwr){
        double rotationRate = 0;
        double P = 0.0008; // going to be small bring this up until osculation occurs then back that number off
        double I = 0.0000;//1; // tie string to one side to apply resistance and make sure it corrects itself
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

        while (motorRearRight.isBusy()){

            output = miniPID.getOutput(actual, rotationRate);
            actual = (motorRearLeft.getCurrentPosition() - motorRearRight.getCurrentPosition());

            double leftPower = pwr; //+ output; //if robot goes crazy one way reverse the + & minus
            double rightPower = pwr; // - output;

            motorRearRight.setPower(rightPower);
            motorRearLeft.setPower(leftPower);

        }

        motorRearLeft.setPower(0);
        motorRearRight.setPower(0);

    }

    public double left(){
        double count;

        count = motorRearLeft.getCurrentPosition();

        return count;

    }

    public double right(){
        double count;

        count = motorRearRight.getCurrentPosition();

        return count;

    }

    //turns the robot to a certain position relative to the position it is in right now
    public void driveTurn(double angle, boolean reset) {
        rateLimiter angleRamp = new rateLimiter();
        double pwr = 0.1;
        double rate = 60;
        double P = 100; // going to be small bring this up until osculation occurs then back that number off
        double I = 0.0000;//1; // tie string to one side to apply resistance and make sure it corrects itself
        double D = 0.0;
        double F = 0.0;//22; // to tune set long distance and set f to a value that causes osculation then lower it until it doesn't osculate
        MiniPID miniPID;
        miniPID = new MiniPID(P, I, D, F);
        miniPID.setOutputLimits(0.2);//was 0.5
        double actual = 0;
        double output = 0;
        motorRearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (reset) {
            //gyro.resetZAxisIntegrator();
            resetAngle();
        }
        while (Math.abs(angle - getAngle()/*gyro.getIntegratedZValue()*/) > 5 /*&& lop.opModeIsActive()*/) {
            double limitedAngle = angleRamp.ratelimiter(angle, rate*5);// was divided by 100

            output = miniPID.getOutput(actual, limitedAngle);
            actual = (getAngle()/*gyro.getIntegratedZValue()*/);

            double leftPower = pwr + output; //if robot goes crazy one way reverse the + & minus
            double rightPower = -(pwr + output);

            motorRearRight.setPower(leftPower);
            motorRearLeft.setPower(rightPower);

            sleep(10);
        }
        motorRearLeft.setPower(0);
        motorRearRight.setPower(0);

        sleep(500);
    }

    //drives using the cypher heading to go to the wall
    public void driveCypher(double distance) {
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
        /*int inches = (int) distance * (int) COUNTS_PER_INCH;

        motorRearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRearRight.setTargetPosition(inches);
        motorRearLeft.setTargetPosition(inches);*/

        while (motorRearLeft.isBusy() && motorRearRight.isBusy() && lop.opModeIsActive() && distance > cypherDistance()) {
            output = miniPID.getOutput(actual, rotationRate);
            actual = (cypherDirection());

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

    public void resetAngle(){

        lastAngles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;

    }

    public double getAngle(){

        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180){
            deltaAngle += 360;
        } else if (deltaAngle > 180){
            deltaAngle -= 360;
        }

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    //returns the value of the present cypher
    public String getCypher() {

        String name = "None";

        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                name = trackable.getName();
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        return name;
        //target.toString();

        //RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        //return vuMark.toString();
    }

    public double cypherDirection(){

        double Angle = 0;

        if (targetVisible) {

            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            Angle = rotation.secondAngle;
        }

        return Angle;

    }

    public double cypherDistance(){

        double distance = 0;

        if (targetVisible) {

            // express position (translation) of robot in inches.
            VectorF translation = lastLocation.getTranslation();

            distance =  (translation.get(1));
        }
        return distance;
    }

    //initilizes the servos so that they are in 18 at the start of the match
    public void initServos() {

    }

    //drive time
    public void driveTime(int time, double pwr){

        motorRearRight.setPower(pwr);
        motorRearLeft.setPower(pwr);

        sleep(time);

        motorRearRight.setPower(0);
        motorRearLeft.setPower(0);
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

    //function to control climbing in autonomous
    public void autoClimbTime(int time, double pwr){

        dumpWinch1.setPower(-pwr);
        dumpWinch2.setPower(-pwr);

        sleep(time);

        dumpWinch1.setPower(0);
        dumpWinch2.setPower(0);
    }

    public void driverToggle(boolean input, double left1, double right1, double left2, double right2){
        if (drivers.value(input)){
            driveTank(left2, right2, 8, false);
        } else {
            driveTank(left1, right1, 8, false);
        }
    }

    // actuate the back slides up to reach up to the rover to dump the minerals
    public void dumpWinch(boolean up, boolean down){

        if (up){
            dumpWinch1.setPower(1);
            dumpWinch2.setPower(1);
        } else if (down){
            dumpWinch1.setPower(-1);
            dumpWinch2.setPower(-1);
        } else {
            dumpWinch1.setPower(0);
            dumpWinch2.setPower(0);
        }
    }

    /*public void dumpWinch(boolean up, boolean down, double pwr) {

        int pos = dumpWinch1.getCurrentPosition();
        double motorPower;

        double rotationRate = 0;
        double P = 0.0008; // going to be small bring this up until osculation occurs then back that number off
        double I = 0.0000;//1; // tie string to one side to apply resistance and make sure it corrects itself
        double D = 0.0;
        double F = 0.0;//172; // to tune set long distance and set f to a value that causes osculation then lower it until it doesn't osculate
        MiniPID miniPID;
        miniPID = new MiniPID(P, I, D, F);
        miniPID.setOutputLimits(0.5);
        double actual = 0;
        double output = 0;

        dumpWinch1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dumpWinch2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        dumpWinch1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dumpWinch2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (up) {
            pos += 20;
        } else if (down) {
            pos -= 20;
        }

        dumpWinch1.setTargetPosition(pos);
        dumpWinch2.setTargetPosition(pos);

        while (motorRearRight.isBusy()){

            output = miniPID.getOutput(actual, rotationRate);
            actual = (motorRearLeft.getCurrentPosition() - pos);

            if (dumpWinch1.getCurrentPosition() > pos) {
                motorPower = pwr - output;
            } else if (dumpWinch1.getCurrentPosition() < pos + 10) {
                motorPower = pwr + output;
            } else {
                motorPower = 1;
            }

            dumpWinch1.setPower(motorPower);
            dumpWinch2.setPower(motorPower);

        }

    }*/

    //move the grabber up and down to collect minerals or dump them out
    public void grabberDump(boolean up, boolean down, double speed){
        if (up){
            grabberDump.setPower(speed);
        } else if (down){
            grabberDump.setPower(-speed);
        } else {
            grabberDump.setPower(0);
        }
    }

    //uses bool inputs to turn the motors on and off forward and reverse to intake minerals
    public void intake(boolean in, boolean out){
        if (in){
            grabber.setPower(0.7);
        } else if (out){
            grabber.setPower(-0.7);
        } else {
            grabber.setPower(0);
        }

    }

    //actuate the front grabber assembly in and out to grab minerals without going inside the crater
    public void grabberWinch (boolean out, boolean in){
        if (out){
            grabberWinch.setPower(0.5);
        } else if (in){
            grabberWinch.setPower(-0.5);
        } else {
            grabberWinch.setPower(0);
        }
    }

    //dumps the minerals into the rover
    public void roverDump (boolean dump, boolean in){
        if (in){
            dumper.setPower(0.5);
        } else if (dump){
            dumper.setPower(-0.6);
        } else {
            dumper.setPower(0);
        }
    }

    public void roverLatch(boolean input){

        if (latcher.value(input)){
            latch.setPosition(0.5);
        } else {
            latch.setPosition(0);
        }

    }

    public void servoPin(boolean on, boolean off){

        if (on) {
            pin.setPosition(0);
        } else if (off) {
            pin.setPosition(1);
        }

        sleep(5000);

        roverLatch(false);

        //servoLock(false, true);

    }

    public void autoDump(){
        roverDump(true, false);
        grabberDump(false, true, 0.5);

        sleep(1400);

        roverDump(false, false);
        grabberDump(true, false, 0.8);

        sleep(800);
        grabberDump(false, false, 0.0);

    }

    public void autoDrop(){

        //roverLatch(true);
        pin.setPosition(1);

        sleep(1800);

        roverLatch(false);

        sleep(800);
    }

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public double range(){

        double rawRange;

        rawRange = rangeSensor.rawUltrasonic();

        return rawRange;
    }

    /*public char sample() {
        char gold = ' ';
        boolean goldFound = true;

        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            gold = 's';

        }

            //Activate Tensor Flow Object Detection.
            if (tfod != null) {
                tfod.activate();
            }

            while (goldFound) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        if (updatedRecognitions.size() == 3) {
                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getLeft();
                                } else if (silverMineral1X == -1) {
                                    silverMineral1X = (int) recognition.getLeft();
                                } else {
                                    silverMineral2X = (int) recognition.getLeft();
                                }
                            }
                            if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                                if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                    gold = 'l';
                                    goldFound = false;
                                } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                    gold = 'r';
                                    goldFound = false;
                                } else {
                                    gold = 'c';
                                    goldFound = false;
                                }
                            }
                        }
                    }
                }
            }

        if (tfod != null) {
            tfod.shutdown();
        }

        return gold;
    }//*/

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "camera");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    public char sample (int timeout) {

        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            //uh on
        }

        if (true) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }

            long startTime = System.currentTimeMillis();
            long currentTime = startTime;
            while (currentTime - startTime < timeout) {
                //telemetry.addData("time", currentTime - startTime);
                //telemetry.update();
                if (tfod != null) {
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

                    if (updatedRecognitions != null) {
                        //telemetry.addData("# Object Detected", updatedRecognitions.size());
                        //telemetry.update();
                        if (updatedRecognitions.size() == 2) {
                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;

                            // This just records values, and is unchanged

                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getTop();
                                } else if (silverMineral1X == -1) {
                                    silverMineral1X = (int) recognition.getTop();
                                } else {
                                    silverMineral2X = (int) recognition.getTop();
                                }
                            }

                            // If there is no gold (-1) and there two silvers (not -1) the gold
                            // is not visible, and must be on the right

                            if (goldMineralX == -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                                //telemetry.addData("Gold Mineral Position", "Right");
                                Pos = 'r';
                                //telemetry.addData("Position", Position);
                                //telemetry.update();
                            }

                            // If you can see one gold and one silver ...

                            else if (goldMineralX != -1 && silverMineral1X != -1) {
                                // ... if the gold is to the right of the silver, the gold is in the center ...


                                if (goldMineralX > silverMineral1X) {
                                    //telemetry.addData("Gold Mineral Position", "Center");
                                    Pos = 'c';
                                    //telemetry.addData("Position", Position);
                                    //telemetry.update();
                                }

                                // ... otherwise it is on the left

                                else {
                                    //telemetry.addData("Gold Mineral Position", "Left");
                                    Pos = 'l';
                                    //telemetry.addData("Position", Position);
                                    //telemetry.update();
                                }
                            }

                            // ... otherwise it is on the left

                        }
                    }
                }
                //telemetry.update();

                currentTime = System.currentTimeMillis();
            }//*/

            //telemetry.addData("Pos", Position);
            //telemetry.update();

            //sleep(5000);

        }
        return Pos;
    }

}