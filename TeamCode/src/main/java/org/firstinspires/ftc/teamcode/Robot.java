package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
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
    //public ModernRoboticsI2cRangeSensor rangeSensor
    public ColorSensor colorSensor;
    //public ModernRoboticsI2cGyro gyro;
    //public Motors leftMotors;
    //public Motors rightMotors;

    //set these inaccessible from outside to force using an interface
    private LinearOpMode lop;

    //motors
    private DcMotor motorRearRight; //right motor
    private DcMotor motorRearLeft; //left motor
    private DcMotor dumpWinch; //winch to actuate the back dumper up and down
    private DcMotor grabberDump; //dumps the balls that are collected in the front
    private DcMotor grabber; //motor to collect balls
    private DcMotor grabberWinch; //motor to reach out with the grabber
    private DcMotor dumper;

    Orientation lastAngles = new Orientation();
    double globalAngle;

    //servos
    //private Servo dumper; //dumps the balls into the rover

    toggle mineralIntake = new toggle();
    toggle drivers = new toggle();

    VuforiaLocalizer vuforia;

    int cameraMonitorViewId;
    VuforiaLocalizer.Parameters parameters;

    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;

    private static final String VUFORIA_KEY = "ARCDmaP/////AAABmQ9Ht4LTf0TeqcQGvl0tJog2e2hdw/j3GpfNy02HmEXHzxWh1DhoVwMGleDp/d58zO69Wh8yPzsJ5WfBFxkNhvtmtvx5eplK+2vTJAte+tu0UEUqxYjLdolHQdcloT3B9J8+MUZZG2TRg/ylavKuBOklS6zTnDANL2Un0ruXolhnXPWnQGd2cbn7vdTGLyv+7+sJr1a0DN/OIyNCt0ocoTdHYNazl9PHmWXqVbSk6G4dRR83Se+esrQlbQJmHJs56e2fQXSt8+NulozDlc1GUGEsI8La1hTr0i1qeLs2ETruoTwOZBY7yecC14JXuCU+hKlbhheXzjeTtDg0thooF+xZbEfVGqlHGKPC1QI9KQuA";
            //"ARrdAAv/////AAAAmW9uze+2tUpOrVymH8EdMU4uGNoIh0dtDy1ZLrUB53M5NpXJ1PMdsAe+3M3/pNqcg9nOrY6KjImV1kxpfomVVraihhTTR6s60pnBfop1LAPtHuDFWTtUVJoT68oD4/pX/jbPhWDCAsk3dDsphHIUz8K53ATDNHXLg1bsljuKjm7RDxjgA0ivV/dVLzhM9vZ0w5DBcApqrl585MOtlCQcLbIkkMcdxUYdKGDHEFK/38z+tnuDMQ6PbA7YnhOCtoxJtYhn2fNimkvExG9mNnXTASTVge0w3vHQ7miA3yq1s8U6u2rUyby/6MaPZEFlOta31e87/sp4z+rZQndy5y9hrt1EjGn0YVKZbll5Uect3WU7";

    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch;

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = FRONT;

    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;

    List<VuforiaTrackable> allTrackables = new ArrayList<>();

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
            grabber = hardwareMap.dcMotor.get("grabber");
            grabberWinch = hardwareMap.dcMotor.get("grabber_winch");
            grabberDump = hardwareMap.dcMotor.get("grabberDump");
            dumpWinch = hardwareMap.dcMotor.get("dump_winch");
            dumper = hardwareMap.dcMotor.get("dumper");


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

        gyro = hardwareMap.get(BNO055IMU.class, "imu");
        //gyro = (BNO055IMU) hardwareMap.gyroSensor.get("gyro");
        gyro.initialize(parameters);

        motorRearLeft.setDirection(DcMotorSimple.Direction.REVERSE);

    }

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

        //gyro.resetZAxisIntegrator();
        resetAngle();


        while (motorRearLeft.isBusy() && motorRearRight.isBusy() && lop.opModeIsActive()) {
            output = miniPID.getOutput(actual, rotationRate);
            actual = (/*gyro.getIntegratedZValue()*/ getAngle());

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
            //gyro.resetZAxisIntegrator();
            resetAngle();
        }
        while (Math.abs(angle - getAngle()/*gyro.getIntegratedZValue()*/) > 1 && lop.opModeIsActive()) {
            double limitedAngle = angleRamp.ratelimiter(angle, rate/100);

            output = miniPID.getOutput(actual, limitedAngle);
            actual = (getAngle()/*gyro.getIntegratedZValue()*/);

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

        dumpWinch.setPower(-pwr);

        sleep(time);

        dumpWinch.setPower(0);
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
            dumpWinch.setPower(0.5);
        } else if (down){
            dumpWinch.setPower(-0.5);
        } else {
            dumpWinch.setPower(0);
        }
    }

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

    //uses our toggle class to toggle the intake motor on and off with a single button
    public void grabber(boolean in, boolean out){
        if (in){
            grabber.setPower(0.5);
        } else if (out){
            grabber.setPower(-0.5);
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

    //dumps the minerals into the rover, and it also resets to the collect position automatically
    public void roverDump (boolean dump, boolean in){
        if (dump){
            dumper.setPower(0.5);
        } else if (in){
            dumper.setPower(-0.5);
        } else {
            dumper.setPower(0);
        }
    }

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}