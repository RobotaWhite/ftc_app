package org.firstinspires.ftc.teamcode;

//------------------------------------------------------------------------------
// PushBotManual
//

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
//import com.qualcomm.robotcore.util.Range;


@TeleOp (name="Pushbot: Teleop", group="Pushbot")

public class PushBotTeleop extends OpMode {
    Robot robot;
    rateLimiter lStick = new rateLimiter();
    rateLimiter rStick = new rateLimiter();

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        robot.init();
        robot.initServos();
    }


    @Override
    public void loop() {
        if (!gamepad2.left_bumper){
            // float the values
            float leftY = gamepad1.left_stick_y;
            float rightY = gamepad1.right_stick_y;

            // set the powers
            robot.leftMotors.setPower(leftY);
            robot.rightMotors.setPower(rightY);
        } else if (gamepad2.left_bumper) {
            // float the values
            float slowLY = gamepad1.right_stick_y/3;
            float slowRY = gamepad1.left_stick_y/3;

            // set the powers
            robot.rightMotors.setHalfSpeed(slowLY);
            robot.leftMotors.setHalfSpeed(slowRY);

        }
        if (gamepad2.dpad_left){
            robot.motorFrontRight.setPower(0.2);
            robot.motorFrontLeft.setPower(0.2);
        } else if (gamepad2.dpad_right){
            robot.motorFrontRight.setPower(1);
            robot.motorFrontLeft.setPower(1);
        }else {
            robot.motorFrontRight.setPower(0);
            robot.motorFrontLeft.setPower(0);
        }

        if (gamepad2.dpad_up) { //servo actuation to up
            robot.servoColorCommand(0);
        }

        if (gamepad2.dpad_down) { //servo actuation to middle
            robot.servoColorCommand(0.5);
        }

        robot.lift(gamepad1.right_bumper, gamepad1.left_bumper, 1, 0.5);

        if (gamepad1.b || gamepad2.a){
            robot.relicServoCommand(1);
        } else if (gamepad1.x || gamepad2.x) {
            robot.relicServoCommand(0);
        } else if (gamepad2.right_bumper){ //loosen
            robot.relicServoCommand(0.7);
        }

        robot.glyphGrabber(gamepad1.a, gamepad1.y, 0.5, 0.8);

        robot.relicArm(gamepad2.y, gamepad2.b, 0.9);
    }
}