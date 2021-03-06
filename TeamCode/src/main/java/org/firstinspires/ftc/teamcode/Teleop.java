package org.firstinspires.ftc.teamcode;

/**
 *
 * 2018 - 2019 FTC season Robota White #8749 teleop
 * Do what you want with our code, if you have any questions
 * you can email our programmer at 19stwa_h@union.k12.ia.us with any questions
 *
 */

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name="Teleop", group="8749")

public class Teleop extends OpMode {
Robot robot;
boolean toggle = false;

    public void init(){
        robot = new Robot(hardwareMap);
        robot.init();
    }

@Override
public void loop() {

//DRIVER ONE
        //tank drive
        robot.driverToggle(gamepad1.b, gamepad1.left_stick_y, gamepad1.right_stick_y, -gamepad2.right_stick_y, -gamepad2.left_stick_y);

        //intake toggle
        robot.intake(gamepad1.x, gamepad1.a);

        //grabber assembly
        robot.grabberWinch(gamepad1.right_bumper, gamepad1.left_bumper, 0.7);

        //grabber dump
        robot.grabberDump(gamepad1.dpad_up, gamepad1.dpad_down, 1.00);


//DRIVER TWO <o/
        //control back slides
        robot.dumpWinch(gamepad2.right_bumper, gamepad2.left_bumper, 1, gamepad2.right_stick_button);

        //control latch
        robot.roverLatch(gamepad2.x);

        //dump into the rover
        robot.roverDump(gamepad2.a, gamepad2.b, 1.0, -1.0);

        //zero the motors
        robot.zero(gamepad2.left_stick_button);

        //latch the servo
        robot.servoPin(gamepad2.dpad_up, gamepad2.dpad_down);


        telemetry.addData("Left", robot.left());
        telemetry.addData("Right", robot.right());
        telemetry.addData("Winch ", robot.winchTele());
        telemetry.addData("Winch 1: ", robot.winch1Pos());
        telemetry.addData("Winch 2", robot.winch2Pos());
        //telemetry.addData("Sample", robot.sample());
        telemetry.addData("Range", robot.range());
        //telemetry.addData("X", robot.gyro.getPosition().x);
        //telemetry.addData("Y", robot.gyro.getPosition().y);
        //telemetry.addData("Z", robot.gyro.getPosition().z);
        telemetry.update();

    }
}