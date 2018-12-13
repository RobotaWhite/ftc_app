package org.firstinspires.ftc.teamcode;

//------------------------------------------------------------------------------
// PushBotManual
//

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaException;

@TeleOp (name="Teleop", group="8749")

public class Teleop extends OpMode {
Robot robot;
boolean toggle = false;

    public void init(){
        robot = new Robot(hardwareMap);
        robot.init();
        //robot.cameraInit();
    }

@Override
public void loop() throws VuforiaException {

//DRIVER ONE
        //tank drive
        robot.driverToggle(gamepad1.b, -gamepad1.right_stick_y, -gamepad1.left_stick_y, gamepad2.left_stick_y, gamepad2.right_stick_y);

        //intake toggle
        robot.intake(gamepad1.a, gamepad1.x);

        //grabber assembly
        robot.grabberWinch(gamepad1.right_bumper, gamepad1.left_bumper);

        //grabber dump
        robot.grabberDump(gamepad1.dpad_up, gamepad1.dpad_down, 1.00);

//DRIVER TWO <o/
        //control back slides
        robot.dumpWinch(gamepad2.right_bumper, gamepad2.left_bumper);

        //control latch
        robot.roverLatch(gamepad2.x);

        //dump into the rover
        robot.roverDump(gamepad2.a, gamepad2.b);

        if(gamepad1.y){
            //robot.driveCypher(3);

            robot.servoPin(0.5);

        }

        telemetry.addData("Left", robot.left());
        telemetry.addData("Right", robot.right());
        telemetry.addData("Range", robot.range());

        telemetry.update();

        //telemetry displays
        /*telemetry.addData("Axis", robot.getAngle());
        telemetry.addData("Trackable", robot.getCypher());
        telemetry.addData("Cypher Axis", robot.cypherDirection());
        telemetry.addData("Distance", robot.cypherDistance());

        telemetry.update();*/


        /*telemetry.addData("cypher", robot.getCypher());

        telemetry.addData("Clear", robot.colorSensor.alpha());
        telemetry.addData("Red  ", robot.colorSensor.red());
        telemetry.addData("Green", robot.colorSensor.green());
        telemetry.addData("Blue ", robot.colorSensor.blue());
        telemetry.update();*/

    }
}