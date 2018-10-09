package org.firstinspires.ftc.teamcode;

//------------------------------------------------------------------------------
// PushBotManual
//

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
        robot.driveTank(gamepad1.left_stick_y, gamepad1.right_stick_y, 8, false);

        //intake toggle
        robot.grabber(gamepad1.a);

        //grabber assembly
        robot.grabberWinch(gamepad1.right_trigger, gamepad1.left_trigger);

        //grabber dump
        robot.grabberDump(gamepad1.dpad_up, gamepad2.dpad_down, 0.25);

//DRIVER TWO <o/
        //reverse tank drive
        robot.driveTank(-gamepad2.left_stick_y, -gamepad2.right_stick_y, 8, false);

        //control back slides
        robot.dumpWinch(gamepad2.right_trigger, gamepad2.left_trigger);

        //dump into the rover




        /*telemetry.addData("cypher", robot.getCypher());

        telemetry.addData("Clear", robot.colorSensor.alpha());
        telemetry.addData("Red  ", robot.colorSensor.red());
        telemetry.addData("Green", robot.colorSensor.green());
        telemetry.addData("Blue ", robot.colorSensor.blue());
        telemetry.update();*/

    }
}