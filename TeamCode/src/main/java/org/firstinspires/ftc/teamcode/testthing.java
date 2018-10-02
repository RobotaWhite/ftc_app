package org.firstinspires.ftc.teamcode;

//------------------------------------------------------------------------------
// PushBotManual
//

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name="Teleop", group="8749")

public class testthing extends OpMode {
Robot robot;
boolean toggle = false;

    public void init(){
        robot = new Robot(hardwareMap);
        robot.init();
    }

@Override
public void loop() {


        /*if (!(gamepad1.dpad_left || gamepad1.dpad_right)) {
            robot.driveTank(gamepad1.left_stick_y, gamepad1.right_stick_y, 8, gamepad1.a);
            robot.TranslateReset();
        } else {
            robot.Translate(gamepad1.dpad_left, gamepad1.dpad_right, 0.8, 22.5);
        }*/

        //tank drive
        robot.driveTank(gamepad1.left_stick_y, gamepad1.right_stick_y, 8, gamepad1.a);

        if (gamepad1.start){
            robot.driveDistance(12);
        }

        telemetry.addData("cypher", robot.getCypher());

        robot.wheelIntake(gamepad2.a);

        robot.dump(gamepad1.a, gamepad1.y, 0.5, 0.3);

        telemetry.addData("Clear", robot.colorSensor.alpha());
        telemetry.addData("Red  ", robot.colorSensor.red());
        telemetry.addData("Green", robot.colorSensor.green());
        telemetry.addData("Blue ", robot.colorSensor.blue());
        telemetry.update();

        if (gamepad2.right_stick_button) {
            robot.driveDistance(48);
        }
        if (gamepad2.left_bumper) {
            robot.driveTurn(90, false);
        } else if (gamepad2.right_bumper) {
            robot.driveTurn(-90, false);
        }

        if (gamepad1.dpad_down || gamepad2.dpad_down) {
            robot.servoColorCommand(true, false);
        } else if (gamepad1.dpad_up || gamepad2.dpad_up) {
            robot.servoColorCommand(false, true);
        }
        robot.relicArm(gamepad2.y, gamepad2.b, 0.5);

        robot.relicServoCommand(gamepad1.b, gamepad1.x);

        robot.lift(gamepad1.right_bumper, gamepad1.left_bumper, 0.7, 0.5);
    }
}