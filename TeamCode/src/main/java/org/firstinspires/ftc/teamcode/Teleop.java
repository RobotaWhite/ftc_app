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

        telemetry.addData("Clear", robot.colorSensor.alpha());
        telemetry.addData("Red  ", robot.colorSensor.red());
        telemetry.addData("Green", robot.colorSensor.green());
        telemetry.addData("Blue ", robot.colorSensor.blue());
        telemetry.update();

        if (gamepad2.right_stick_button) {
            robot.driveDistance(48);
        }
    }
}