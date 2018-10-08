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

        // driver 1 tank drive
        robot.driveTank(gamepad1.left_stick_y, gamepad1.right_stick_y, 8, false);

        //driver 2 tank drive
        robot.driveTank(-gamepad2.left_stick_y, -gamepad2.right_stick_y, 8, false);



        /*telemetry.addData("cypher", robot.getCypher());

        telemetry.addData("Clear", robot.colorSensor.alpha());
        telemetry.addData("Red  ", robot.colorSensor.red());
        telemetry.addData("Green", robot.colorSensor.green());
        telemetry.addData("Blue ", robot.colorSensor.blue());
        telemetry.update();*/

    }
}