package org.firstinspires.ftc.teamcode;

//------------------------------------------------------------------------------
// PushBotManual
//

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="LanderAuto", group="Auto")

public class LanderAuto extends LinearOpMode {
Robot robot;
boolean toggle = false;

@Override
public void runOpMode() {

    //set the hardware map
    robot = new Robot(hardwareMap);
    robot.init();

    waitForStart();

    //while (opModeIsActive()){

        //robot.autoClimbTime(1000, 0.25);

        robot.driveTime(3000, 0.5);

        //}
    }
}