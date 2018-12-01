package org.firstinspires.ftc.teamcode;

//------------------------------------------------------------------------------
// PushBotManual
//

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Crater", group="Auto")

public class LanderAutoCrater extends LinearOpMode {
Robot robot;
boolean toggle = false;

@Override
public void runOpMode() {

    //set the hardware map
    robot = new Robot(hardwareMap);
    robot.init();
    //robot.cameraInit();

    waitForStart();


        //robot.autoClimbTime(1000, 0.25);

        robot.driveTime(3000, 0.5);


        //robot.driveDistance(4);

    //robot.driveCypher(3);

    //telemetry.addData("Distance", robot.cypherDistance());
    //telemetry.addData("Name", robot.getCypher());
    //telemetry.update();


    }
}