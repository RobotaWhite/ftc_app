package org.firstinspires.ftc.teamcode;

//------------------------------------------------------------------------------
// PushBotManual
//

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="LanderAuto", group="Auto")

public class LanderAutoPark extends LinearOpMode {
Robot robot;
boolean toggle = false;

@Override
public void runOpMode() {

    //set the hardware map
    robot = new Robot(hardwareMap);
    robot.init();
    //robot.cameraInit();

    waitForStart();

    //drive forward 4 inches
    robot.driveDistance(5);

    //turn to the left 45 degrees
    robot.driveTurn(-45, true);

    //drive to wall
    robot.driveDistance(5);

    //turn to crater
    robot.driveTurn(90, true);

    //drive to crater
    robot.driveDistance(5);

    //dump out marker
    robot.intake(false, true);
    sleep(500);
    robot.intake(false, false);

        //robot.autoClimbTime(1000, 0.25);

        //robot.driveDistance(4);

    //robot.driveCypher(3);

    //telemetry.addData("Distance", robot.cypherDistance());
    //telemetry.addData("Name", robot.getCypher());
    //telemetry.update();


    }
}