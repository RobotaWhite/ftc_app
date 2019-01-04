package org.firstinspires.ftc.teamcode;

//------------------------------------------------------------------------------
// PushBotManual
//

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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

    //robot.resetAngle();

    waitForStart();

    robot.autoDrop();

    //robot.driveRange(9);
    //robot.driveTurn(90, true);

    //drive forward 10 inches
    robot.driveDistance(15);

    //turn to the left 90 degrees
    robot.driveTurn(90, true);

    //drive around minerals
    robot.driveDistance(30);

    //turn to wall
    robot.driveTurn(-40, true);

    //drive to wall
    robot.driveRange(9);

    //turn to back
    robot.driveTurn(90, true);

    //back into the parking spot
    robot.driveDistance(-30);

    //dump out marker*/
    robot.autoDump();

        //robot.autoClimbTime(1000, 0.25);

        //robot.driveDistance(4);

    //robot.driveCypher(3);

    //telemetry.addData("Distance", robot.cypherDistance());
    //telemetry.addData("Name", robot.getCypher());

    telemetry.addData("Done", "");
    telemetry.update();


    }
}