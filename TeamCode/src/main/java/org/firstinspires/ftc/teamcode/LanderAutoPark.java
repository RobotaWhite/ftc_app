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

    //drive forward 4 inches
    robot.encoderDrive(10, 0.3);

    //turn to the left 90 degrees
    //robot.driveTurn(-90, true);

    //drive around minerals
    //robot.driveDistance(25);

    //turn to wall
    //robot.driveTurn(45, true);

    //drive to wall

    //turn to back

    //back into the parking spot

    //dump out marker


        //robot.autoClimbTime(1000, 0.25);

        //robot.driveDistance(4);

    //robot.driveCypher(3);

    //telemetry.addData("Distance", robot.cypherDistance());
    //telemetry.addData("Name", robot.getCypher());

    telemetry.update();


    }
}