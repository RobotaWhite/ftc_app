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

    //char gold = 'r';
    char gold = robot.sample(3000);

    waitForStart();

    robot.autoDrop();

    switch (gold) {

        case 'l':
            telemetry.addData("LEFT", "");
            telemetry.update();

            left();

            break;

        case 'c':
            telemetry.addData("CENTER", "");
            telemetry.update();

            center();

            break;

        case 'r':
            telemetry.addData("RIGHT", "");
            telemetry.update();

            right();

            break;

        case 's':
            telemetry.addData("SKIP", "");
            telemetry.update();

            skip();

            break;

        default:
            telemetry.addData("Nothing", "");
            telemetry.update();

            skip();

            break;
    }

    telemetry.addData("Done", "");
    telemetry.update();


    }

    public void skip () {

        //drive forward 15 inches
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

        //dump out marker
        robot.autoDump();

    }

    public void center () {

        //drive forward to knock off the block
        robot.driveDistance(63);

        //turn 180
        robot.driveTurn(225, true);

        //dump
        robot.autoDump();

        //drive to crater
        robot.driveDistance(63);

    }

    public void left () {

        //drive forward
        robot.driveDistance(15);

        //turn to the block
        robot.driveTurn(45, true);

        //hit block
        robot.driveDistance(30);

        //turn back to the depot
        robot.driveTurn(90, true);

        //back up to the depot
        robot.driveDistance(-35);

        //turn to face crater
        robot.driveTurn(90, true);

        //dump marker
        robot.autoDump();

        //drive to crater
        robot.driveDistance(65);

    }

    public void right () {

        //drive forward
        robot.driveDistance(15);

        //turn to gold
        robot.driveTurn(-45, true);

        //hit block
        robot.driveDistance(30);

    }

}