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
    robot = new Robot(this);
    robot.init();

    //char gold = 'r';

    waitForStart();

    robot.autoDrop();

    char gold = robot.sample(2000);

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

    telemetry.addData("Pos", gold);

    telemetry.addData("Done", "");
    telemetry.update();


    }

    public void skip () {

        //drive forward 15 inches
        robot.driveDistance(12);

        //turn to the left 90 degrees
        robot.driveTurn(90, true);

        //drive around minerals
        robot.driveDistance(25);

        //turn to wall
        robot.driveTurn(-45, true);

        //drive to wall
        robot.driveRange(9);

        //turn to back
        robot.driveTurn(80, true);

        //back into the parking spot
        robot.driveDistance(-30);

        //dump out marker
        robot.autoDump();

    }

    public void center () {

        //drive forward to knock off the block
        robot.driveDistance(43);

        //turn 180
        robot.driveTurn(225, true);

        //dump
        robot.autoDump();

        //drive to crater
        robot.driveDistance(55);

    }

    public void left () {

        //drive forward
        robot.driveDistance(10);

        //turn to the block
        robot.driveTurn(45, true);

        //hit block
        robot.driveDistance(17);

        //turn back to the depot
        robot.driveTurn(75, true);

        //back up to the depot
        robot.driveDistance(-27);

        //turn to face crater
        robot.driveTurn(90, true);

        //dump marker
        robot.autoDump();

        //drive to crater
        robot.driveDistance(50);

    }

    public void right () {

        //drive forward
        robot.driveDistance(10);

        //turn to gold
        robot.driveTurn(-45, true);

        //hit block
        robot.driveDistance(22);

        //face crater
        robot.driveTurn(-85, true);

        //back up to depot
        robot.driveDistance(-10);

        //dump marker
        robot.autoDump();

        //floor it
        robot.driveDistance(40);

    }

}