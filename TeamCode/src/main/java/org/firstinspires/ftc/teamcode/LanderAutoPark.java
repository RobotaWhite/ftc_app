package org.firstinspires.ftc.teamcode;

/**
 *
 * 2018 - 2019 FTC season Robota White #8749 depot autonomous
 * Do what you want with our code, if you have any questions
 * you can email our programmer at 19stwa_h@union.k12.ia.us with any questions
 *
 */

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

    char gold = 'c';

    waitForStart();

    robot.autoDrop();

    //char gold = robot.sample(2000);

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
        robot.driveDistance(12, 0.6);

        //turn to the left 90 degrees
        robot.driveTurn(90, true);

        //drive around minerals
        robot.driveDistance(25, 0.6);

        //turn to wall
        robot.driveTurn(-45, true);

        //drive to wall
        robot.driveRange(9);

        //turn to back
        robot.driveTurn(80, true);

        //back into the parking spot
        robot.driveDistance(-30, 0.6);

        //dump out marker
        robot.autoDump();

        //drive to other crater
        robot.driveDistance(40, 1.0);

    }

    public void center () {

        //flop out the front grabber
        robot.grabberDump(false, true, 0.5);

        sleep(500);

        robot.grabberDump(false, false, 0.0);

        //drive forward to knock off the block
        robot.driveDistance(30, 0.6);

        //knock block out of the way
        robot.intake(false, true);

        sleep(800);

        robot.intake(false, false);
        robot.grabberDump(true, false, 0.8);

        sleep(800);

        robot.grabberDump(false, false, 0.0);

        //turn to wall
        robot.driveTurn(45, true);

        //drive to the wall
        robot.driveDistance(14, 0.6);

        //turn around to face crater
        robot.driveTurn(80, true);

        //dump
        robot.autoDump();

        //drive to crater
        robot.driveDistance(32, 1.0);

        //put the grabber on the crater
        touch();

    }

    public void left () {

        //drive forward
        robot.driveDistance(10, 0.6);

        //turn to the block
        robot.driveTurn(45, true);

        //hit block
        robot.driveDistance(28, 0.6);

        //turn back to the depot
        robot.driveTurn(78, true);

        //back up to the depot
        robot.driveDistance(-15, 0.6);

        //dump marker
        robot.autoDump();

        //drive to crater
        robot.driveDistance(32, 0.6);

        //touch the crater
        touch();

    }

    public void right () {

        //drive forward
        robot.driveDistance(10, 0.6);

        //turn to gold
        robot.driveTurn(-45, true);

        //hit block
        robot.driveDistance(22, 0.6);

        //face crater
        robot.driveTurn(-85, true);

        //back up to depot
        robot.driveDistance(-28, 0.6);

        //turn to face the crater
        robot.driveTurn(-93, true);

        //drive a bit to dump safely
        robot.driveDistance(10, 0.6);

        //dump marker
        robot.autoDump();

        //floor it
        robot.driveDistance(32, 1.0);

        //put grabber on the crater
        touch();

    }

    public void touch() {

        //flop the front out
        /*robot.grabberDump(false, true, 0.5);

        sleep(500);

        robot.grabberDump(false, false, 0.0);*/

        //reach out to the crater
        robot.grabberWinch(true, false, 0.7);

        sleep(1000);

        robot.grabberWinch(false, false, 0.7);


    }

}