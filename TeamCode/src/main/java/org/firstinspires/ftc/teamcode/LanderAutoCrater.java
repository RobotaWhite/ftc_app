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
    robot = new Robot(this);
    robot.init();
    //robot.cameraInit();

    waitForStart();

        /**old code
         robot.servoPin(false, true);

        sleep(1000);

        robot.driveTime(3000, -0.5);*/

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

        //drive forward 15 inches to park
        robot.driveDistance(35);

    }

    public void center () {

        //drive forward to knock off the block
        robot.driveDistance(35);

    }

    public void left () {

        //drive forward
        robot.driveDistance(15);

        //turn to the block
        robot.driveTurn(45, true);

        //hit block
        robot.driveDistance(25);

    }

    public void right () {

        //drive forward
        robot.driveDistance(15);

        //turn to gold
        robot.driveTurn(-45, true);

        //hit block
        robot.driveDistance(25);

    }
}
