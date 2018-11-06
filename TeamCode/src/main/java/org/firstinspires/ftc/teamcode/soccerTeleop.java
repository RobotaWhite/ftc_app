package org.firstinspires.ftc.teamcode;

//------------------------------------------------------------------------------
// Teleop program for simple soccer robot for outreach
//

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp (name="soccerTeleop", group="8749")

public class soccerTeleop extends OpMode {

    //public DcMotor right_drive;
    //public DcMotor left_drive;

    //public Servo ballHitter;
    public Servo test;
    public ModernRoboticsI2cRangeSensor dist;

    public void init(){
       //right_drive = hardwareMap.dcMotor.get("right_drive");
       //left_drive = hardwareMap.dcMotor.get("left_drive");
       //ballHitter = hardwareMap.servo.get("ballHitter");
        test = hardwareMap.servo.get("test");
        dist = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "dist");
    }

@Override
public void loop() {

        if (dist.rawUltrasonic() < 10)
        {
            test.setPosition(1);
        } else {
            test.setPosition(0);
        }

    telemetry.addData("raw ultrasonic", dist.rawUltrasonic());
    telemetry.addData("raw optical", dist.rawOptical());
    telemetry.addData("cm optical", "%.2f cm", dist.cmOptical());
    telemetry.addData("cm", "%.2f cm", dist.getDistance(DistanceUnit.CM));
    telemetry.update();

    /*float rightStick = gamepad1.right_stick_y;
    float leftStick = gamepad1.left_stick_y;

    right_drive.setPower(rightStick);
    left_drive.setPower(leftStick);

    if (gamepad1.right_bumper){
        ballHitter.setPosition(0.5);
    } else {
        ballHitter.setPosition(0.25);
    }
    if (gamepad1.left_bumper){
        ballHolder.setPosition(0);
    } else if (gamepad1.left_trigger > 0.1){
        ballHolder.setPosition(0.4);
    }*/



    }
}