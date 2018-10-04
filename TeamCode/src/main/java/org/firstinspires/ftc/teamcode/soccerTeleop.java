package org.firstinspires.ftc.teamcode;

//------------------------------------------------------------------------------
// Teleop program for simple soccer robot for outreach
//

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@TeleOp (name="soccerTeleop", group="8749")

public class soccerTeleop extends OpMode {

    public DcMotor right_drive;
    public DcMotor left_drive;

    public Servo ballHitter;
    public Servo ballHolder;

    public void init(){
       right_drive = hardwareMap.dcMotor.get("right_drive");
       left_drive = hardwareMap.dcMotor.get("left_drive");
       ballHitter = hardwareMap.servo.get("ballHitter");
        ballHolder = hardwareMap.servo.get("ballHolder");
    }

@Override
public void loop() {

    float rightStick = gamepad1.right_stick_y;
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
    }
    }
}