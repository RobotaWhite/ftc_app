package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class Motors {
    public DcMotor frontMotor;
    public DcMotor rearMotor;
    public void setDirection(DcMotorSimple.Direction direction){
        frontMotor.setDirection(direction);
        rearMotor.setDirection(direction);
    }

    public Motors(DcMotor front, DcMotor rear) {
        frontMotor = front;
        rearMotor = rear;
    }
    public int getCurrentPosition() {
        return (frontMotor.getCurrentPosition()+rearMotor.getCurrentPosition())/2;
    }

    public void setTargetPosition(int target) {
        frontMotor.setTargetPosition(target);
        rearMotor.setTargetPosition(target);
    }
    public void setMode(DcMotor.RunMode mode)  {
        frontMotor.setMode(mode);
        rearMotor.setMode(mode);
    }

    public void setPower(double power)  {
        frontMotor.setPower(power);
        rearMotor.setPower(power);
    }
    public boolean isBusy() {
        return frontMotor.isBusy() && rearMotor.isBusy();
    }

}