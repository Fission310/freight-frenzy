package org.firstinspires.ftc.teamcode.hardware;

import org.firstinspires.ftc.teamcode.hardware.Mechanism;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Flywheel extends Mechanism{

    private DcMotor motor;

    public Flywheel(LinearOpMode opMode){
        this.opMode = opMode;
    }

    public void init(HardwareMap hwMap){
        motor = hwMap.dcMotor.get("flywheel");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void forward(){
        motor.setPower(1);
    }
    public void reverse(){motor.setPower(-1);}
    public void stop(){
        motor.setPower(0);
    }


}
