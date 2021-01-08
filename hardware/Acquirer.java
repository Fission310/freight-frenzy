package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


public class Acquirer extends Mechanism {

    private DcMotor intake;

    public Acquirer(LinearOpMode opMode){this.opMode = opMode;}

    public void init(HardwareMap hwMap){
        intake = hwMap.dcMotor.get("intake");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);;
    }

    public void forward(){
        intake.setPower(1);
    }

    public void back(){
        intake.setPower(-1);
    }

    public void stop(){
        intake.setPower(0);
    }
}
