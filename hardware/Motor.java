package org.firstinspires.ftc.teamcode.hardware;

import org.firstinspires.ftc.teamcode.hardware.Mechanism;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Motor extends Mechanism{

    private DcMotor motor;

    public Motor(LinearOpMode opMode){
        this.opMode = opMode;
    }

    public void init(HardwareMap hwMap){
        motor = hwMap.dcMotor.get("motor");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void rotate(){
        motor.setPower(1);
    }

    public void backwards(){
        motor.setPower(-1);
    }


}
