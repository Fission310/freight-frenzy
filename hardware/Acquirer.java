package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Acquirer extends Mechanism{

    private DcMotor pasta;

    public Acquirer(LinearOpMode opMode){
        this.opMode = opMode;
    }

    @Override
    public void init(HardwareMap hwMap) {
        pasta = hwMap.dcMotor.get("pasta");

        //TODO reverse if necessary
        pasta.setDirection(DcMotorSimple.Direction.REVERSE);
        pasta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void acquire(){
        pasta.setPower(1);
    }

    public void reverse(){
        pasta.setPower(-1);
    }

    public void stop() {pasta.setPower(0);}
}
