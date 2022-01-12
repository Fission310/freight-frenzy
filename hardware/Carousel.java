package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Carousel extends Mechanism {

    private DcMotor duckDisk;

    public Carousel(LinearOpMode opMode){
        this.opMode = opMode;
    }

    @Override
    public void init(HardwareMap hwMap) {
        duckDisk = hwMap.dcMotor.get("duckDisk");

        //TODO: reverse if necessary
        //duckDisk.setDirection(DcMotorSimple.Direction.REVERSE);
        duckDisk.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void spin() {
        duckDisk.setPower(1);
    }

    public void reverse() {
        duckDisk.setPower(-1);
    }

    public void stop(){duckDisk.setPower(0);}

}
