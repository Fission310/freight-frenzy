package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Carousel extends Mechanism {

    public static double DISK_POWER = 0.55;
    private DcMotor duckDisk;

    public Carousel(LinearOpMode opMode){
        this.opMode = opMode;
    }

    @Override
    public void init(HardwareMap hwMap) {
        duckDisk = hwMap.dcMotor.get("duckDisk");

        duckDisk.setDirection(DcMotorSimple.Direction.REVERSE);
        duckDisk.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void spin() {
        duckDisk.setPower(DISK_POWER);
    }

    public void reverse() {
        duckDisk.setPower(-DISK_POWER);
    }

    public void stop(){duckDisk.setPower(0);}

}
