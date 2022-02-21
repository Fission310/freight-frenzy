package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class CarouselTest extends Mechanism {
    private CRServo supaspeed;

    public static double POWER = 1;

    public CarouselTest (LinearOpMode opMode) { this.opMode = opMode; }

    @Override
    public void init(HardwareMap hwMap) {
        supaspeed = hwMap.get(CRServo.class, "CRServo");
    }

    public void rotate() {
        supaspeed.setPower(POWER);
    }
    public void reverse() {
        supaspeed.setPower(-POWER);
    }
    public void stop() {
        supaspeed.setPower(0);
    }

}
