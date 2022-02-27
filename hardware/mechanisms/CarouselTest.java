package org.firstinspires.ftc.teamcode.hardware.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class CarouselTest extends Mechanism {
    private CRServo carouselRight;

    public static double POWER = 1;

    public CarouselTest (LinearOpMode opMode) { this.opMode = opMode; }

    @Override
    public void init(HardwareMap hwMap) {
        carouselRight = hwMap.get(CRServo.class, "carouselRight");
    }

    public void rotate() {
        carouselRight.setPower(POWER);
    }
    public void reverse() {
        carouselRight.setPower(-POWER);
    }
    public void stop() {
        carouselRight.setPower(0);
    }

    public void loop(Gamepad gamepad1) {
        if (gamepad1.right_bumper) {
            rotate();
        } else if (gamepad1.left_bumper) {
            reverse();
        } else {
            stop();
        }
    }

}
