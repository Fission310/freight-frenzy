package org.firstinspires.ftc.teamcode.hardware.mechanisms;

import com.stuyfission.fissionlib.util.Mechanism;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Carousel extends Mechanism {
    private CRServo carouselRight;
    private CRServo carouselLeft;

    public static double POWER = 1;

    public Carousel(LinearOpMode opMode) { this.opMode = opMode; }

    @Override
    public void init(HardwareMap hwMap) {
        carouselRight = hwMap.get(CRServo.class, "carouselRight");
        carouselLeft = hwMap.get(CRServo.class, "carouselLeft");
    }

    public void rotate() {
        carouselRight.setPower(POWER);
        carouselLeft.setPower(POWER);
    }
    public void reverse() {
        carouselRight.setPower(-POWER);
        carouselLeft.setPower(-POWER);
    }
    public void stop() {
        carouselRight.setPower(0);
        carouselLeft.setPower(0);
    }

    @Override
    public void loop(Gamepad gamepad) {
        if (gamepad.right_bumper) {
            rotate();
        } else if (gamepad.left_bumper) {
            reverse();
        } else {
            stop();
        }
    }

}
