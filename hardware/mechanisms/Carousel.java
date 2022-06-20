package org.firstinspires.ftc.teamcode.hardware.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.stuyfission.fissionlib.util.Mechanism;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Carousel extends Mechanism {
    private DcMotorEx carousel;

    public static double POWER = 0.27;
    public static double AUTO_POWER = 0.2;

    public Carousel(LinearOpMode opMode) { this.opMode = opMode; }

    @Override
    public void init(HardwareMap hwMap) {
        carousel = hwMap.get(DcMotorEx.class, "carousel");
    }

    public void rotate() {
        carousel.setPower(POWER);
    }
    public void rotateAUTO() {
        carousel.setPower(AUTO_POWER);
    }
    public void reverse() {
        carousel.setPower(-POWER);
    }
    public void reverseAUTO() {
        carousel.setPower(-AUTO_POWER);
    }
    public void stop() {
        carousel.setPower(0);
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
