package org.firstinspires.ftc.teamcode.hardware.mechanisms;

import com.stuyfission.fissionlib.motion.MotionProfiledDcMotor;
import com.stuyfission.fissionlib.util.Mechanism;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.math3.geometry.euclidean.twod.Line;

@Config
public class Carousel extends Mechanism {
    private MotionProfiledDcMotor carousel;

    private static final double WHEEL_RADIUS = 1.415;
    private static final double GEAR_RATIO = 1.0;
    private static final double TICKS_PER_REV = 537.6;

    public static double MAX_VEL = 10;
    public static double MAX_ACCEL = 10;

    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0;

    public static double ROTATE_POS = 10;

    public Carousel(LinearOpMode opMode) { this.opMode = opMode; }

    @Override
    public void init(HardwareMap hwMap) {
        carousel = new MotionProfiledDcMotor(hwMap, "carousel");
        carousel.setWheelConstants(WHEEL_RADIUS, GEAR_RATIO, TICKS_PER_REV);
        carousel.setMotionConstraints(MAX_VEL, MAX_ACCEL);
        carousel.setPIDCoefficients(kP, kI, kD, kF);
        carousel.setTargetPosition(0);
    }

    public void rotate() {
        carousel.setTargetPosition(ROTATE_POS);
    }

    public void stop() {
        carousel.setPower(0);
    }

    @Override
    public void loop(Gamepad gamepad) {
        if (gamepad.right_bumper) {
            rotate();
        } else {
            stop();
        }
    }

}
