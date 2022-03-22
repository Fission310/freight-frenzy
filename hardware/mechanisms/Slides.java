package org.firstinspires.ftc.teamcode.hardware.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.Servo;
import com.stuyfission.fissionlib.util.Mechanism;
import com.stuyfission.fissionlib.motion.MotionProfiledDcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@Config
public class Slides extends Mechanism {

    MotionProfiledDcMotor spool;

    public static double EXTEND_POS = 7;
    public static double MAX_VEL = 30;
    public static double MAX_ACCEL = 30;
    public static double RETRACTION_MULTIPLIER = 0.5;
    private static final double WHEEL_RADIUS = 1.37795;
    private static final double GEAR_RATIO = 1.0;
    private static final double TICKS_PER_REV = 537.6;

    public static double kP = 1.6;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0;

    Servo cup;
    Servo armLeft;
    Servo armRight;

    public static double CUP_MAX = 0.7;
    public static double CUP_REST = 0.08;
    public static double CUP_MIN = 0;

    public static double ARM_MAX = 1;
    public static double ARM_REST = 0.21;
    public static double ARM_MIN = 0;

    public Slides(LinearOpMode opMode) { this.opMode = opMode; }

    @Override
    public void init(HardwareMap hwMap) {
        // init motor
        spool = new MotionProfiledDcMotor(hwMap, "slidesMotor");
        spool.setWheelConstants(WHEEL_RADIUS, GEAR_RATIO, TICKS_PER_REV);
        spool.setMotionConstraints(MAX_VEL, MAX_ACCEL);
        spool.setPIDCoefficients(kP, kI, kD, kF);
        spool.setRetractionMultiplier(RETRACTION_MULTIPLIER);
        spool.setDirection(DcMotorSimple.Direction.REVERSE);
        spool.setTargetPosition(0);

        cup = hwMap.get(Servo.class, "freightServo");
        cup.setDirection(Servo.Direction.REVERSE);
        armLeft = hwMap.get(Servo.class, "leftSlide");
        armLeft.setDirection(Servo.Direction.FORWARD);
        armRight = hwMap.get(Servo.class, "rightSlide");
        armRight.setDirection(Servo.Direction.REVERSE);
    }
    @Override
    public void loop(Gamepad gamepad) {
        if (gamepad.b) {
            spool.setTargetPosition(EXTEND_POS);
        } else if (gamepad.a) {
            spool.setTargetPosition(0);
        }
        spool.update();

        if (gamepad.dpad_up) {
            armLeft.setPosition(ARM_MAX);
            armRight.setPosition(ARM_MAX);
        } else if (gamepad.dpad_down) {
            armLeft.setPosition(ARM_MIN);
            armRight.setPosition(ARM_MIN);
        }

        if (gamepad.x) {
            armLeft.setPosition(ARM_REST);
            armRight.setPosition(ARM_REST);
            cup.setPosition(CUP_REST);
        }

        if (gamepad.dpad_right) {
            cup.setPosition(CUP_MAX);
        } else if (gamepad.dpad_left) {
            cup.setPosition(CUP_MIN);
        }
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("position", spool.getPosition());
        telemetry.addData("velocity", spool.getVelocity());
        telemetry.addData("servo pos", cup.getPosition());
    }
}
