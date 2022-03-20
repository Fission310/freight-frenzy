package org.firstinspires.ftc.teamcode.hardware.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.stuyfission.fissionlib.util.Mechanism;
import com.stuyfission.fissionlib.motion.MotionProfiledDcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@Config
public class Slides extends Mechanism {

    MotionProfiledDcMotor spool;

    public static double EXTEND_POS = 8;
    public static double MAX_VEL = 50;
    public static double MAX_ACCEL = 50;
    public static double RETRACTION_MULTIPLIER = 0.5;
    private static final double WHEEL_RADIUS = 1.37795;
    private static final double GEAR_RATIO = 1.0;
    private static final double TICKS_PER_REV = 537.6;

    public static double kP = 2;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0;

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
    }
    @Override
    public void loop(Gamepad gamepad) {
        if (gamepad.b) {
            spool.setTargetPosition(EXTEND_POS);
        } else if (gamepad.a) {
            spool.setTargetPosition(0);
        }
        spool.update();
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("position", spool.getPosition());
        telemetry.addData("velocity", spool.getVelocity());
    }
}
