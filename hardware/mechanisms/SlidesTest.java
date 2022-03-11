package org.firstinspires.ftc.teamcode.hardware.mechanisms;

import com.stuyfission.fissionlib.motion.MotionProfiledDcMotor;
import com.stuyfission.fissionlib.util.Mechanism;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@Config
public class SlidesTest extends Mechanism {
    private MotionProfiledDcMotor spool;

    // Wheel Constants //
    private static final double WHEEL_RADIUS = 1.37795;
    private static final double TICKS_PER_REV = 537.6;
    private static final double GEAR_RATIO = 1.0;

    // PID constants //
    public static double EXTEND_POS = 11;
    public static double MAX_VEL = 10;
    public static double MAX_ACCEL = 10;

    public static double kP = 1.2;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0.08;

    public SlidesTest(LinearOpMode opMode) { this.opMode = opMode; }

    @Override
    public void init(HardwareMap hwMap) {
        spool.initialize(hwMap, "spoolMotor");
        // TODO: reverse if necessary
        spool.setDirection(DcMotorSimple.Direction.REVERSE);

        spool.setWheelConstants(WHEEL_RADIUS, GEAR_RATIO, TICKS_PER_REV);
        spool.setMotionConstraints(MAX_VEL, MAX_ACCEL);
        spool.setPIDCoefficients(kP, kI, kD, kF);

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
        telemetry.addData("Velocity", spool.getVelocity());
        telemetry.addData("Position", spool.getPosition());
    }
}
