package org.firstinspires.ftc.teamcode.hardware.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.stuyfission.fissionlib.util.Mechanism;
import com.stuyfission.fissionlib.motion.MotionProfiledDcMotor;


@Config
public class SlidesTest extends Mechanism {

    MotionProfiledDcMotor spool;

    public static double EXTEND_POS = 11;
    public static double MAX_VEL = 10;
    public static double MAX_ACCEL = 10;
    private static final double WHEEL_RADIUS = 1.37795;
    private static final double GEAR_RATIO = 1.0;
    private static final double TICKS_PER_REV = 537.6;

    public static double kP = 0.2;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0;

    public SlidesTest(LinearOpMode opMode) { this.opMode = opMode; }

    @Override
    public void init(HardwareMap hwMap) {
        // init motor
        spool = new MotionProfiledDcMotor(hwMap, "slidesMotor");
        spool.setWheelConstants(WHEEL_RADIUS, GEAR_RATIO, TICKS_PER_REV);
        spool.setMotionConstraints(MAX_VEL, MAX_ACCEL);
        spool.setPIDCoefficients(kP, kI, kD, kF);
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
}
