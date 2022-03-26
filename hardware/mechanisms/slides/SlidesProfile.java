package org.firstinspires.ftc.teamcode.hardware.mechanisms.slides;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.stuyfission.fissionlib.motion.MotionProfiledDcMotor;
import com.stuyfission.fissionlib.util.Mechanism;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@Config
public class SlidesProfile extends Mechanism {

    MotionProfiledDcMotor spool;
    Carriage carriage = new Carriage(opMode);

    public static double EXTEND_POS_LEVEL3 = 7.3;
    public static double EXTEND_POS_LEVEL2 = 5;
    public static double EXTEND_POS_LEVEL1 = 6;
    public static double EXTEND_POS_SHARED = 3;

    public static double MAX_VEL = 60;
    public static double MAX_ACCEL = 30;
    public static double RETRACTION_MULTIPLIER = 0.6;
    private static final double WHEEL_RADIUS = 1.37795;
    private static final double GEAR_RATIO = 1.0;
    private static final double TICKS_PER_REV = 537.6;

    public static double kP = 0.8;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0;

    public SlidesProfile(LinearOpMode opMode) { this.opMode = opMode; }

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

        carriage.init(hwMap);
    }

    public void rest() {
        spool.setTargetPosition(0);
        carriage.rest();
    }
    public void extendLevel3() {
        spool.setTargetPosition(EXTEND_POS_LEVEL3);
    }
    public void level3Temp() {
        carriage.level3Temp();
    }
    public void level3Tip() {
        carriage.level3Tip();
    }
    public void update() {
        spool.update();
    }

//    @Override
//    public void loop(Gamepad gamepad) {
//        if (gamepad.y) {
//            spool.setTargetPosition(EXTEND_POS_LEVEL3);
//        } else if (gamepad.b) {
//            spool.setTargetPosition(0);
//        }
//        spool.update();
//
//        if (gamepad.x) {
//            carriage.level3Temp();
//        } else if (gamepad.a) {
//            carriage.level3Tip();
//        } else if (gamepad.dpad_down) {
//            carriage.rest();
//        }
//    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("position", spool.getPosition());
        telemetry.addData("velocity", spool.getVelocity());
        carriage.telemetry(telemetry);
    }
}
