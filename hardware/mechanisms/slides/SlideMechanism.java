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
public class SlideMechanism extends Mechanism {

    MotionProfiledDcMotor spool;
    Carriage carriage = new Carriage(opMode);

    public static double EXTEND_POS_LEVEL3 = 7;
    public static double EXTEND_POS_LEVEL2 = 6;
    public static double EXTEND_POS_LEVEL2_TEMP = 3.5;
    public static double EXTEND_POS_LEVEL1 = 6;
    public static double EXTEND_POS_LEVEL1_TEMP = 3;
    public static double EXTEND_POS_SHARED = 1;
    public static double EXTEND_POS_REST = 1.1;
    public static double EXTEND_POS_REST_TEMP = 2.5;
    public static double EXTEND_POS_CAP = 2;

    public static double MAX_VEL = 80;
    public static double MAX_ACCEL = 80;
    public static double RETRACTION_MULTIPLIER = 0.5;
    private static final double WHEEL_RADIUS = 1.37795;
    private static final double GEAR_RATIO = 1.0;
    private static final double TICKS_PER_REV = 537.6;

    public static double kP = 0.2;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0;

    public SlideMechanism(LinearOpMode opMode) { this.opMode = opMode; }
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

    public void open(){carriage.open();}
    public void close(){carriage.close();}

    public void rest() {
        spool.setTargetPosition(EXTEND_POS_REST);
        carriage.rest();
    }
    public void restCarriage() {
        carriage.rest();
    }
    public void restTEMP() {
        spool.setTargetPosition(EXTEND_POS_REST_TEMP, 1);
    }

    public void extendLevel3() {
        spool.setTargetPosition(EXTEND_POS_LEVEL3);
    }
    public void armLevel3() {
        carriage.level3();
    }

    public void extendLevel2() {
        spool.setTargetPosition(EXTEND_POS_LEVEL2);
    }
    public void extendLevel2TEMP() {
        spool.setTargetPosition(EXTEND_POS_LEVEL2_TEMP);
    }
    public void armLevel2() {
        carriage.level2();
    }

    public void extendLevel1() {
        spool.setTargetPosition(EXTEND_POS_LEVEL1);
    }
    public void extendLevel1TEMP() {
        spool.setTargetPosition(EXTEND_POS_LEVEL1_TEMP);
    }
    public void armLevel1() {
        carriage.level1();
    }

    public void extendCapping() {
        spool.setTargetPosition(EXTEND_POS_CAP);
    }

    public void update() {
        spool.update();
    }

    public boolean hasFreight() {
        return carriage.hasFreight();
    }

    @Override
    public void loop(Gamepad gamepad2) {
//        update();
//        if (gamepad2.y) {
//            spool.setTargetPosition(2);
//            carriage.higherCap();
//        }
//        if (gamepad2.b) {
//            spool.setTargetPosition(2);
//            carriage.midCap();
//        }
//        if (gamepad2.a) {
//            spool.setTargetPosition(2);
//            carriage.lowerCap();
//        }
//        if (gamepad2.x) {
//            rest();
//        }
    }


    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("position", spool.getPosition());
        telemetry.addData("velocity", spool.getVelocity());
        carriage.telemetry(telemetry);
    }
}
