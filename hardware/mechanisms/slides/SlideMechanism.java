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

    public static double EXTEND_POS_LEVEL3 = 7.5; //7.7
    public static double EXTEND_POS_LEVEL2 = 4.7; //5.1
    public static double EXTEND_POS_LEVEL1 = 6;
    public static double EXTEND_POS_SHARED = 1;

    public static double MAX_VEL = 60;
    public static double MAX_ACCEL = 50;
    public static double RETRACTION_MULTIPLIER = 0.35;
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

    public void sharedRestTemp() {
        spool.setTargetPosition(0);
        carriage.restCup();
        carriage.sharedArmRestTemp();
    }
    public void rest() {
        spool.setTargetPosition(0);
        carriage.rest();
    }
    public void sharedRest() {
        spool.setRetractionMultiplier(1);
        spool.setTargetPosition(0);
        carriage.rest();
        spool.setRetractionMultiplier(RETRACTION_MULTIPLIER);
    }
    public void resetServos(){
        carriage.rest();
    }
    public void extendLevel3() {
        spool.setTargetPosition(EXTEND_POS_LEVEL3);
    }
    public void level3Temp() {
        carriage.level3Temp();
    }
    public void level3TempClose() {
        carriage.level3TempClose();
    }
    public void level3Tip() {
        carriage.level3Tip();
    }
    public void level3TipClose() {
        carriage.level3TipClose();
    }

    public void extendLevel2() {
        spool.setTargetPosition(EXTEND_POS_LEVEL2);
    }
    public void level2Temp() {
        carriage.level2Temp();
    }
    public void level2Tip() {
        carriage.level2Tip();
    }
    public void level2ArmTemp() {
        carriage.level2ArmTemp();
    }
    public void level2CupTemp(){
        carriage.level2CupTemp();
    }

    public void extendShared() {
        spool.setTargetPosition(EXTEND_POS_SHARED);
    }
    public void sharedTemp() {
        carriage.sharedTemp();
    }
    public void sharedTip() {
        carriage.sharedTip();
    }

    public void update() {
        spool.update();
    }


    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("position", spool.getPosition());
        telemetry.addData("velocity", spool.getVelocity());
        carriage.telemetry(telemetry);
    }
}
