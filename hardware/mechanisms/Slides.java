package org.firstinspires.ftc.teamcode.hardware.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Slides extends Mechanism {

    // ======= SERVOS ======= //
    private Servo leftSlide;
    private Servo rightSlide;
    private Servo freightServo;

    // TODO: tune freight servo
    public static double FREIGHT_LOAD = 0;
    public static double FREIGHT_DUMP1 = 0.5;
    public static double FREIGHT_DUMP3 = 1;

    public static double SLIDE_LOAD = 0;
    public static double SLIDE_DUMP1 = 0.5; // TODO: tune
    public static double SLIDE_DUMP3 = 1;

    // prevent double inputs
    ElapsedTime servoDelay = new ElapsedTime();
    public static double SERVO_DELAY_TIME = 1;

    // ====================== //

    // ======= SPOOL ======= //
    public DcMotorEx spool;

    public static int SLIDE_EXTEND_POS = 13;
    public static int SLIDE_RETRACT_POS = -1;

    public static int TOLERANCE = 1;

    private double targetPosition;

    // PID constants //
    private static double WHEEL_RADIUS = 1.37795;
    private static double TICKS_PER_REV = 537.6;
    private static double GEAR_RATIO = 1.0;

    public static PIDCoefficients coeffs = new PIDCoefficients(0.1, 0, 0);
    PIDFController controller;
    // ============ //

    private boolean lvl3;

    public enum SlidesState {
        SLIDES_START,
        SLIDES_MAX,
        SLIDES_DUMP1,
        SLIDES_DUMP3,
        SERVOS_RESET
    }
    SlidesState slidesState;

    public Slides(LinearOpMode opMode) { this.opMode = opMode; }

    @Override
    public void init(HardwareMap hwMap) {
        leftSlide = hwMap.get(Servo.class, "leftSlide");
        rightSlide = hwMap.get(Servo.class, "rightSlide");
        freightServo = hwMap.get(Servo.class, "freightServo");
        leftSlide.setDirection(Servo.Direction.FORWARD);
        rightSlide.setDirection(Servo.Direction.REVERSE);
        freightServo.setDirection(Servo.Direction.FORWARD);

        spool = hwMap.get(DcMotorEx.class, "slidesMotor");
        spool.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spool.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spool.setDirection(DcMotorSimple.Direction.REVERSE);
        spool.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        targetPosition = SLIDE_RETRACT_POS;
        controller = new PIDFController(coeffs);

        slidesState = SlidesState.SLIDES_START;

        servoDelay.reset();
    }


    public void extend() {
        targetPosition = SLIDE_EXTEND_POS;
        controller.setTargetPosition(SLIDE_EXTEND_POS);
    }
    public void retract() {
        targetPosition = SLIDE_RETRACT_POS;
        controller.setTargetPosition(SLIDE_RETRACT_POS);
    }

    // PID methods //
    public double getPosition() {
        return encoderTicksToInches(spool.getCurrentPosition());
    }
    public double getVelocity() {
        return encoderTicksToInches(spool.getVelocity());
    }
    public void update() {
        double power = controller.update(getPosition(), getVelocity());
        spool.setPower(power);
    }
    // ========= //

    private boolean close() {
        return Math.abs(getPosition() - targetPosition) <= TOLERANCE;
    }

    // tips carriage to score
    public void dump1() {
        leftSlide.setPosition(SLIDE_DUMP1);
        rightSlide.setPosition(SLIDE_DUMP1);
        freightServo.setPosition(FREIGHT_DUMP1);
    }
    public void dump3() {
        leftSlide.setPosition(SLIDE_DUMP3);
        rightSlide.setPosition(SLIDE_DUMP3);
        freightServo.setPosition(FREIGHT_DUMP3);
    }

    public void resetServos(){
        leftSlide.setPosition(SLIDE_LOAD);
        rightSlide.setPosition(SLIDE_LOAD);
        freightServo.setPosition(FREIGHT_LOAD);
    }

   public void loop(Gamepad gamepad){
        switch (slidesState){
            case SLIDES_START:
                resetServos();
                retract();
                if (gamepad.y) { // lvl 3 scoring
                    lvl3 = true;
                    slidesState = SlidesState.SLIDES_MAX;
                } else if (gamepad.x) { // lvl 1 scoring
                    lvl3 = false;
                    slidesState = SlidesState.SLIDES_MAX;
                }
                break;
            case SLIDES_MAX:
                extend();
                if (close() && lvl3) slidesState = SlidesState.SLIDES_DUMP3;
                else if (close() && !lvl3) slidesState = SlidesState.SLIDES_DUMP1;
                break;
            // TODO: slow down actions so carriage can reset before slides retract
            case SLIDES_DUMP1:
                dump1();
                servoDelay.reset();
                slidesState = SlidesState.SERVOS_RESET;
                break;
            case SLIDES_DUMP3:
                dump3();
                servoDelay.reset();
                slidesState = SlidesState.SERVOS_RESET;
                break;
            case SERVOS_RESET:
                resetServos();
                if (servoDelay.seconds() >= SERVO_DELAY_TIME) {
                    slidesState = SlidesState.SLIDES_START;
                }
            default:
                slidesState = SlidesState.SLIDES_START;
        }
   }

   public void telemetry(Telemetry telemetry) {
        telemetry.addData("Current Position", getPosition());
        telemetry.addData("Target Position", targetPosition);
        telemetry.addData("Close?", close());
        telemetry.addData("Current State", slidesState);
        telemetry.addData("servo delay", servoDelay.seconds());
   }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }
    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }
    public static double getMotorVelocityF(double ticksPerSecond) {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 / ticksPerSecond;
    }
}
