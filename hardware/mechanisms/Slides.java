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

    public static String cameraLvl;
    public static boolean done;

    // ======= SERVOS ======= //
    private Servo leftSlide;
    private Servo rightSlide;
    private Servo freightServo;

    // TODO: tune freight servo
    public static double FREIGHT_LOAD = 0.48;
    public static double FREIGHT_DUMP1 = 1;
    public static double FREIGHT_TEMP1 = 0.7;
    public static double FREIGHT_TEMP2 = 0;
    public static double FREIGHT_DUMP2 = 0.25;
    public static double FREIGHT_DUMP3 = 0.6;
    public static double FREIGHT_SHARED = 1;

    public static double SLIDE_LOAD = 0.15;
    public static double SLIDE_DUMP1 = 0; // TODO: tune
    public static double SLIDE_TEMP2 = 1; // TODO: tune
    public static double SLIDE_DUMP3 = 0.7;

    // Timers
    ElapsedTime servoDelay = new ElapsedTime();
    ElapsedTime servo1Delay = new ElapsedTime();
    ElapsedTime slidesDelay = new ElapsedTime();
    ElapsedTime sharedDelay = new ElapsedTime();
    ElapsedTime temp2Delay = new ElapsedTime();
    ElapsedTime dump2Delay = new ElapsedTime();
    ElapsedTime lvl1Delay = new ElapsedTime();

    ElapsedTime omg = new ElapsedTime();
    public static double OMG_TIME = 0.5;

    public static double SERVO_DELAY_TIME = 0.7;
    public static double SERVO1_DELAY_TIME = 1.6;
    public static double SLIDES_DELAY_TIME = 0.1;
    public static double SHARED_DELAY_TIME = 0.5;
    public static double TEMP2_DELAY_TIME = 1;
    public static double DUMP2_DELAY_TIME = 0.8;
    public static double LVL1_DELAY_TIME = 0;

    // ====================== //

    // ======= SPOOL ======= //
    public DcMotorEx spool;

    public static double SLIDE_EXTEND_POS = 7.5;
    public static double SLIDE_LEVEL2_POS = 4.5;
    public static double SLIDE_LEVEL1_POS = 7;
    public static double SLIDE2_RETRACT_POS = -2;
    public static double SLIDE1_RETRACT_POS = -2.2;
    public static double SLIDE_RETRACT_POS = -1;
    public static double SLIDE_SHARED_RETRACT_POS = -0.3;
    public static double SLIDE_SHARED_POS = 1.85;

    public static double TOLERANCE = 4;

    private double targetPosition;

    // PID constants //
    private static double WHEEL_RADIUS = 1.37795;
    private static double TICKS_PER_REV = 537.6;
    private static double GEAR_RATIO = 1.0;
    public static double RETRACT_MULTIPLIER = 0.05;

    public static double kF = 0.08;

    public static PIDCoefficients coeffs = new PIDCoefficients(1.2, 0, 0);
    PIDFController controller = new PIDFController(coeffs, 0, 0, 0, (position, velocity) -> kF);
    // ============ //

    public int lvl = 0;

    public enum SlidesState {
        SLIDES_START,
        SLIDES_MAX,
        SLIDES_DUMP1,
        SLIDES_TEMP1,
        SLIDES_DUMP3,
        SLIDES_DUMP2,
        SLIDES_TEMP2,
        SLIDES_SHARED,
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
        freightServo.setDirection(Servo.Direction.REVERSE);

        spool = hwMap.get(DcMotorEx.class, "slidesMotor");
        spool.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spool.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spool.setDirection(DcMotorSimple.Direction.REVERSE);
        spool.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        targetPosition = SLIDE_RETRACT_POS;

        retract();
        resetServos();

        slidesState = SlidesState.SLIDES_START;

        servoDelay.reset();

        cameraLvl = "NULL";
        done = false;
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
        if((targetPosition == SLIDE_RETRACT_POS) || (targetPosition == SLIDE2_RETRACT_POS) || (targetPosition == SLIDE1_RETRACT_POS)) power *= RETRACT_MULTIPLIER;

        spool.setPower(power);
    }

    public void setTargetPosition(double target){
        targetPosition = target;
        controller.setTargetPosition(target);
    }
    // ========= //

    public void extend() {
        setTargetPosition(SLIDE_EXTEND_POS);
    }

    public void level2Extend() {
        setTargetPosition(SLIDE_LEVEL2_POS);
    }

    public void level1Extend() {
        setTargetPosition(SLIDE_LEVEL1_POS);
    }
    public void retract() {
        setTargetPosition(SLIDE_RETRACT_POS);
    }
    public void retract1() {
        setTargetPosition(SLIDE1_RETRACT_POS);
    }
    public void retract2() {
        setTargetPosition(SLIDE2_RETRACT_POS);
    }

    public void retractFar(){
        setTargetPosition(SLIDE_SHARED_RETRACT_POS);
    }


    public void shared(){
        setTargetPosition(SLIDE_SHARED_POS);
    }

    private boolean close() {
        return Math.abs(getPosition() - targetPosition) <= TOLERANCE;
    }

    // tips carriage to score
    public void temp1() {
        freightServo.setPosition(FREIGHT_TEMP1);
    }
    public void dump1() {
        leftSlide.setPosition(SLIDE_DUMP1);
        rightSlide.setPosition(SLIDE_DUMP1);
        freightServo.setPosition(FREIGHT_DUMP1);
    }

    public void temp2(){
        leftSlide.setPosition(SLIDE_TEMP2);
        rightSlide.setPosition(SLIDE_TEMP2);
        freightServo.setPosition(FREIGHT_TEMP2);
    }

    public void dump2(){
        leftSlide.setPosition(SLIDE_TEMP2);
        rightSlide.setPosition(SLIDE_TEMP2);
        freightServo.setPosition(FREIGHT_DUMP2);
    }


    public void dump3() {
        leftSlide.setPosition(SLIDE_DUMP3);
        rightSlide.setPosition(SLIDE_DUMP3);
        freightServo.setPosition(FREIGHT_DUMP3);
    }

    public void dumpShared(){
        leftSlide.setPosition(SLIDE_LOAD);
        rightSlide.setPosition(SLIDE_LOAD);
        freightServo.setPosition(FREIGHT_SHARED);
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
                if (gamepad.y) { // lvl 3 scoring
                    lvl = 3;
                    slidesDelay.reset();
                    slidesState = SlidesState.SLIDES_MAX;
                } else if (gamepad.a){
                    lvl = 2;
                    temp2Delay.reset();
                    slidesState = SlidesState.SLIDES_TEMP2;
                } else if (gamepad.x) { // lvl 1 scoring
                    lvl = 1;
                    slidesDelay.reset();
                    omg.reset();
                    slidesState = SlidesState.SLIDES_MAX;
                } else if(gamepad.b){
                    sharedDelay.reset();
                    slidesState = SlidesState.SLIDES_SHARED;
                }
                break;
            case SLIDES_MAX:
                if(lvl == 3) extend();
                else if(lvl == 2) level2Extend();

                if(slidesDelay.seconds() >= SLIDES_DELAY_TIME){
                    if(lvl == 3) slidesState = SlidesState.SLIDES_DUMP3;
                    else if(lvl == 2) {
                        dump2Delay.reset();
                        slidesState = SlidesState.SLIDES_DUMP2;
                    }
                    else if(lvl == 1) {
                        slidesState = SlidesState.SLIDES_TEMP1;
                    }

                    servoDelay.reset();
                    servo1Delay.reset();
                }

                break;
            // TODO: slow down actions so carriage can reset before slides retract
            case SLIDES_TEMP1:
                temp1();
                if(omg.seconds() >= OMG_TIME) {
                    level1Extend();
                    slidesState = SlidesState.SLIDES_DUMP1;
                    lvl1Delay.reset();
                }
                break;
            case SLIDES_DUMP1:
                if (lvl1Delay.seconds() >= LVL1_DELAY_TIME) {
                    dump1();

                    if (servo1Delay.seconds() >= SERVO1_DELAY_TIME) {
                        retract1();
                        slidesState = SlidesState.SLIDES_START;
                        done = true;
                    }
                }
                break;
            case SLIDES_TEMP2: // Initial servo temp position
                temp2();
                if (temp2Delay.seconds() >= TEMP2_DELAY_TIME) {
                    slidesState = SlidesState.SLIDES_MAX;
                }
                break;
            case SLIDES_DUMP2: // Actually placing
                dump2();
                if (dump2Delay.seconds() >= DUMP2_DELAY_TIME) {
                    retract2();
                    slidesState = SlidesState.SLIDES_START;
                    done = true;
                }
                break;
            case SLIDES_DUMP3:
                dump3();
                if (servoDelay.seconds() >= SERVO_DELAY_TIME) {
                    retract();
                    slidesState = SlidesState.SLIDES_START;
                    done = true;
                }
                break;
            case SLIDES_SHARED:
                shared();
                dumpShared();
                if (sharedDelay.seconds() >= SHARED_DELAY_TIME) {
                    retractFar();
                    slidesState = SlidesState.SLIDES_START;
                }
                break;
//            case SERVOS_RESET:
//                resetServos();
//                slidesState = SlidesState.SLIDES_START;
//                break;
            default:
                slidesState = SlidesState.SLIDES_START;
        }
        update();
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
