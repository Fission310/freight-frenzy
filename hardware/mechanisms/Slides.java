package org.firstinspires.ftc.teamcode.hardware.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Slides extends Mechanism {

    private DcMotor leftSlide;
    private DcMotor rightSlide;

    private Servo freightServo;

    // ===== TESTING ===== //
    public static double SLIDE_RESET =  1.0;
    public static double SLIDE_HIGH = 0.6;

    public static double CUP_TEMP = 0.03;
    public static double CUP_TIP = 0.7;

    private Servo slide;
    private Servo cup;

    ElapsedTime slideTimer = new ElapsedTime();
    ElapsedTime cupTimer = new ElapsedTime();

    public static double DUMP_TIME = 1.5;
    public static double CUP_TIME = DUMP_TIME + 1;
    // =================== //

    public enum SlidesState {
        SLIDES_START,
        SLIDES_MAX,
        SLIDES_DUMP,
        SLIDES_RESET
    }

    SlidesState slidesState = SlidesState.SLIDES_START;

    public Slides(LinearOpMode opMode) { this.opMode = opMode; }

    @Override
    public void init(HardwareMap hwMap) {
        /*
          front (slides)
        |---------------|
        | left    right |
        |---------------|
            wall side
         */
//        leftSlide = hwMap.dcMotor.get("leftSlide");
//        rightSlide = hwMap.dcMotor.get("rightSlide");

        // ===== TESTING ===== //
        slide = hwMap.servo.get("slide");
        cup = hwMap.servo.get("cup");

        slideTimer.reset();
        cupTimer.reset();
        // =================== //
    }

    /*
    FSM:
    Y is pressed -> extend slides
    when slides are fully extended -> dump freightServo
    when freight is dumped -> reset freightServo, reset slide position
    when everything is reset -> back at start
     */

    // need to sync leftSlide and rightSlide
    // extends to max length (wall to hub)
    public void extend() {

    }

    // tips carriage to score
    public void dumpFreight() {

    }

    // ===== TESTING ===== //
    public void loop(Gamepad gamepad1) {
        switch (slidesState) {
            case SLIDES_START:
                if (gamepad1.y) {
                    slide.setPosition(SLIDE_HIGH);
                    slidesState = SlidesState.SLIDES_MAX;
                }
                break;
            case SLIDES_MAX:
                // check if slides have fully extended
                if (Math.abs(slide.getPosition() - SLIDE_HIGH) < 10) {
                    // threshold 10 ticks
                    // tip cup
                    cup.setPosition(CUP_TIP);

                    slideTimer.reset();
                    cupTimer.reset();
                    slidesState = SlidesState.SLIDES_DUMP;
                }
                break;
            case SLIDES_DUMP:
                // if freight is dumped
                if (slideTimer.seconds() >= DUMP_TIME) {
                    // reset freightServo
                    cup.setPosition(CUP_TEMP);
                    if (cupTimer.seconds() >= CUP_TIME) {
                        slide.setPosition(SLIDE_RESET);
                        slidesState = SlidesState.SLIDES_RESET;
                    }
                }
                break;
            case SLIDES_RESET:
                // check if slides have fully retracted
                if (Math.abs(slide.getPosition() - SLIDE_RESET) < 10) {
                    slidesState = SlidesState.SLIDES_START;
                }
                break;
            default:
                slidesState = SlidesState.SLIDES_START;
        }

        if (gamepad1.a && slidesState != SlidesState.SLIDES_START) {
            slidesState = SlidesState.SLIDES_START;
        }
    }
    // =================== //

    public double getCupPos() { return cup.getPosition(); }
    public double getCupTimer() { return cupTimer.seconds(); }

}
