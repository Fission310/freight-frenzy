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

    private Servo leftSlide;
    private Servo rightSlide;

    private Servo freightServo;

    public static double FREIGHT_LOAD = 0;
    public static double FREIGHT_DUMP = 1;

    public static double LEFT_SLIDE_LOAD = 0;
    public static double LEFT_SLIDE_DUMP = 1;

    public static double RIGHT_SLIDE_LOAD = 0;
    public static double RIGHT_SLIDE_DUMP = 1;


    public enum SlidesState {
        SLIDES_START,
        SLIDES_MAX,
        SLIDES_DUMP,
        SLIDES_RESET
    }
    SlidesState slidesState;

    public Slides(LinearOpMode opMode) { this.opMode = opMode; }

    @Override
    public void init(HardwareMap hwMap) {
        leftSlide = hwMap.get(Servo.class, "leftSlide");
        rightSlide = hwMap.get(Servo.class, "rightSlide");
        freightServo = hwMap.get(Servo.class, "freightServo");

        slidesState = SlidesState.SLIDES_START;

        leftSlide.setDirection(Servo.Direction.FORWARD);
        rightSlide.setDirection(Servo.Direction.FORWARD);
        freightServo.setDirection(Servo.Direction.FORWARD);
    }


    public void extend() {

    }

    // tips carriage to score
    public void dumpFreight() {
        leftSlide.setPosition(LEFT_SLIDE_DUMP);
        rightSlide.setPosition(RIGHT_SLIDE_DUMP);
        freightServo.setPosition(FREIGHT_DUMP);
    }

    public void resetFreight(){
        leftSlide.setPosition(LEFT_SLIDE_LOAD);
        rightSlide.setPosition(RIGHT_SLIDE_LOAD);
        freightServo.setPosition(FREIGHT_LOAD);
    }

   public void loop(Gamepad gamepad){
        switch (slidesState){
            case SLIDES_START:
                if(gamepad.b){
                    dumpFreight();
                }
                else if(gamepad.b){
                    resetFreight();
                }
        }
   }


}
