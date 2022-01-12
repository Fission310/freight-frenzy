package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Lift extends Mechanism{

    static double SLIDE_RESET =  0;
    static double SLIDE_LOW = .25;
    static double SLIDE_MID = .75;
    static double SLIDE_HIGH = 1;

    static double CUP_RESET = 0;
    static double CUP_TIP = 1;

    private Servo slide;
    private Servo cup;

    public Lift(LinearOpMode opMode){
        this.opMode = opMode;
    }

    @Override
    public void init(HardwareMap hwMap) {
        slide = hwMap.servo.get("slide");
        cup = hwMap.servo.get("cup");

    }

    public void reset(){
        slide.setPosition(SLIDE_RESET);
    }

    public void low(){
        slide.setPosition(SLIDE_LOW);
    }

    public void mid(){
        slide.setPosition(SLIDE_MID);
    }

    public void high(){
        slide.setPosition(SLIDE_HIGH);
    }

    public void down(){
        cup.setPosition(CUP_RESET);
    }

    public void tip(){
        cup.setPosition(CUP_TIP);
    }





}
