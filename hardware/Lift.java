package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Lift extends Mechanism{

    public static double SLIDE_RESET =  1.0;
    public static double SLIDE_HIGH = 0.58;

    public static double CUP_RESET = 0;
    public static double CUP_TEMP = 0.05;
    public static double CUP_TIP = 0.8;

    public static double CUP_LOW = 0.95;
    public static double CUP_MID = 0.8;
    public static double CUP_HIGH = 0.67;

    public static double ROOF_OPEN = 1.0;
    public static double ROOF_CLOSE = 0.0;

    private Servo slide;
    private Servo cup;
    private Servo roof;


    private boolean tipped;
    private boolean raised;

    public Lift(LinearOpMode opMode){
        this.opMode = opMode;
    }

    @Override
    public void init(HardwareMap hwMap) {
        slide = hwMap.servo.get("slide");
        cup = hwMap.servo.get("cup");
        roof = hwMap.servo.get("roof");

        down();
        reset();

        tipped = false;
        raised = false;
    }

    private void reset(){
        ElapsedTime t = new ElapsedTime();
        t.reset();

        while(t.seconds() < 0.25) slide.setPosition(SLIDE_RESET);

        cup.setPosition(CUP_RESET);
    }


    private void high(){
        ElapsedTime t = new ElapsedTime();
        t.reset();

        while(t.seconds() < 0.5) cup.setPosition(CUP_TEMP);

        slide.setPosition(SLIDE_HIGH);

    }

    public void toggleSlide(){

        if(tipped) return;

        if(raised){

            reset();
        }
        else{

            high();
        }

        raised = !raised;
    }

    public void temp(){
        cup.setPosition(CUP_TEMP);
    }

    private void down(){
        cup.setPosition(CUP_RESET);
    }

    private void tip(){
        cup.setPosition(CUP_TIP);
    }

    public void toggleCup(){

        if(!raised) return;

        if(tipped){
            temp();
        }
        else{

            tip();
        }

        tipped = !tipped;
    }

    public void open(){
        roof.setPosition(ROOF_OPEN);
    }

    public void close(){
        roof.setPosition(ROOF_CLOSE);
    }





}
