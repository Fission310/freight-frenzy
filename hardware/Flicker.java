package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Flicker extends Mechanism{

    public Servo leg;
    public Servo adjustor;


    public Flicker(LinearOpMode opMode){this.opMode = opMode;}

    public void init(HardwareMap hwMap){

        leg = hwMap.servo.get("leg");
        adjustor = hwMap.servo.get("adjustor");

        swing();
        adjustor.setPosition(0.5);
    }

    public void swing(){

        leg.setPosition(0.4);
    }

    public void reset(){

        leg.setPosition(0.6);
    }

    public void up(){
        adjustor.setPosition(0.52);
    }

    public void down(){
        adjustor.setPosition(1.0);
    }

    //For Testing

    public void moveF(){

        adjustor.setPosition(adjustor.getPosition() + 0.01);
    }

    public void moveB(){

        adjustor.setPosition(adjustor.getPosition() - 0.01);
    }


}
