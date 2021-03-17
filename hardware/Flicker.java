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

        leg.setPosition(0.5);
        adjustor.setPosition(0.5);
    }

    public void swing(){

        leg.setPosition(1.0);
    }

    public void reset(){

        leg.setPosition(0.0);
    }

    //For Testing

    public void moveF(){

        leg.setPosition(leg.getPosition() + 0.001);
    }

    public void moveB(){

        leg.setPosition(leg.getPosition() - 0.001);
    }


}
