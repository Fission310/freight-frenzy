package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Leg extends Mechanism{

    public Servo servo;

    public Leg(LinearOpMode opMode){this.opMode = opMode;}

    public void init(HardwareMap hwMap){
        servo = hwMap.servo.get("leg");
    }

    public void swing(){
        servo.setPosition(1);
    }

    public void reset(){
        servo.setPosition(0);
    }

    //For Testing

    public void moveF(){
        servo.setPosition(servo.getPosition() + 0.001);
    }

    public void moveB(){
        servo.setPosition(servo.getPosition() - 0.001);
    }


}
