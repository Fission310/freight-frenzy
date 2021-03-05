package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Rotator extends Mechanism{

    public Servo servo;

    public Rotator(LinearOpMode opMode){this.opMode = opMode;}

    public void init(HardwareMap hwMap){
        servo = hwMap.servo.get("rotator");
        reset();
    }

    public void swing(){
        servo.setPosition(0.73);
    }

    public void reset(){
        servo.setPosition(0.10);
    }
}
