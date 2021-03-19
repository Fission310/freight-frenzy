package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Wobble extends Mechanism{

    private Servo clamp;
    public Servo rotator;

    public Wobble(LinearOpMode opMode){this.opMode = opMode;}

    public void init(HardwareMap hwMap){

        rotator = hwMap.servo.get("wobbleRotator");
        clamp = hwMap.servo.get("wobbleClamp");

        rotator.setPosition(0.5);
    }

    public void moveF(){
        rotator.setPosition(rotator.getPosition() + 0.001);
    }

    public void moveB(){
        rotator.setPosition(rotator.getPosition() - 0.001);
    }


}
