package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Wobble extends Mechanism{

    private Servo clamp;
    private Servo rotator;

    public Wobble(LinearOpMode opMode){this.opMode = opMode;}

    public void init(HardwareMap hwMap){

        rotator = hwMap.servo.get("wobbleRotator");
        clamp = hwMap.servo.get("wobbleClamp");
    }


}
