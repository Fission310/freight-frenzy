package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift extends Mechanism{

    static float RESET_POS;
    static float LOW_POS;
    static float MID_POS;
    static float HIGH_POS;

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


}
