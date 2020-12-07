package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoTest extends Mechanism{

    public Servo servo;

    public ServoTest(LinearOpMode opMode){this.opMode = opMode;}

    public void init(HardwareMap hwMap){
        servo = hwMap.servo.get("servo");
    }

    public void moveToPos(double pos){
        servo.setPosition(pos);
    }

    public void moveBy(double amount){
        servo.setPosition(servo.getPosition() + amount);
    }

}
