package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;


public class Acquirer extends Mechanism {

    private CRServo intake;

    public Acquirer(LinearOpMode opMode){this.opMode = opMode;}

    public void init(HardwareMap hwMap){
        intake = hwMap.crservo.get("intake");
    }

    public void forward(){
        intake.setPower(1);
    }

    public void reverse(){
        intake.setPower(-1);
    }

    public void stop(){
        intake.setPower(0);
    }
}
