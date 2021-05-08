package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;


public class Acquirer extends Mechanism {

    private CRServo intake;
    public Servo servo;
    public DcMotor ramp;

    public Acquirer(LinearOpMode opMode){this.opMode = opMode;}

    public void init(HardwareMap hwMap){

        intake = hwMap.crservo.get("intake");
        servo = hwMap.servo.get("rotator");
        ramp = hwMap.dcMotor.get("ramp");

        reset();
        stop();
    }

    public void forward(){
        ramp.setPower(-1);
        intake.setPower(1);
    }

    public void reverse(){
        ramp.setPower(1);
        intake.setPower(-1);
    }

    public void forward2() {
        ramp.setPower(-1);
    }

    public void stop(){
        intake.setPower(0);
        ramp.setPower(0);
    }

    public void swing(){
        servo.setPosition(0.73);
    }

    public void reset(){
        servo.setPosition(0.12);
    }


}
