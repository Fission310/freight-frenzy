package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.hardware.ServoTest;

@TeleOp(name="MotorTest", group="Test")
public class MotorTest extends LinearOpMode{

    private Motor motor = new Motor(this);
    private ServoTest servo = new ServoTest(this);

    @Override
    public void runOpMode() throws InterruptedException{

        motor.init(hardwareMap);
        servo.init(hardwareMap);

        waitForStart();

        while(opModeIsActive()){

            if(gamepad1.a){
                motor.rotate();
            }
            else if(gamepad1.b){
                motor.backwards();
            }
            else if(gamepad1.x){
                servo.moveToPos(0.9);
            }
            else if(gamepad1.y){
                servo.moveBy(-0.1);
            }
        }

    }
}
