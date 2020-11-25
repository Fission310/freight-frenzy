package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Motor;

@TeleOp(name="MotorTest", group="Test")
public class MotorTest extends LinearOpMode{

    private Motor motor = new Motor(this);

    @Override
    public void runOpMode() throws InterruptedException{

        motor.init(hardwareMap);

        waitForStart();

        while(opModeIsActive()){
            
        }

    }
}
