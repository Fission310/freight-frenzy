package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Flicker;

@TeleOp(name="CompTest", group="Test")
public class ComponentTest extends LinearOpMode{

    private Drivetrain drive = new Drivetrain(this);
    private Flicker flicker = new Flicker(this);

    @Override
    public void runOpMode() throws InterruptedException{

        drive.init(hardwareMap);
        flicker.init(hardwareMap);

        waitForStart();

        while(opModeIsActive()){

//            if(gamepad1.a)drive.frontLeft.setPower(1);
//            else drive.frontLeft.setPower(0);
//
//            if(gamepad1.b)drive.frontRight.setPower(1);
//            else drive.frontRight.setPower(0);
//
//            if(gamepad1.x)drive.backLeft.setPower(1);
//            else drive.backLeft.setPower(0);
//
//            if(gamepad1.y)drive.backRight.setPower(1);
//            else drive.backRight.setPower(0);

            if(gamepad1.a){
                flicker.swing();
            }
            else if(gamepad1.b){
                flicker.reset();
            }

            telemetry.addData("servoPos", flicker.servo.getPosition());
            telemetry.update();

        }

    }
}
