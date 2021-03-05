package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Rotator;
import org.firstinspires.ftc.teamcode.hardware.Acquirer;

@TeleOp(name="CompTest", group="Test")
public class ComponentTest extends LinearOpMode{

    private Drivetrain drive = new Drivetrain(this);
    private Rotator rotator = new Rotator(this);
    private Acquirer acquirer = new Acquirer(this);

    @Override
    public void runOpMode() throws InterruptedException{

        drive.init(hardwareMap);
        rotator.init(hardwareMap);
        acquirer.init(hardwareMap);


        waitForStart();

        while(opModeIsActive()){

            if(gamepad1.a){
                rotator.swing();
            }
            else if(gamepad1.b){
                rotator.reset();
            }

            if(gamepad1.x){
                acquirer.forward();
            }
            else if(gamepad1.y){
                acquirer.reverse();
            }
            else{
                acquirer.stop();
            }


            telemetry.addData("servoPos", rotator.servo.getPosition());
            telemetry.update();

        }

    }
}
