package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Camera;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;

@TeleOp(name="AutonTest", group="Test")
public class AutonDrive extends LinearOpMode {

    private Drivetrain drive = new Drivetrain(this);
    private Camera camera = new Camera(this);

    @Override
    public void runOpMode() throws InterruptedException{

        drive.init(hardwareMap);
        camera.init(hardwareMap);

        waitForStart();

        while(opModeIsActive()){
            camera.listDetections();

            if(gamepad1.x){
                drive.driveToPos(10, 0.8);
            }
            else if(gamepad1.y){
                drive.driveToPos(-10, 0.8);
            }

            if(gamepad1.a){
                drive.turn(90, 0.8);
            }
            else if(gamepad1.b){
                drive.turn(-90, 0.8);
            }

            if(gamepad1.dpad_right){
                drive.strafePID(0.8, 0.5);
            }
            else if(gamepad1.dpad_left){
                drive.strafePID(-0.8, 0.5);
            }

        }

        camera.shutdown();
    }
}
