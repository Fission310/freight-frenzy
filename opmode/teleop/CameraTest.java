package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Webcam;

@TeleOp
public class CameraTest extends LinearOpMode {

    Webcam camera = new Webcam(this);

    @Override
    public void runOpMode() throws InterruptedException {
        camera.init(hardwareMap);
        camera.setPipeline();

        waitForStart();

        while(opModeIsActive()){
            telemetry.addData("Location",camera.location());
            telemetry.update();
        }
    }
}
