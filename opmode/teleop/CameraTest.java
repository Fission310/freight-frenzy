package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Camera;

@TeleOp
public class CameraTest extends LinearOpMode {

    Camera camera = new Camera(this);

    @Override
    public void runOpMode() throws InterruptedException {
        camera.init(hardwareMap);

        telemetry.addData("test", "test");
        telemetry.update();


        waitForStart();

        while(opModeIsActive()){


            camera.listDetections();
        }
    }
}
