package org.firstinspires.ftc.teamcode.opmode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.BarcodeDetector;
import org.firstinspires.ftc.teamcode.hardware.Webcam;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@Autonomous
public class CameraAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Webcam webcam = new Webcam(this);
        webcam.init(hardwareMap);


        waitForStart();
        switch (webcam.location()) {
            case LEFT:
                // ...
                break;
            case MIDDLE:
                // ...
                break;
            case RIGHT:
                // ...
                break;
            case NOT_FOUND:
                // ...
        }
        webcam.stopStreaming();
    }
}
