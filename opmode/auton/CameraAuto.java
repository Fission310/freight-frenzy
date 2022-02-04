package org.firstinspires.ftc.teamcode.opmode.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Carousel;
import org.firstinspires.ftc.teamcode.hardware.Lift;
import org.firstinspires.ftc.teamcode.hardware.Webcam;
import org.firstinspires.ftc.teamcode.hardware.Webcam.Location;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;


@Autonomous
public class CameraAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Webcam webcam = new Webcam(this);
        Lift lift = new Lift(this);
        Carousel carousel = new Carousel(this);



        webcam.init(hardwareMap);
        lift.init(hardwareMap);
        carousel.init(hardwareMap);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);



        waitForStart();
        Location location = webcam.location();

        lift.close();
        lift.toggleSlide();

        TrajectorySequence strafe = drive.trajectorySequenceBuilder(new Pose2d())
                .strafeLeft(5)
                .build();




        drive.followTrajectorySequence(strafe);
        


        webcam.stopStreaming();
    }
}
