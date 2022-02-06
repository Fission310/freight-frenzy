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
public class CameraCarouselRed extends LinearOpMode {

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

        lift.toggleRoof();
        lift.toggleSlide();

        TrajectorySequence strafe = drive.trajectorySequenceBuilder(new Pose2d())
                .strafeLeft(28)
                .forward(10)
                .build();

        TrajectorySequence lowPlace = drive.trajectorySequenceBuilder(strafe.end())
                .addTemporalMarker(() ->{
                    lift.lowPlace();
                })
                .waitSeconds(1)
                .back(15)
                .addTemporalMarker(() -> {
                    lift.autonOpen();
                })
                .waitSeconds(1)
                .forward(17)
                .addTemporalMarker(() ->{
                    lift.close();
                    lift.temp();
                })
                .build();

        TrajectorySequence highPlace = drive.trajectorySequenceBuilder(strafe.end())
                .back(15)
                .addTemporalMarker(() ->{
                    lift.highPlace();
                })
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    lift.open();
                })
                .waitSeconds(1)
                .addTemporalMarker(() ->{
                    lift.close();
                    lift.temp();
                })
                .forward(17)
                .build();

        TrajectorySequence midPlace = drive.trajectorySequenceBuilder(strafe.end())
                .addTemporalMarker(() ->{
                    lift.midPlace();
                })
                .waitSeconds(1)
                .back(14)
                .addTemporalMarker(() -> {
                    lift.open();
                })
                .waitSeconds(1)
                .addTemporalMarker(() ->{
                    lift.close();
                    lift.temp();
                })
                .forward(17)
                .build();


        drive.followTrajectorySequence(strafe);

        Pose2d placeEnd = new Pose2d();
        switch (location){
            case LEFT: //low
                drive.followTrajectorySequence(lowPlace);
                placeEnd = lowPlace.end();
                break;

            case MIDDLE:
                drive.followTrajectorySequence(midPlace);
                placeEnd = midPlace.end();
                break;
            case RIGHT:
                drive.followTrajectorySequence(highPlace);
                placeEnd = highPlace.end();
                break;
        }

        TrajectorySequence toCarousel = drive.trajectorySequenceBuilder(placeEnd)
                .back(5)
                .strafeRight(54)
                .forward(6)
                .addTemporalMarker(() -> {
                    carousel.spin();
                })
                .waitSeconds(4)
                .addTemporalMarker(() ->{
                    carousel.stop();
                })
                .build();

        TrajectorySequence park = drive.trajectorySequenceBuilder(toCarousel.end())
                .back(18)
                .strafeRight(8)
                .build();

        drive.followTrajectorySequence(toCarousel);
        drive.followTrajectorySequence(park);



        webcam.stopStreaming();
    }
}