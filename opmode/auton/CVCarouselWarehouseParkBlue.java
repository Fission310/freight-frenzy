package org.firstinspires.ftc.teamcode.opmode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilderKt;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Carousel;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.OdoLift;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Slides;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Webcam;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Webcam.Location;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;

@Autonomous(group = "blue")
@Config
public class CVCarouselWarehouseParkBlue extends LinearOpMode {

    public static double SCORE_DISTANCE = 23;
    public static double CAROUSEL_STRAFE_DISTANCE = 5;
    public static double CAROUSEL_MOVE_DISTANCE = 37;
    public static double PARK_DISTANCE = 65;

    public enum DriveState{
        STRAFE,
        PLACE,
        CAROUSEL,
        PARK,
        FINISH
    }
    DriveState state;

    @Override
    public void runOpMode() throws InterruptedException {
        ScheduledExecutorService service = Executors.newSingleThreadScheduledExecutor();

        Webcam webcam = new Webcam(this);
        Slides slides = new Slides(this);
        OdoLift odo = new OdoLift(this);
        Carousel carousel = new Carousel(this);
        Drivetrain drive = new Drivetrain(this);

        webcam.init(hardwareMap);
        slides.init(hardwareMap);
        drive.init(hardwareMap);
        carousel.init(hardwareMap);
        odo.init(hardwareMap);

        odo.lower();
        drive.rrDrive.setPoseEstimate(new Pose2d());


        state = DriveState.STRAFE;

        TrajectorySequence strafe = drive.rrDrive.trajectorySequenceBuilder(new Pose2d())
                .back(SCORE_DISTANCE)
                .build();

        TrajectorySequence level3 = drive.rrDrive.trajectorySequenceBuilder(strafe.end())
                .forward(0.01)
                .addTemporalMarker(() ->{
                    slides.extend();
                })
                .waitSeconds(Slides.SLIDES_DELAY_TIME)
                .addTemporalMarker(() ->{
                    slides.dump3();
                })
                .waitSeconds(Slides.SERVO_DELAY_TIME)
                .addTemporalMarker(() ->{
                    slides.resetServos();
                    slides.retract();
                })
                .build();

        TrajectorySequence level2 = drive.rrDrive.trajectorySequenceBuilder(strafe.end())
                .forward(0.01)
                .addTemporalMarker(() ->{
                    slides.temp2();
                })
                .waitSeconds(Slides.TEMP2_DELAY_TIME)
                .addTemporalMarker(() ->{
                    slides.level2Extend();
                })
                .waitSeconds(Slides.SLIDES_DELAY_TIME)
                .addTemporalMarker(() ->{
                    slides.dump2();
                })
                .waitSeconds(Slides.DUMP2_DELAY_TIME)
                .addTemporalMarker(() ->{
                    slides.retract2();
                    slides.resetServos();
                })
                .build();

        TrajectorySequence level1 = drive.rrDrive.trajectorySequenceBuilder(strafe.end())
                .forward(0.01)
                .addTemporalMarker(() ->{
                    slides.temp1();
                })
                .waitSeconds(Slides.OMG_TIME)
                .addTemporalMarker(() ->{
                    slides.level1Extend();
                })
                .waitSeconds(Slides.LVL1_DELAY_TIME)
                .addTemporalMarker(() ->{
                    slides.dump1();
                })
                .waitSeconds(Slides.SERVO1_DELAY_TIME)
                .addTemporalMarker(() ->{
                    slides.retract1();
                    slides.resetServos();
                })
                .build();
        TrajectorySequence placeBlock = level3;
        TrajectorySequence toCarousel = level3;
        TrajectorySequence park = level3;


        waitForStart();
        Location location = webcam.location();


        while(opModeIsActive()){


            slides.update();
            drive.rrDrive.update();


            switch (state){
                case STRAFE:

                    if(!drive.rrDrive.isBusy()){
                        drive.rrDrive.followTrajectorySequenceAsync(strafe);
                        state = DriveState.CAROUSEL;
                    }
                    break;
                case PLACE:

                    switch (location) {
                        case LEFT:
                            placeBlock = level1;
                            break;
                        case MIDDLE:
                            placeBlock = level2;
                            break;
                        case RIGHT:
                            placeBlock = level3;
                            break;
                    }

                    if(!drive.rrDrive.isBusy()){
                        drive.rrDrive.followTrajectorySequenceAsync(placeBlock);
                        state = DriveState.CAROUSEL;
                    }

                    break;
                case CAROUSEL:
                    if(!drive.rrDrive.isBusy()){
                        toCarousel = drive.rrDrive.trajectorySequenceBuilder(placeBlock.end())
                                .strafeLeft(CAROUSEL_STRAFE_DISTANCE)
                                .forward(CAROUSEL_MOVE_DISTANCE)
                                .addTemporalMarker(() ->{
                                    carousel.rotate();
                                })
                                .waitSeconds(5)
                                .addTemporalMarker(() ->{
                                    carousel.stop();
                                })
                                .build();

                        drive.rrDrive.followTrajectorySequenceAsync(toCarousel);
                        state = DriveState.FINISH;
                    }
                    break;
                case PARK:
                    if(!drive.rrDrive.isBusy()){
                        park = drive.rrDrive.trajectorySequenceBuilder(toCarousel.end())
                                .back(CAROUSEL_MOVE_DISTANCE)
                                .strafeRight(CAROUSEL_STRAFE_DISTANCE)
                                .back(PARK_DISTANCE)
                                .build();

                        drive.rrDrive.followTrajectorySequenceAsync(park);
                        state = DriveState.FINISH;
                    }
                    break;
                case FINISH:
                    break;
                default:
                    break;

            }
        }


        webcam.stopStreaming();
    }
}
