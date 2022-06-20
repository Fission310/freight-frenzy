package org.firstinspires.ftc.teamcode.opmode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Acquirer;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Carousel;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Webcam;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Webcam.Location;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.slides.SlideMechanism;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
@Config
public class RED_duck extends LinearOpMode {

    private static final double WALL_POS = -70.5+(12.5/2.0);

    @Override public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Acquirer acquirer = new Acquirer(this);
        acquirer.init(hardwareMap);

        Carousel carousel = new Carousel(this);
        carousel.init(hardwareMap);

        Webcam webcam = new Webcam(this);
        webcam.init(hardwareMap);

        SlideMechanism slides = new SlideMechanism(this);
        slides.init(hardwareMap);

        Pose2d startPose = new Pose2d(-31, WALL_POS);

        TrajectorySequence duckTraj_LEVEL1 = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(-31, WALL_POS+4))
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(-57.5, -60, Math.toRadians(-30)))
                .addTemporalMarker(carousel::reverseAUTO)
                .addTemporalMarker(8, carousel::stop)
                .waitSeconds(8)
                .lineToLinearHeading(new Pose2d(-60, -23, Math.toRadians(-90)))
                .waitSeconds(1)

                // SCORE WOW //
                .addTemporalMarker(() -> {
                    slides.extendLevel1TEMP();
                    slides.close();
                })
                .waitSeconds(1)
                .addTemporalMarker(slides::armLevel1)
                .waitSeconds(0.7)
                .addTemporalMarker(slides::extendLevel1)
                .waitSeconds(1.5)
                .addTemporalMarker(slides::open)
                .waitSeconds(0.2)
                .addTemporalMarker(slides::restTEMP)
                .waitSeconds(0.4)
                .addTemporalMarker(slides::restCarriage)
                .waitSeconds(1)
                .addTemporalMarker(slides::rest)
                .waitSeconds(5)
                // SCORE WOW //

//                // SCORE //
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    slides.extendLevel1TEMP();
//                    slides.close();
//                })
//                .UNSTABLE_addTemporalMarkerOffset(1.3, slides::armLevel1)
//                .UNSTABLE_addTemporalMarkerOffset(1.7, slides::extendLevel1)
//                .UNSTABLE_addTemporalMarkerOffset(1.5, slides::open)
//                .UNSTABLE_addTemporalMarkerOffset(1.2, slides::restTEMP)
//                .UNSTABLE_addTemporalMarkerOffset(1.4, slides::restCarriage)
//                .UNSTABLE_addTemporalMarkerOffset(2, slides::rest)
//                .waitSeconds(4)
//                // SCORE //

                .lineToLinearHeading(new Pose2d(WALL_POS+2, -34, Math.toRadians(-90)))
                .build();

        TrajectorySequence duckTraj_LEVEL2 = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(-31, WALL_POS+4))
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(-60, -60, Math.toRadians(-30)))
                .addTemporalMarker(carousel::reverseAUTO)
                .addTemporalMarker(8, carousel::stop)
                .waitSeconds(8)
                .lineToLinearHeading(new Pose2d(-57, -23, Math.toRadians(-90)))
                .waitSeconds(0.2)
                .lineToLinearHeading(new Pose2d(-52, -23, Math.toRadians(-90)))

                // SCORE //
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    slides.extendLevel2TEMP();
                    slides.close();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.3, slides::armLevel2)
                .UNSTABLE_addTemporalMarkerOffset(0.7, slides::extendLevel2)
                .UNSTABLE_addTemporalMarkerOffset(0.5, slides::open)
                .UNSTABLE_addTemporalMarkerOffset(0.2, slides::restTEMP)
                .UNSTABLE_addTemporalMarkerOffset(0.4, slides::restCarriage)
                .UNSTABLE_addTemporalMarkerOffset(1, slides::rest)
                .waitSeconds(4)
                // SCORE //

                .lineToLinearHeading(new Pose2d(WALL_POS+2, -34, Math.toRadians(-90)))
                .build();

        TrajectorySequence duckTraj_LEVEL3 = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(-31, WALL_POS+4))
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(-60, -60, Math.toRadians(-30)))
                .addTemporalMarker(carousel::reverse)
                .addTemporalMarker(8, carousel::stop)
                .waitSeconds(8)
                .lineToLinearHeading(new Pose2d(-57, -23, Math.toRadians(-90)))
                .waitSeconds(1)
//                .lineToLinearHeading(new Pose2d(-52, -23, Math.toRadians(-90)))

                // SCORE //
                .addTemporalMarker(slides::extendLevel3)
                .waitSeconds(4)
                // SCORE //

                .lineToLinearHeading(new Pose2d(WALL_POS+2, -34, Math.toRadians(-90)))
                .build();

        drive.setPoseEstimate(startPose);
        waitForStart();

        Location location = webcam.location();

        switch (location) {
            case LEFT:
                drive.followTrajectorySequenceAsync(duckTraj_LEVEL1);
                break;
            case MIDDLE:
                drive.followTrajectorySequenceAsync(duckTraj_LEVEL2);
                break;
            case RIGHT:
                drive.followTrajectorySequenceAsync(duckTraj_LEVEL3);
                break;
        }

        webcam.stopStreaming();

        while(opModeIsActive() && !isStopRequested()) {
            slides.update();
            drive.update();
        }
    }
}
