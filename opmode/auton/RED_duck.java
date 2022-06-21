package org.firstinspires.ftc.teamcode.opmode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Acquirer;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Carousel;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Slides;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Webcam;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Webcam.Location;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.slides.SlideMechanism;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group = "_red")
@Config
public class RED_duck extends LinearOpMode {

    private static final double WALL_POS = -70.5+(12.5/2.0);

    private enum TrajState {
        CAROUSEL,
        SCORING,
        PARKING,
        IDLING
    }
    private TrajState trajState;

    Slides.SlidesState slidesState;

    ElapsedTime time = new ElapsedTime();
    public static double TIP_WAIT = 2;

    public static double SCORING_POS = -58;
    @Override public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Acquirer acquirer = new Acquirer(this);
        Carousel carousel = new Carousel(this);
        Webcam webcam = new Webcam(this);
        SlideMechanism slides = new SlideMechanism(this);

        acquirer.init(hardwareMap);
        carousel.init(hardwareMap);
        webcam.init(hardwareMap);
        slides.init(hardwareMap);

        trajState = TrajState.CAROUSEL;
        slidesState = Slides.SlidesState.WAIT;

        slides.close();

        Pose2d startPose = new Pose2d(-31, WALL_POS);

        TrajectorySequence carouselTraj = drive.trajectorySequenceBuilder(startPose)

                // SCAN 4 DUCK
                .waitSeconds(0.2)
                .lineToLinearHeading(new Pose2d(-31, WALL_POS+4))
                .waitSeconds(0.2)
                .lineToLinearHeading(new Pose2d(-60, -59, Math.toRadians(-37)))
                .addTemporalMarker(carousel::reverseAUTO)
                .waitSeconds(8)
                .addTemporalMarker(carousel::stop)
                .lineToLinearHeading(new Pose2d(WALL_POS-4, -20, Math.toRadians(-90)))
                .waitSeconds(0.1)
                .lineToLinearHeading(new Pose2d(SCORING_POS, -20, Math.toRadians(-90)))
                .waitSeconds(0.5)

                .build();

        TrajectorySequence parkTraj = drive.trajectorySequenceBuilder(carouselTraj.end())
                .lineToLinearHeading(new Pose2d(WALL_POS-5, -34, Math.toRadians(-90)))

                .build();

        drive.setPoseEstimate(startPose);
        waitForStart();

        Location location = webcam.location();
        webcam.stopStreaming();

        drive.followTrajectorySequenceAsync(carouselTraj);

        while(opModeIsActive() && !isStopRequested()) {

            slides.update();
            drive.update();

            switch (trajState) {
                case CAROUSEL:
                    if (!drive.isBusy()) {
                        trajState = TrajState.SCORING;
                    }
                    break;
                case SCORING:
                    switch (slidesState) {
                        case WAIT:
                            switch (location) {
                                case LEFT:
                                    slides.extendLevel1TEMP();
                                    slides.close();

                                    time.reset();
                                    slidesState = Slides.SlidesState.DELAY;
                                    break;
                                case MIDDLE:
                                    slides.extendLevel2TEMP();
                                    slides.close();

                                    time.reset();
                                    slidesState = Slides.SlidesState.DELAY;
                                    break;
                                case RIGHT:
                                    slides.extendLevel3();
                                    slides.close();

                                    time.reset();
                                    slidesState = Slides.SlidesState.DELAY;
                                    break;
                            }
                            break;
                        case DELAY:
                            switch (location) {
                                case LEFT:
                                    if (time.seconds() > Slides.LEVEL1_ARM_TEMP_WAIT) {
                                        slides.armLevel1();
                                    }
                                    if (time.seconds() > Slides.LEVEL1_TEMP_WAIT) {
                                        slides.extendLevel1();

                                        slidesState = Slides.SlidesState.TIP;
                                        time.reset();
                                    }
                                    break;
                                case MIDDLE:
                                    if (time.seconds() > Slides.LEVEL1_ARM_TEMP_WAIT) {
                                        slides.armLevel2();
                                    }
                                    if (time.seconds() > Slides.LEVEL2_TEMP_WAIT) {
                                        slides.extendLevel2();

                                        slidesState = Slides.SlidesState.TIP;
                                        time.reset();
                                    }
                                    break;
                                case RIGHT:
                                    if (time.seconds() > Slides.LEVEL3_TEMP_WAIT) {
                                        slides.armLevel3();

                                        slidesState = Slides.SlidesState.TIP;
                                        time.reset();
                                    }
                                    break;
                            }
                            break;
                        case TIP:
                            if (time.seconds() > TIP_WAIT) {
                                slides.open();

                                time.reset();
                                slidesState = Slides.SlidesState.TEMP_RETRACT;
                            }
                            break;
                        case TEMP_RETRACT:
                            if (time.seconds() > Slides.TEMP_RETRACT_WAIT) {
                                time.reset();
                                switch (location) {
                                    case LEFT:
                                    case MIDDLE:
                                        slides.restTEMP();
                                        slidesState = Slides.SlidesState.TEMP_CARRIAGE;
                                        time.reset();
                                        break;
                                    case RIGHT:
                                        slidesState = Slides.SlidesState.TIP_DELAY;
                                        time.reset();
                                        break;
                                }
                            }
                            break;
                        case TEMP_CARRIAGE:
                            if (time.seconds() > Slides.TEMP_CARRIAGE_WAIT) {
                                slides.restCarriage();
                                time.reset();
                                slidesState = Slides.SlidesState.TIP_DELAY;
                            }
                            break;
                        case TIP_DELAY:
                            switch (location) {
                                case LEFT:
                                    if (time.seconds() > Slides.LEVEL1_TIP_WAIT) {
                                        time.reset();
                                        slides.rest();
                                        trajState = TrajState.PARKING;
                                    }
                                    break;
                                case MIDDLE:
                                    if (time.seconds() > Slides.LEVEL2_TIP_WAIT) {
                                        time.reset();
                                        slides.rest();
                                        trajState = TrajState.PARKING;
                                    }
                                    break;
                                case RIGHT:
                                    if (time.seconds() > Slides.LEVEL3_TIP_WAIT) {
                                        time.reset();
                                        slides.rest();
                                        trajState = TrajState.PARKING;
                                    }
                                    break;
                            }
                            break;
                    }
                    break;
                case PARKING:
                    drive.followTrajectorySequenceAsync(parkTraj);
                    trajState = TrajState.IDLING;
                    break;
                case IDLING:
                    // wait for parking to finish
                    break;
            }
        }
    }
}
