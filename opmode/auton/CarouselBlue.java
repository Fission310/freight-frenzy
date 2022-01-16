package org.firstinspires.ftc.teamcode.opmode.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Carousel;
import org.firstinspires.ftc.teamcode.hardware.Lift;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class CarouselBlue extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Lift lift = new Lift(this);
        Carousel carousel = new Carousel(this);

        ElapsedTime t = new ElapsedTime();

        lift.init(hardwareMap);
        carousel.init(hardwareMap);



        TrajectorySequence toCarousel = drive.trajectorySequenceBuilder(new Pose2d())
                .strafeLeft(4)
                .forward(7)

                .addTemporalMarker(2, () -> {
                    carousel.reverse();
                })
                .addTemporalMarker(8, () ->{
                    carousel.stop();
                })
                .waitSeconds(6)
                .strafeLeft(26)
                .forward(7)
                .build();

        waitForStart();

        lift.toggleSlide();


        drive.followTrajectorySequence(toCarousel);



    }
}
