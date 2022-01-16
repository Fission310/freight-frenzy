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
public class CarouselRed extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Lift lift = new Lift(this);
        Carousel carousel = new Carousel(this);

        ElapsedTime t = new ElapsedTime();

        lift.init(hardwareMap);
        carousel.init(hardwareMap);



        TrajectorySequence toCarousel = drive.trajectorySequenceBuilder(new Pose2d())
                .back(4)
                .strafeRight(8)
                .addTemporalMarker(2, () -> {
                    carousel.spin();
                })
                .addTemporalMarker(8, () ->{
                    carousel.stop();
                })
                .waitSeconds(6)
                .back(16)
                .strafeRight(5)
                .build();

        waitForStart();



        lift.toggleSlide();

        drive.followTrajectorySequence(toCarousel);



    }
}
