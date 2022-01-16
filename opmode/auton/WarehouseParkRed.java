package org.firstinspires.ftc.teamcode.opmode.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Lift;

@Autonomous
public class WarehouseParkRed extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Lift lift = new Lift(this);

        lift.init(hardwareMap);


        waitForStart();

        lift.toggleSlide();

        Trajectory strafe = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(1)
                .build();

        Trajectory park = drive.trajectoryBuilder(strafe.end())
                .splineTo(new Vector2d(30, -2), Math.toRadians(-10))
                .build();

        drive.followTrajectory(strafe);
        drive.followTrajectory(park);

    }
}
