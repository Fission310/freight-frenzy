package org.firstinspires.ftc.teamcode.opmode.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class RedWarehouseSpline extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();
        Pose2d startPose = new Pose2d(14, -70.5+(15/2.0), Math.toRadians(270));
        drive.setPoseEstimate(startPose);
        TrajectorySequence traj = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(2)
                .setReversed(true)
                .splineTo(new Vector2d(-5,-42), Math.toRadians(110))
                .waitSeconds(2)
                .setReversed(false)
                .splineTo(new Vector2d(20, -70.5+(12.5/2.0)+0.3), Math.toRadians(0))
                .forward(25)
                .build();

        drive.followTrajectorySequence(traj);
    }
}
