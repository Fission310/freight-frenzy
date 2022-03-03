package org.firstinspires.ftc.teamcode.opmode.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class RedCycles extends LinearOpMode {

    private static final double WALL_POS = -70.5+(12.5/2.0);
    private static final Pose2d SCORE = new Pose2d(-11.5, WALL_POS+1.2);
    private static final Pose2d WAREHOUSE_0 = new Pose2d(47, WALL_POS+1.2);
    private static final Pose2d WAREHOUSE_1 = new Pose2d(50, WALL_POS+1.2);
    private static final Pose2d WAREHOUSE_2 = new Pose2d(53, WALL_POS+1.2);
    private static final Pose2d WAREHOUSE_3 = new Pose2d(56, WALL_POS+1.2);
    private static final Pose2d WAREHOUSE_4 = new Pose2d(59, WALL_POS+1.2);

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();
        Pose2d startPose = new Pose2d(12, WALL_POS);
        drive.setPoseEstimate(startPose);
        TrajectorySequence traj = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(SCORE)
                .waitSeconds(0.5)
                .lineToLinearHeading(WAREHOUSE_0)
                .waitSeconds(0.15)
                .lineToLinearHeading(SCORE)
                .waitSeconds(0.5)
                .lineToLinearHeading(WAREHOUSE_1)
                .waitSeconds(0.15)
                .lineToLinearHeading(SCORE)
                .waitSeconds(0.5)
                .lineToLinearHeading(WAREHOUSE_2)
                .waitSeconds(0.15)
                .lineToLinearHeading(SCORE)
                .waitSeconds(0.5)
                .lineToLinearHeading(WAREHOUSE_3)
                .waitSeconds(0.15)
                .lineToLinearHeading(SCORE)
                .waitSeconds(0.5)
                .lineToLinearHeading(WAREHOUSE_4)
                .waitSeconds(0.15)
                .lineToLinearHeading(SCORE)
                .waitSeconds(0.5)
                .lineToLinearHeading(WAREHOUSE_0)
                .build();

        drive.followTrajectorySequence(traj);
    }
}
