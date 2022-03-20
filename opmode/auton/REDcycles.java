package org.firstinspires.ftc.teamcode.opmode.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class REDcycles extends LinearOpMode {

    private static final double WALL_POS = -70.5+(12.5/2.0);
    private static final Pose2d SCORE = new Pose2d(-11.5, WALL_POS+1);
    private static final Pose2d PARK = new Pose2d(41, WALL_POS+2);
    private static final Pose2d WAREHOUSE_0 = new Pose2d(47, WALL_POS+1);
    private static final Pose2d WAREHOUSE_1 = new Pose2d(50, WALL_POS+1.25);
    private static final Pose2d WAREHOUSE_2 = new Pose2d(53, WALL_POS+1.5);
    private static final Pose2d WAREHOUSE_3 = new Pose2d(47, WALL_POS+1.75);
    private static final Pose2d WAREHOUSE_4 = new Pose2d(59, WALL_POS+1);

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();
        Pose2d startPose = new Pose2d(18, WALL_POS);
        drive.setPoseEstimate(startPose);
        TrajectorySequence traj = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(1)
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
//                                .lineToLinearHeading(WAREHOUSE_4)
//                                .waitSeconds(0.15)
//                                .lineToLinearHeading(SCORE)
//                                .waitSeconds(0.5)
                .lineToLinearHeading(PARK)
                .build();

        drive.followTrajectorySequence(traj);
    }
}
