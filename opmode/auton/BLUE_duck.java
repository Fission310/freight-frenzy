package org.firstinspires.ftc.teamcode.opmode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Acquirer;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
@Config
public class BLUE_duck extends LinearOpMode {

    private static final double WALL_POS = -70.5+(12.5/2.0);

    @Override public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Acquirer acquirer = new Acquirer(this);
        acquirer.init(hardwareMap);

        Pose2d startPose = new Pose2d(-31, WALL_POS, Math.toRadians(180));

        TrajectorySequence duckTraj = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(-31, WALL_POS-2, Math.toRadians(180)))
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(-58, 59, Math.toRadians(220)))
                .waitSeconds(4)
                .lineToLinearHeading(new Pose2d(-WALL_POS+2, 23, Math.toRadians(270)))
                .waitSeconds(3)
                .lineToLinearHeading(new Pose2d(-WALL_POS+2, 36, Math.toRadians(270)))
                .build();

        drive.setPoseEstimate(startPose);
        waitForStart();
        drive.followTrajectorySequenceAsync(duckTraj);

        while(opModeIsActive() && !isStopRequested()) {
            drive.update();
        }
    }
}
