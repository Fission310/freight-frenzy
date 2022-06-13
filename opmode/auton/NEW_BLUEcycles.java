package org.firstinspires.ftc.teamcode.opmode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
@Config
public class NEW_BLUEcycles extends LinearOpMode {

    private static double HEADING = Math.toRadians(180);

    public static double WAREHOUSE_WAIT = 0.15;

    public static double WALL_POS = 70.5 - (12.15/2.0);

    private static final Pose2d SCORE_0 = new Pose2d(-11.5+3.6, WALL_POS+0.6, HEADING);
    private static final Pose2d SCORE_1 = new Pose2d(-11.5+4.6, WALL_POS+1.6, HEADING);
    private static final Pose2d SCORE_2 = new Pose2d(-11.5+4.6, WALL_POS+1.6, HEADING);
    private static final Pose2d SCORE_3 = new Pose2d(-11.5+5.2, WALL_POS+2.6, HEADING);
    private static final Pose2d SCORE_4 = new Pose2d(-11.5+5.6, WALL_POS+3.6, HEADING);
    private static final Pose2d SCORE_5 = new Pose2d(-11.5+6, WALL_POS+4.6, HEADING);


    private static final Pose2d WAREHOUSE_0 = new Pose2d(47, WALL_POS+0.6, HEADING);
    private static final Pose2d WAREHOUSE_1 = new Pose2d(50, WALL_POS+1.6, HEADING);
    private static final Pose2d WAREHOUSE_2 = new Pose2d(53, WALL_POS+1.6, HEADING);
    private static final Pose2d WAREHOUSE_3 = new Pose2d(56, WALL_POS+2.6, HEADING);
    private static final Pose2d WAREHOUSE_4 = new Pose2d(59, WALL_POS+3.6, HEADING);
    private static final Pose2d WAREHOUSE_5 = new Pose2d(59, WALL_POS+4.6, HEADING);

    private static final Pose2d PARK = new Pose2d(50, WALL_POS+4.6, HEADING);

    @Override public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(12, WALL_POS, HEADING);

        TrajectorySequence cycles = drive.trajectorySequenceBuilder(startPose)
                // CV //
                .lineToLinearHeading(SCORE_0)
                // CV //

                // CYCLE 1 //
                .lineToLinearHeading(WAREHOUSE_0)
                .waitSeconds(WAREHOUSE_WAIT)
                .lineToLinearHeading(SCORE_0)
                // CYCLE 1 //

                // CYCLE 2 //
                .lineToLinearHeading(WAREHOUSE_1)
                .waitSeconds(WAREHOUSE_WAIT)
                .lineToLinearHeading(SCORE_1)
                // CYCLE 2 //

                // CYCLE 3 //
                .lineToLinearHeading(WAREHOUSE_2)
                .waitSeconds(WAREHOUSE_WAIT)
                .lineToLinearHeading(SCORE_2)
                // CYCLE 3 //

                // CYCLE 4 //
                .lineToLinearHeading(WAREHOUSE_3)
                .waitSeconds(WAREHOUSE_WAIT)
                .lineToLinearHeading(SCORE_3)
                // CYCLE 4 //

                // CYCLE 5 //
                .lineToLinearHeading(WAREHOUSE_4)
                .waitSeconds(WAREHOUSE_WAIT)
                .lineToLinearHeading(SCORE_4)
                // CYCLE 5 //

                // CYCLE 6 //
                .lineToLinearHeading(WAREHOUSE_5)
                .waitSeconds(WAREHOUSE_WAIT)
                .lineToLinearHeading(SCORE_5)
                // CYCLE 6 //

                // PARK //
                .lineToLinearHeading(PARK)
                // PARK //

                .build();

        drive.setPoseEstimate(startPose);

        waitForStart();

        drive.followTrajectorySequenceAsync(cycles);

        while(opModeIsActive()) {
            drive.update();
        }
    }
}
