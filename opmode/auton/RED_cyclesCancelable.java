package org.firstinspires.ftc.teamcode.opmode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Acquirer;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.FreightSensor;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
@Config
public class RED_cyclesCancelable extends LinearOpMode {

    public static double WAREHOUSE_WAIT = 0.15;

    public static double WALL_POS = -70.5 + (12.5/2.0);

    private static final Pose2d SCORE_0 = new Pose2d(-11, WALL_POS+1);
    private static final Pose2d SCORE_1 = new Pose2d(-11+1, WALL_POS+1);
    private static final Pose2d SCORE_2 = new Pose2d(-11, WALL_POS+1);
    private static final Pose2d SCORE_3 = new Pose2d(-11+1, WALL_POS+1);
    private static final Pose2d SCORE_4 = new Pose2d(-11+1, WALL_POS+1);
    private static final Pose2d SCORE_5 = new Pose2d(-11+1, WALL_POS+1);

    private static final Pose2d PARK = new Pose2d(41+3, WALL_POS+2);
    private static final Pose2d WAREHOUSE_0 = new Pose2d(44, WALL_POS+1);
    private static final Pose2d WAREHOUSE_1 = new Pose2d(50, WALL_POS+1);
    private static final Pose2d WAREHOUSE_2 = new Pose2d(54, WALL_POS+1);
    private static final Pose2d WAREHOUSE_3 = new Pose2d(55.5, WALL_POS+1);
    private static final Pose2d WAREHOUSE_4 = new Pose2d(57, WALL_POS+1);
    private static final Pose2d WAREHOUSE_5 = new Pose2d(58, WALL_POS+1);

    TrajectorySequence CV;
    TrajectorySequence wh_0;
    TrajectorySequence score_0;
    TrajectorySequence wh_1;
    TrajectorySequence score_1;

    TrajectorySequence park;

    enum TrajState {
        CV,
        WH_0,
        SC_0,
        WH_1,
        SC_1,
        IDLE
    }

    TrajState currentTraj;

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        FreightSensor sensor = new FreightSensor(this);
        sensor.init(hardwareMap);

        Pose2d startPose = new Pose2d(18, WALL_POS);

        CV = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(SCORE_0)

                .build();

        wh_0 = drive.trajectorySequenceBuilder(CV.end())
                .lineToLinearHeading(WAREHOUSE_0)
                .waitSeconds(WAREHOUSE_WAIT)

                .build();

        score_0 = drive.trajectorySequenceBuilder(wh_0.end())
                .lineToLinearHeading(SCORE_0)
                .build();

        wh_1 = drive.trajectorySequenceBuilder(score_0.end())
                .lineToLinearHeading(WAREHOUSE_1)
                .waitSeconds(WAREHOUSE_WAIT)

                .build();

        score_1 = drive.trajectorySequenceBuilder(wh_1.end())
                .lineToLinearHeading(SCORE_1)

                .build();

        park = drive.trajectorySequenceBuilder(score_1.end())
                .lineToLinearHeading(PARK)

                .build();


        drive.setPoseEstimate(startPose);

        waitForStart();

        currentTraj = TrajState.CV;
        drive.followTrajectorySequenceAsync(CV);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        while(opModeIsActive() && !isStopRequested()) {

            switch(currentTraj) {
                case CV:
                    if (!drive.isBusy()) {
                        currentTraj = TrajState.WH_0;
                        drive.followTrajectorySequenceAsync(wh_0);
                    }
                    break;
                case WH_0:
                    if (sensor.hasFreightRight()) {
                        drive.breakFollowing();
                        drive.setDrivePower(new Pose2d());
                    }
                    if (!drive.isBusy()) {
                        currentTraj = TrajState.SC_0;
                        drive.followTrajectorySequenceAsync(score_0);
                    }
                    break;
                case SC_0:
                    if (!drive.isBusy()) {
                        currentTraj = TrajState.WH_1;
                        drive.followTrajectorySequenceAsync(wh_1);
                    }
                    break;
                case WH_1:
                    if (sensor.hasFreightRight()) {
                        drive.breakFollowing();
                        drive.setDrivePower(new Pose2d());
                    }
                    if (!drive.isBusy()) {
                        currentTraj = TrajState.SC_1;
                        drive.followTrajectorySequenceAsync(score_1);
                    }
                    break;
                case SC_1:
                    if (!drive.isBusy()) {
                        currentTraj = TrajState.IDLE;
                        drive.followTrajectorySequenceAsync(park);
                    }
                case IDLE:
                    break;
            }

            drive.update();

            telemetry.addData("current trajectory", currentTraj);
            telemetry.update();
        }
    }
}
