package org.firstinspires.ftc.teamcode.opmode.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Acquirer;
import org.firstinspires.ftc.teamcode.hardware.FreightSensor;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

@Autonomous
public class NewBotRedCycles extends LinearOpMode {

    private static final double WALL_POS = -70.5+(12.5/2.0);
    private static final Pose2d SCORE = new Pose2d(-11.5, WALL_POS+0.6);
    private static final Pose2d WAREHOUSE_0 = new Pose2d(47, WALL_POS+0.6);
    private static final Pose2d WAREHOUSE_1 = new Pose2d(50, WALL_POS+0.6);
    private static final Pose2d WAREHOUSE_2 = new Pose2d(53, WALL_POS+0.6);
    private static final Pose2d WAREHOUSE_3 = new Pose2d(56, WALL_POS+0.6);



    @Override
    public void runOpMode() throws InterruptedException {
        ScheduledExecutorService service = Executors.newSingleThreadScheduledExecutor();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Acquirer acquirer = new Acquirer(this);
        FreightSensor sensor = new FreightSensor(this);

        acquirer.init(hardwareMap);
        sensor.init(hardwareMap);

        Runnable resetIntake = new Runnable() {
            @Override
            public void run() {
                acquirer.stop();
            }
        };

        Runnable tempReverse = new Runnable() {
            @Override
            public void run() {
                acquirer.reverse();

                service.schedule(resetIntake, 1000, TimeUnit.MILLISECONDS);
            }


        };

        Runnable loop = new Runnable() {
            @Override
            public void run() {
                acquirer.acquire();

                ElapsedTime t = new ElapsedTime();

                while(t.seconds() < 5){
                    if (sensor.hasFreight()) {

                        service.schedule(tempReverse, 200, TimeUnit.MILLISECONDS);
                    }
                }

                acquirer.stop();
            }
        };

        waitForStart();
        Pose2d startPose = new Pose2d(12, WALL_POS);
        drive.setPoseEstimate(startPose);
        TrajectorySequence traj = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(SCORE)
                .waitSeconds(1.2)
                .addTemporalMarker(() -> {
                    service.submit(loop);
                })
                .lineToLinearHeading(WAREHOUSE_0)
                .waitSeconds(0.2)
                .lineToLinearHeading(SCORE)
                .waitSeconds(1.2)
                .lineToLinearHeading(WAREHOUSE_1)
                .waitSeconds(0.2)
                .lineToLinearHeading(SCORE)
                .waitSeconds(1.2)
                .lineToLinearHeading(WAREHOUSE_2)
                .waitSeconds(0.2)
                .lineToLinearHeading(SCORE)
                .waitSeconds(1.2)
                .lineToLinearHeading(WAREHOUSE_3)
                .waitSeconds(0.2)
                .lineToLinearHeading(SCORE)
                .waitSeconds(1.2)
                .lineToLinearHeading(WAREHOUSE_0)
                .build();

        drive.followTrajectorySequence(traj);
    }
}
