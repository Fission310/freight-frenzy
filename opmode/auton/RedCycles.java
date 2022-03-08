package org.firstinspires.ftc.teamcode.opmode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Acquirer;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Acquirer.AcquirerState.*;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.FreightSensor;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.OdoLift;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Slides;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

@Config
@Autonomous(group = "red")
public class RedCycles extends LinearOpMode {
    public static double WALL_DISTANCE = 1.1;
    public static double SCORE_POS = -8.1;
    public static double DRIFT_CORRECTION = 0.24;
    public static double WALL_POS = -70.5+(12.5/2.0);

    Drivetrain drive = new Drivetrain(this);
    OdoLift odo = new OdoLift(this);
    Acquirer acquirer = new Acquirer(this);
    Slides slides = new Slides(this);
    FreightSensor sensor = new FreightSensor(this);

    ElapsedTime outtakeDelay = new ElapsedTime();
    ElapsedTime outtakeDuration = new ElapsedTime();

    public static double OUTTAKE_DELAY_TIME = 0.6;
    public static double OUTTAKE_DURATION_TIME = 1;

    ElapsedTime delay = new ElapsedTime();


    public enum AutonState{
        SCORE_CV,
        SCORE,
        WAREHOUSE0,
        WAREHOUSE1,
        WAREHOUSE2
    }
    AutonState autonState;
    Acquirer.AcquirerState acquirerState;

    @Override
    public void runOpMode() throws InterruptedException {

        ScheduledExecutorService service = Executors.newSingleThreadScheduledExecutor();




        drive.init(hardwareMap);
        odo.init(hardwareMap);
        acquirer.init(hardwareMap);
        slides.init(hardwareMap);
        sensor.init(hardwareMap);

        odo.lower();

        Pose2d startPose = new Pose2d(18, WALL_POS);

        Pose2d SCORE = new Pose2d(SCORE_POS, WALL_POS+WALL_DISTANCE);
        Pose2d WAREHOUSE_0 = new Pose2d(47, WALL_POS+WALL_DISTANCE);
        Pose2d WAREHOUSE_1 = new Pose2d(50, WALL_POS+WALL_DISTANCE+DRIFT_CORRECTION);
        Pose2d WAREHOUSE_2 = new Pose2d(53, WALL_POS+WALL_DISTANCE+DRIFT_CORRECTION);
//        private static final Pose2d WAREHOUSE_3 = new Pose2d(56, WALL_POS+WALL_DISTANCE+0.2);
//        private static final Pose2d WAREHOUSE_4 = new Pose2d(59, WALL_POS+WALL_DISTANCE+0.2);

        //Level3 Code
        Runnable retract = new Runnable() {
            @Override
            public void run() {
                slides.retract();
                slides.resetServos();
            }
        };

        Runnable dump3 = new Runnable() {
            @Override
            public void run() {
                slides.dump3();

                service.schedule(retract, (long) Slides.SERVO_DELAY_TIME * 1000, TimeUnit.MILLISECONDS);
            }
        };

        Runnable score3 = new Runnable() {
            @Override
            public void run() {
                slides.extend();

                service.schedule(dump3, (long) (Slides.SLIDES_DELAY_TIME * 1000), TimeUnit.MILLISECONDS);
            }
        };

        // Level 2 Code
        Runnable retract2 = new Runnable() {
            @Override
            public void run() {
                slides.retract2();
                slides.resetServos();
            }
        };

        Runnable dump2 = new Runnable() {
            @Override
            public void run() {
                slides.dump2();

                service.schedule(retract2, (long) Slides.SERVO_DELAY_TIME * 1000, TimeUnit.MILLISECONDS);
            }
        };

        Runnable slides2 = new Runnable() {
            @Override
            public void run() {
                slides.level2Extend();

                service.schedule(dump2, (long) (Slides.SLIDES_DELAY_TIME * 1000), TimeUnit.MILLISECONDS);
            }
        };

        Runnable score2 = new Runnable() {
            @Override
            public void run() {
                slides.temp2();

                service.schedule(slides2, (long) (Slides.SLIDES_DELAY_TIME * 1000), TimeUnit.MILLISECONDS);
            }
        };

        //Level 1 Code
        Runnable retract1 = new Runnable() {
            @Override
            public void run() {
                slides.retract1();
                slides.resetServos();
            }
        };

        Runnable dump1 = new Runnable() {
            @Override
            public void run() {
                slides.dump1();

                service.schedule(retract1, (long) Slides.SERVO_DELAY_TIME * 1000, TimeUnit.MILLISECONDS);
            }
        };

        Runnable slides1 = new Runnable() {
            @Override
            public void run() {
                slides.level2Extend();

                service.schedule(dump1, (long) (Slides.SLIDES_DELAY_TIME * 1000), TimeUnit.MILLISECONDS);
            }
        };

        Runnable score1 = new Runnable() {
            @Override
            public void run() {
                slides.temp1();

                service.schedule(slides1, (long) (Slides.SLIDES_DELAY_TIME * 1000), TimeUnit.MILLISECONDS);
            }
        };

        drive.rrDrive.setPoseEstimate(startPose);

        TrajectorySequence traj = drive.rrDrive.trajectorySequenceBuilder(startPose)

                .lineToLinearHeading(SCORE)

                .addTemporalMarker(() ->{
                    service.submit(score3);
                })
                .waitSeconds(Slides.SLIDES_DELAY_TIME + Slides.SERVO_DELAY_TIME)

                .lineToLinearHeading(WAREHOUSE_0)
                .waitSeconds(0.15)
                .lineToLinearHeading(SCORE)

                .addTemporalMarker(() ->{
                    service.submit(score3);
                })
                .waitSeconds(Slides.SLIDES_DELAY_TIME + Slides.SERVO_DELAY_TIME)


                .lineToLinearHeading(WAREHOUSE_1)
                .waitSeconds(0.15)
                .lineToLinearHeading(SCORE)

                .addTemporalMarker(() ->{
                    service.submit(score3);
                })
                .waitSeconds(Slides.SLIDES_DELAY_TIME + Slides.SERVO_DELAY_TIME)


                .lineToLinearHeading(WAREHOUSE_2)
                .waitSeconds(0.15)
                .lineToLinearHeading(SCORE)

                .addTemporalMarker(() ->{
                    service.submit(score3);
                })
                .waitSeconds(Slides.SLIDES_DELAY_TIME + Slides.SERVO_DELAY_TIME)

                .lineToLinearHeading(WAREHOUSE_0)
                .build();

        autonState = AutonState.SCORE_CV;
        acquirerState = Acquirer.AcquirerState.ACQUIRER_START;

        boolean leftOuttake = false;

        waitForStart();

        acquirer.intake();

        drive.rrDrive.followTrajectorySequenceAsync(traj);

        while(opModeIsActive()){


            drive.rrDrive.update();
            slides.update();

            switch (acquirerState) {
                case ACQUIRER_START:

                    if (sensor.hasFreightLeft()) {
                        leftOuttake = true;

                        acquirerState = Acquirer.AcquirerState.ACQUIRER_DELAY;
                        outtakeDelay.reset();
                    }
                    else if  (sensor.hasFreightRight()) {
                        leftOuttake = false;

                        acquirerState = Acquirer.AcquirerState.ACQUIRER_DELAY;
                        outtakeDelay.reset();
                    }

                    break;
                case ACQUIRER_DELAY:
                    if (outtakeDelay.seconds() >= OUTTAKE_DELAY_TIME) {
                        acquirerState = Acquirer.AcquirerState.ACQUIRER_PREVENT;
                        outtakeDuration.reset();
                    }
                    else {
                        acquirer.intake();
                    }
                    break;
                case ACQUIRER_PREVENT:
                    if (outtakeDuration.seconds() >= OUTTAKE_DURATION_TIME) {
                        acquirerState = Acquirer.AcquirerState.ACQUIRER_START;
                    } else {
                        if(leftOuttake) acquirer.outtakeLeft();
                        else acquirer.outtakeRight();
                    }
            }


        }




    }
}
