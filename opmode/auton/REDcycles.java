package org.firstinspires.ftc.teamcode.opmode.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Acquirer;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Slides;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Webcam;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.slides.SlideMechanism;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class REDcycles extends LinearOpMode {

    private static final double WALL_POS = -70.5+(12.5/2.0);
    private static final Pose2d SCORE_0 = new Pose2d(-11, WALL_POS+1);
    private static final Pose2d SCORE_1 = new Pose2d(-11+1.5, WALL_POS+1);
    private static final Pose2d SCORE_2 = new Pose2d(-11+2, WALL_POS+1);
    private static final Pose2d SCORE_3 = new Pose2d(-11+2.5, WALL_POS+1);

    private static final Pose2d PARK = new Pose2d(41, WALL_POS+2);
    private static final Pose2d WAREHOUSE_0 = new Pose2d(47, WALL_POS+1);
    private static final Pose2d WAREHOUSE_1 = new Pose2d(50, WALL_POS+1.25);
    private static final Pose2d WAREHOUSE_2 = new Pose2d(53, WALL_POS+2);
    private static final Pose2d WAREHOUSE_3 = new Pose2d(57, WALL_POS+2.25);
    private static final Pose2d WAREHOUSE_4 = new Pose2d(59, WALL_POS+1);

    enum AutonState{
        PRELOAD_PLACE,
        CYCLES,
        IDLE
    }
    AutonState state;

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        SlideMechanism slides = new SlideMechanism(this);
        Acquirer acquirer = new Acquirer(this);


        slides.init(hardwareMap);
        acquirer.init(hardwareMap);
        state = AutonState.PRELOAD_PLACE;


        Pose2d startPose = new Pose2d(18, WALL_POS);
        TrajectorySequence preLoadPlace = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() ->{
                    slides.extendLevel2();
                })
                .addTemporalMarker(() ->{
                    slides.level2Temp();
                })
                .lineToLinearHeading(SCORE_0)
                .addTemporalMarker( () ->{

                    slides.level2Tip();
                })
                .waitSeconds(0.1)
                .addTemporalMarker( ()->{
                    slides.rest();
                })
                .build();

        TrajectorySequence cycles = drive.trajectorySequenceBuilder(preLoadPlace.end())

                .lineToLinearHeading(WAREHOUSE_0)
                .waitSeconds(0.15)

                .addTemporalMarker(() ->{
                    slides.extendLevel3();
                })
                .addTemporalMarker(() ->{
                    slides.level3Temp();
                })
                .lineToLinearHeading(SCORE_0)
                .addTemporalMarker( () ->{

                    slides.level3Tip();
                })
                .waitSeconds(0.1)
                .addTemporalMarker( ()->{
                    slides.rest();
                })

                .lineToLinearHeading(WAREHOUSE_1)
                .waitSeconds(0.15)

                .addTemporalMarker(() ->{
                    slides.extendLevel3();
                })
                .addTemporalMarker(() ->{
                    slides.level3Temp();
                })
                .lineToLinearHeading(SCORE_1)
                .addTemporalMarker(() ->{

                    slides.level3Tip();
                })
                .waitSeconds(0.1)
                .addTemporalMarker( ()->{
                    slides.rest();
                })

                .lineToLinearHeading(WAREHOUSE_2)
                .waitSeconds(0.15)

                .addTemporalMarker(() ->{
                    slides.extendLevel3();
                })
                .addTemporalMarker(() ->{
                    slides.level3Temp();
                })
                .lineToLinearHeading(SCORE_2)
                .addTemporalMarker( () ->{

                    slides.level3Tip();
                })
                .waitSeconds(0.1)
                .addTemporalMarker( ()->{
                    slides.rest();
                })

                .lineToLinearHeading(WAREHOUSE_3)
                .waitSeconds(0.15)

                .addTemporalMarker(() ->{
                    slides.extendLevel3();
                })
                .addTemporalMarker(() ->{
                    slides.level3Temp();
                })
                .lineToLinearHeading(SCORE_3)
                .addTemporalMarker( () ->{

                    slides.level3Tip();
                })
                .waitSeconds(0.1)
                .addTemporalMarker( ()->{
                    slides.rest();
                })

                .lineToLinearHeading(PARK)
                .build();


        slides.rest();
        drive.setPoseEstimate(startPose);

        // WEBCAM STUFF
        Webcam webcam = new Webcam(this);
        webcam.init(hardwareMap);

        waitForStart();

        drive.followTrajectorySequenceAsync(preLoadPlace);
        while(opModeIsActive()){
            drive.update();
            slides.update();
            acquirer.autonLoop();

            switch (state){
                case PRELOAD_PLACE:

                    if(!drive.isBusy()){
                        state = AutonState.CYCLES;
                        drive.followTrajectorySequenceAsync(cycles);
                    }
                    break;
                case CYCLES:
                    if(!drive.isBusy()){
                        state = AutonState.IDLE;
                    }
                    break;
                case IDLE:
                    break;
            }


        }

    }
}
