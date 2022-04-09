package org.firstinspires.ftc.teamcode.opmode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Acquirer;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.slides.SlideMechanism;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
@Config
public class REDcycles6 extends LinearOpMode {

    private static final double WALL_POS = -70.5+(12.5/2.0);
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

    public static double LVL2_WAIT = 0.43;
    public static double LVL3_WAIT = 0.35;
    public static double SCORE_ANTI_WAIT = 0.1;
    public static double WAREHOUSE_WAIT = 0.15;

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

        TrajectorySequence cycles = drive.trajectorySequenceBuilder(startPose)

                // ================ CV ================ //
                .addTemporalMarker(slides::extendLevel2)
                .UNSTABLE_addDisplacementMarkerOffset(0.2, slides::level2ArmTemp)
                .UNSTABLE_addTemporalMarkerOffset(0.4, slides::level2CupTemp)

                .lineToLinearHeading(SCORE_0)
                .UNSTABLE_addDisplacementMarkerOffset(-0.2, slides::level2Tip)
                .UNSTABLE_addDisplacementMarkerOffset(-0.2 + LVL2_WAIT, slides::rest)
                // =========================================== //


                // ================ Cycle 1 ================ //
                .lineToLinearHeading(WAREHOUSE_0)
                .waitSeconds(WAREHOUSE_WAIT)

                .UNSTABLE_addTemporalMarkerOffset(0.5, () ->{
                    slides.extendLevel3();
                    slides.level3Temp();
                })
                .lineToLinearHeading(SCORE_0)
                .UNSTABLE_addTemporalMarkerOffset(-SCORE_ANTI_WAIT + 0.05, slides::level3Tip)
                .UNSTABLE_addTemporalMarkerOffset(-SCORE_ANTI_WAIT + LVL3_WAIT + 0.05, slides::rest)
                // =========================================== //

                // ================ Cycle 2 ================ //
                .lineToLinearHeading(WAREHOUSE_1)
                .waitSeconds(WAREHOUSE_WAIT)

                .UNSTABLE_addTemporalMarkerOffset(0.5, () ->{
                    slides.extendLevel3();
                    slides.level3Temp();
                })
                .lineToLinearHeading(SCORE_1)
                .UNSTABLE_addTemporalMarkerOffset(-SCORE_ANTI_WAIT, slides::level3Tip)
                .UNSTABLE_addTemporalMarkerOffset(-SCORE_ANTI_WAIT + LVL3_WAIT, slides::rest)
                // =========================================== //

                // ================ Cycle 3 ================ //
                .lineToLinearHeading(WAREHOUSE_2)
                .waitSeconds(WAREHOUSE_WAIT)

                .UNSTABLE_addTemporalMarkerOffset(0.5, () ->{
                    slides.extendLevel3();
                    slides.level3Temp();
                })
                .lineToLinearHeading(SCORE_2)
                .UNSTABLE_addTemporalMarkerOffset(-SCORE_ANTI_WAIT, slides::level3Tip)
                .UNSTABLE_addTemporalMarkerOffset(-SCORE_ANTI_WAIT + LVL3_WAIT, slides::rest)
                // =========================================== //

                // ================ Cycle 4 ================ //
                .lineToLinearHeading(WAREHOUSE_3)
                .waitSeconds(WAREHOUSE_WAIT)

                .UNSTABLE_addTemporalMarkerOffset(0.5, () ->{
                    slides.extendLevel3();
                    slides.level3Temp();
                })
                .lineToLinearHeading(SCORE_3)
                .UNSTABLE_addTemporalMarkerOffset(-SCORE_ANTI_WAIT, slides::level3Tip)
                .UNSTABLE_addTemporalMarkerOffset(-SCORE_ANTI_WAIT + LVL3_WAIT, slides::rest)
                // =========================================== //

                // ================ Cycle 5 ================ //
                .lineToLinearHeading(WAREHOUSE_4)
                .waitSeconds(WAREHOUSE_WAIT)

                .UNSTABLE_addTemporalMarkerOffset(0.5, () ->{
                    slides.extendLevel3();
                    slides.level3Temp();
                })
                .lineToLinearHeading(SCORE_4)
                .UNSTABLE_addTemporalMarkerOffset(-SCORE_ANTI_WAIT, slides::level3Tip)
                .UNSTABLE_addTemporalMarkerOffset(-SCORE_ANTI_WAIT + LVL3_WAIT, slides::rest)
                // =========================================== //

                // ================ Cycle 6 ================ //
                .lineToLinearHeading(WAREHOUSE_5)
                .waitSeconds(WAREHOUSE_WAIT)

                .UNSTABLE_addTemporalMarkerOffset(0.5, () ->{
                    slides.extendLevel3();
                    slides.level3Temp();
                })
                .lineToLinearHeading(SCORE_5)
                .UNSTABLE_addTemporalMarkerOffset(-SCORE_ANTI_WAIT, slides::level3Tip)
                .UNSTABLE_addTemporalMarkerOffset(-SCORE_ANTI_WAIT + LVL3_WAIT, slides::rest)
                // =========================================== //

                .lineToLinearHeading(PARK)
                .build();


        slides.rest();
        drive.setPoseEstimate(startPose);

        waitForStart();

        drive.followTrajectorySequenceAsync(cycles);
        while(opModeIsActive()){
            drive.update();
            slides.update();
            acquirer.autonLoop();
        }
    }
}
