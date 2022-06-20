package org.firstinspires.ftc.teamcode.hardware.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.stuyfission.fissionlib.util.Mechanism;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.slides.SlideMechanism;

/**
 * FSM for slides scoring
 */
@Config
public class Slides extends Mechanism {

    SlideMechanism slides = new SlideMechanism(opMode);

    public Slides(LinearOpMode opMode) { this.opMode = opMode; }

    public enum SlidesState {
        REST,
        WAIT,
        DELAY,
        TIP,
        TEMP_RETRACT,
        TEMP_CARRIAGE,
        TIP_DELAY,
        CAP_RESET
    }
    SlidesState state;

    public enum Level {
        LEVEL1,
        LEVEL2,
        LEVEL3
    }
    Level level;


    ElapsedTime time = new ElapsedTime();
    public static double LEVEL3_TEMP_WAIT = 0.2;
    public static double LEVEL3_TIP_WAIT = 0.25;

    public static double LEVEL2_ARM_TEMP_WAIT = 0.3;
    public static double LEVEL2_TEMP_WAIT = 0.9;
    public static double LEVEL2_TIP_WAIT = 0.5;

    public static double LEVEL1_ARM_TEMP_WAIT = 0.3;
    public static double LEVEL1_TEMP_WAIT = 1;
    public static double LEVEL1_TIP_WAIT = 1;

    public static double TEMP_RETRACT_WAIT = 0.2;
    public static double TEMP_CARRIAGE_WAIT = 0.4;

    @Override
    public void init(HardwareMap hwMap) {
        slides.init(hwMap);

        state = SlidesState.REST;
        level = Level.LEVEL1;
        time.reset();
    }

    @Override
    public void loop(Gamepad gamepad) {
        slides.update();
        switch (state) {
            case REST:
                slides.rest();
                state = SlidesState.WAIT;
                break;

        // wait for input
            case WAIT:
                // if clamp has freight
                if (slides.hasFreight()) {
                    slides.close();
                }
                if (gamepad.y) {
                    slides.extendLevel3();
                    slides.close();

                    level = Level.LEVEL3;

                    time.reset();
                    state = SlidesState.DELAY;
                }
                if (gamepad.x) {
                    slides.extendLevel2TEMP();
                    slides.close();

                    level = Level.LEVEL2;

                    time.reset();
                    state = SlidesState.DELAY;
                }
                if (gamepad.a) {
                    slides.extendLevel1TEMP();
                    slides.close();

                    level = Level.LEVEL1;

                    time.reset();
                    state = SlidesState.DELAY;
                }
                if (gamepad.b) {
                    slides.extendCapping();

                    state = SlidesState.CAP_RESET;
                }
                break;
            case DELAY:
                switch (level) {
                    case LEVEL1:
                        if (time.seconds() > LEVEL1_ARM_TEMP_WAIT) {
                            slides.armLevel1();
                        }
                        if (time.seconds() > LEVEL1_TEMP_WAIT) {
                            slides.extendLevel1();

                            state = SlidesState.TIP;
                        }
                        break;
                    case LEVEL2:
                        if (time.seconds() > LEVEL1_ARM_TEMP_WAIT) {
                            slides.armLevel2();
                        }
                        if (time.seconds() > LEVEL2_TEMP_WAIT) {
                            slides.extendLevel2();

                            state = SlidesState.TIP;
                        }
                        break;
                    case LEVEL3:
                        if (time.seconds() > LEVEL3_TEMP_WAIT) {
                            slides.armLevel3();

                            state = SlidesState.TIP;
                        }
                        break;
                }
                break;
            case TIP:
                if (gamepad.x) {
                    slides.open();

                    time.reset();

                    state = SlidesState.TEMP_RETRACT;
                }
                break;
            case TEMP_RETRACT:
                if (time.seconds() > TEMP_RETRACT_WAIT) {
                    time.reset();
                    switch (level) {
                        case LEVEL1:
                        case LEVEL2:
                            slides.restTEMP();
                            state = SlidesState.TEMP_CARRIAGE;
                            time.reset();
                            break;
                        case LEVEL3:
                            state = SlidesState.TIP_DELAY;
                            time.reset();
                            break;
                    }
                }
                break;
            case TEMP_CARRIAGE:
                if (time.seconds() > TEMP_CARRIAGE_WAIT) {
                    slides.restCarriage();
                    time.reset();
                    state = SlidesState.TIP_DELAY;
                }
                break;
            case TIP_DELAY:
                switch (level) {
                    case LEVEL1:
//                        slides.restCarriage();
                        if (time.seconds() > LEVEL1_TIP_WAIT) {
                            time.reset();
                            state = SlidesState.REST;
                        }
                        break;
                    case LEVEL2:
//                        slides.restCarriage();
                        if (time.seconds() > LEVEL2_TIP_WAIT) {
                            time.reset();
                            state = SlidesState.REST;
                        }
                        break;
                    case LEVEL3:
                        if (time.seconds() > LEVEL3_TIP_WAIT) {
                            time.reset();
                            state = SlidesState.REST;
                        }
                        break;
                }
                break;
            case CAP_RESET:
                if (gamepad.b) {
                    time.reset();
                    state = SlidesState.REST;
                }
                break;
        }
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("case", state);
        telemetry.addData("timer", time.seconds());
        slides.telemetry(telemetry);
    }
}
