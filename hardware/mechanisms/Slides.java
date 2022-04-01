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
        LEVEL2_TEMP_DELAY,
        EXTENDED,
        TIPPING,
    }
    SlidesState state;

    public enum LevelState {
        LEVEL1,
        LEVEL2,
        LEVEL3
    }
    LevelState level;

    ElapsedTime time = new ElapsedTime();
    public static double CARRIAGE2_DELAYTIME = 0.25;
    public static double CARRIAGE2_TEMP_DELAYTIME = 0.2;
    public static double CARRIAGE3_DELAYTIME = 0.4;
    public static double TIPPING2_DELAYTIME = 0.7;
    public static double TIPPING3_DELAYTIME = 0.5;

    @Override
    public void init(HardwareMap hwMap) {
        slides.init(hwMap);

        state = SlidesState.REST;
        level = LevelState.LEVEL1;
        time.reset();
    }

    @Override
    public void loop(Gamepad gamepad) {
        slides.update();
        switch(state) {
            case REST:
                slides.rest();
                state = SlidesState.WAIT;
                break;
            case WAIT:
                if (gamepad.y) {
                    level = LevelState.LEVEL3;
                    slides.extendLevel3();
                    state = SlidesState.DELAY;
                    time.reset();
                }
                if (gamepad.b) {
                    level = LevelState.LEVEL2;
                    slides.extendLevel2();
                    state = SlidesState.DELAY;
                    time.reset();
                }
                break;
            case DELAY:
                switch(level) {
                    case LEVEL1:
                        break;
                    case LEVEL2:
                        if (time.seconds() > CARRIAGE2_DELAYTIME) {
                            slides.level2ArmTemp();
                            time.reset();
                            state = SlidesState.LEVEL2_TEMP_DELAY;
                        }
                        break;
                    case LEVEL3:
                        if (time.seconds() > CARRIAGE3_DELAYTIME) {
                            slides.level3Temp();
                            state = SlidesState.EXTENDED;
                        }
                        break;
                }
                break;
            case LEVEL2_TEMP_DELAY:
                if (time.seconds() > CARRIAGE2_TEMP_DELAYTIME) {
                    slides.level2CupTemp();
                    state = SlidesState.EXTENDED;
                }
                break;
            case EXTENDED:
                if (gamepad.x) {
                    switch(level) {
                        case LEVEL1:
                            break;
                        case LEVEL2:
                            slides.level2Tip();
                            time.reset();
                            state = SlidesState.TIPPING;
                            break;
                        case LEVEL3:
                            slides.level3Tip();
                            time.reset();
                            state = SlidesState.TIPPING;
                            break;
                    }
                }
                break;
            case TIPPING:
                switch (level) {
                    case LEVEL1:
                        break;
                    case LEVEL2:
                        if (time.seconds() > TIPPING2_DELAYTIME) {
                            state = SlidesState.REST;
                        }
                        break;
                    case LEVEL3:
                        if (time.seconds() > TIPPING3_DELAYTIME) {
                            state = SlidesState.REST;
                        }
                        break;
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
