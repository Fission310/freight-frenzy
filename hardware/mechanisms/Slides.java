package org.firstinspires.ftc.teamcode.hardware.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.stuyfission.fissionlib.util.Mechanism;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.slides.Carriage;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.slides.SlidesProfile;

/**
 * FSM for slides scoring
 */
@Config
public class Slides extends Mechanism {

    SlidesProfile slides = new SlidesProfile(opMode);

    public Slides(LinearOpMode opMode) { this.opMode = opMode; }

    public enum SlidesState {
        REST,
        WAIT,
        CARRIAGE_DELAY,
        LEVEL3_EXTENDED,
        LEVEL3_TIPPING,
    }

    SlidesState state;
    ElapsedTime time = new ElapsedTime();
    public static double CARRIAGE_DELAYTIME = 0.25;
    public static double LEVEL3_DELAYTIME = 0.5;

    @Override
    public void init(HardwareMap hwMap) {
        slides.init(hwMap);

        state = SlidesState.REST;
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
                    slides.extendLevel3();
                    state = SlidesState.CARRIAGE_DELAY;
                    time.reset();
                }
                break;
            case CARRIAGE_DELAY:
                if (time.seconds() > CARRIAGE_DELAYTIME) {
                    slides.level3Temp();
                    state = SlidesState.LEVEL3_EXTENDED;
                }
                break;
            case LEVEL3_EXTENDED:
                if (gamepad.x) {
                    slides.level3Tip();
                    time.reset();
                    state = SlidesState.LEVEL3_TIPPING;
                }
                break;
            case LEVEL3_TIPPING:
                if (time.seconds() > LEVEL3_DELAYTIME) {
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
