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
        TIP_DELAY
    }
    SlidesState state;


    ElapsedTime time = new ElapsedTime();
    public static double LEVEL3_TEMP_WAIT = 0.2;
    public static double LEVEL3_TIP_WAIT = 0.25;

    @Override
    public void init(HardwareMap hwMap) {
        slides.init(hwMap);

        state = SlidesState.REST;
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

                    time.reset();
                    state = SlidesState.DELAY;
                }
                break;
            case DELAY:
                if (time.seconds() > LEVEL3_TEMP_WAIT) {
                    slides.armLevel3();

                    state = SlidesState.TIP;
                }
                break;
            case TIP:
                if (gamepad.x) {
                    slides.open();

                    time.reset();
                    state = SlidesState.TIP_DELAY;
                }
                break;
            case TIP_DELAY:
                if (time.seconds() > LEVEL3_TIP_WAIT) {
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
