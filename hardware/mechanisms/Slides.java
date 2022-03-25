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
        WAIT,
        REST,
        LEVEL3_EXTEND,
        LEVEL3_TIP
    }

    SlidesState state;
    ElapsedTime time = new ElapsedTime();

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
                    slides.level3Temp();

                    state = SlidesState.LEVEL3_EXTEND;
                    time.reset();
                }
                break;
            case LEVEL3_EXTEND:
                if (gamepad.x) {
                    slides.level3Tip();
                    state = SlidesState.LEVEL3_TIP;
                }
                break;
            case LEVEL3_TIP:
                if(gamepad.a){
                    state = SlidesState.REST;
                }
                break;
        }
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("case", state);
        slides.telemetry(telemetry);
    }
}
