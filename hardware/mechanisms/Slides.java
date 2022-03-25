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
        LEVEL3_TEMP,
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
                if (gamepad.y) {
                    state = SlidesState.LEVEL3_TEMP;
                    time.reset();
                }
                break;
            case LEVEL3_TEMP:
                if (time.seconds() > 0.5)
                    slides.CARRIAGElevel3Temp();
                slides.level3Temp();
                if (gamepad.x) {
                    state = SlidesState.LEVEL3_TIP;
                }
                break;
            case LEVEL3_TIP:
                slides.level3Tip();
                if (gamepad.a) {
                    state = SlidesState.REST;
                }
                break;
            default:
                slides.rest();
                break;
        }
        slides.update();
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("case", state);
        slides.telemetry(telemetry);
    }
}
