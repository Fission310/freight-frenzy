package org.firstinspires.ftc.teamcode.hardware.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.stuyfission.fissionlib.util.Mechanism;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.slides.Carriage;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.slides.SlidesProfile;

/**
 * FSM for slides scoring
 */
@Config
public class Slides extends Mechanism {

    SlidesProfile slides;
    Carriage carriage;

    public Slides(LinearOpMode opMode) { this.opMode = opMode; }

    @Override
    public void init(HardwareMap hwMap) {
        slides.init(hwMap);
        carriage.init(hwMap);
    }

    @Override
    public void loop(Gamepad gamepad) {
        // TODO: FSM

        // temporary:
        slides.loop(gamepad);
        carriage.loop(gamepad);
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        slides.telemetry(telemetry);
        carriage.telemetry(telemetry);
    }
}
