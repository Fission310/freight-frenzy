package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.stuyfission.fissionlib.util.Mechanism;

import org.firstinspires.ftc.teamcode.hardware.mechanisms.FlipServo;

public class FlipTest extends Mechanism {

    public FlipTest(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    private FlipServo grant = new FlipServo(opMode);

    @Override
    public void init(HardwareMap hwMap) {
        grant.init(hwMap);
    }

    @Override
    public void loop(Gamepad gamepad) {
        grant.loop(gamepad);
    }
}
