package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.stuyfission.fissionlib.util.Mechanism;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Acquirer;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.slides.Carriage;

public class TestRobot extends Mechanism {

    public TestRobot(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    private Acquirer acquirer = new Acquirer(opMode);
    private Carriage carriage = new Carriage(opMode);

    @Override
    public void init(HardwareMap hwMap) {

        acquirer.init(hwMap);
        carriage.init(hwMap);
    }

    @Override
    public void loop(Gamepad gamepad) {

        acquirer.loop(gamepad);
        carriage.loop(gamepad);
    }

    public void telemetry(Telemetry telemetry){
        acquirer.telemetry(telemetry);}
}
