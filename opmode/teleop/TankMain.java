package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.TankRobot;

@TeleOp(name = "TankMain", group = "Test")
public class TankMain extends LinearOpMode {

    private TankRobot robot = new TankRobot(this);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            robot.loop(gamepad1);
        }
    }
}
