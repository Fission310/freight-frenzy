package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.FlipTest;

@TeleOp(name = "GrantTest", group = "drive")
public class GrantTest extends LinearOpMode {

    private FlipTest robot = new FlipTest(this);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            robot.loop(gamepad1);
        }
    }
}
