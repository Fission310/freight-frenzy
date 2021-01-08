package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Flywheel;
import org.firstinspires.ftc.teamcode.hardware.Leg;

@TeleOp(name="LegTest", group="Test")
public class ComponentTest extends LinearOpMode{

    private Leg leg = new Leg(this);
    private Flywheel flywheel = new Flywheel(this);

    @Override
    public void runOpMode() throws InterruptedException{

        leg.init(hardwareMap);

        waitForStart();

        while(opModeIsActive()){

            if(gamepad1.a) {
                leg.moveB();
                flywheel.rotate();
            }
            else if(gamepad1.b) leg.moveF();
            else if(gamepad1.x) leg.swing();
            else if(gamepad1.y) leg.reset();

        }

    }
}
