package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Flywheel;
import org.firstinspires.ftc.teamcode.hardware.Leg;

@TeleOp(name="CompTest", group="Test")
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
                flywheel.forward();
            }
            else if(gamepad1.b) {
                leg.moveF();
                flywheel.reverse();
            }
            else if(gamepad1.x) leg.swing();
            else if(gamepad1.y) leg.reset();

            telemetry.addData("LegPos", leg.servo.getPosition());
            telemetry.update();

        }

    }
}
