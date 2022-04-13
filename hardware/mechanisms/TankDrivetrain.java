package org.firstinspires.ftc.teamcode.hardware.mechanisms;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.stuyfission.fissionlib.util.Mechanism;

import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;

public class TankDrivetrain extends Mechanism {
    public SampleTankDrive rrDrive;

    public TankDrivetrain(LinearOpMode opMode) { this.opMode = opMode; }

    @Override
    public void init(HardwareMap hwMap) {
        rrDrive = new SampleTankDrive(hwMap);
    }

    public void loop(Gamepad gamepad) {
        rrDrive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad.right_stick_x,
                        0,
                        gamepad.left_stick_x
                )
        );
    }
}
