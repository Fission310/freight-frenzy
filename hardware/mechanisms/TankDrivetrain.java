package org.firstinspires.ftc.teamcode.hardware.mechanisms;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.stuyfission.fissionlib.util.Mechanism;

import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;

public class TankDrivetrain extends Mechanism {
//    public SampleTankDrive rrDrive;
    DcMotorEx leftFront;
    DcMotorEx leftRear;
    DcMotorEx rightRear;
    DcMotorEx rightFront;

    public TankDrivetrain(LinearOpMode opMode) { this.opMode = opMode; }

    @Override
    public void init(HardwareMap hwMap) {
//        rrDrive = new SampleTankDrive(hwMap);
        leftFront = hwMap.get(DcMotorEx.class, "hubRight");
        leftRear = hwMap.get(DcMotorEx.class, "hubLeft");
        rightRear = hwMap.get(DcMotorEx.class, "wallLeft");
        rightFront = hwMap.get(DcMotorEx.class, "wallRight");
    }

    public void loop(Gamepad gamepad) {
        if (-gamepad.left_stick_x < 0) {
            leftFront.setPower(-1);
            rightFront.setPower(-1);
            leftRear.setPower(- 1);
        } else {
            leftFront.setPower(0);
            leftRear.setPower(0);
            rightRear.setPower(0);
            rightFront.setPower(0);
        }

//        rrDrive.setWeightedDrivePower(
//                new Pose2d(
//                        -gamepad.left_stick_x,
//                        0,
//                        gamepad.right_stick_x
//                )
//        );
    }
}
