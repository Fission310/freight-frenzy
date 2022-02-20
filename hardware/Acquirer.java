package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Acquirer extends Mechanism {
    private DcMotor pasta;

    public Acquirer(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    @Override
    public void init(HardwareMap hwMap) {
        pasta = hwMap.dcMotor.get("pasta");
        pasta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void loop(Gamepad gamepad1) {
        if (gamepad1.right_trigger > 0) { pasta.setPower(1); }
        else if (gamepad1.left_trigger > 0) { pasta.setPower(-1); }
        else { pasta.setPower(0); }
    }

}
