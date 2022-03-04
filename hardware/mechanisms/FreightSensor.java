package org.firstinspires.ftc.teamcode.hardware.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@Config
public class FreightSensor extends Mechanism {
    ColorSensor colorLeft;
    ColorSensor colorRight;

    public static int YELLOW_THRESHOLD = 700;
    public static int WHITE_THRESHOLD = 500;

    public FreightSensor(LinearOpMode opMode){
        this.opMode = opMode;
    }

    @Override
    public void init(HardwareMap hwMap) {

        colorLeft = hwMap.get(ColorSensor.class, "colorLeft");
//        colorRight = hwMap.get(ColorSensor.class, "colorRight");
    }

    public boolean hasFreightSensor(ColorSensor color) {
        int yellow = (color.red() + color.green()) / 2;
        int white = (color.red() + color.green() + color.blue()) / 3;

        return ((yellow > color.blue() && yellow >= YELLOW_THRESHOLD) || white >= WHITE_THRESHOLD); //idk how this works mannn
    }

    public boolean hasFreightLeft() {

        return hasFreightSensor(colorLeft);
    }


    public void telemetry(Telemetry telemetry){

        int yellow = (colorLeft.red() + colorLeft.green()) / 2;
        int white = (colorLeft.red() + colorLeft.green() + colorLeft.blue()) / 3;

        telemetry.addData("yellow",yellow);

        telemetry.addData("white", white);

        telemetry.addData("red", colorLeft.red());
        telemetry.addData("blue", colorLeft.blue());
        telemetry.addData("green", colorLeft.green());
    }

//    public boolean hasFreightRight(){
//        return hasFreightSensor(colorRight);
//    }
}
