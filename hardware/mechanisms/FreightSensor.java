package org.firstinspires.ftc.teamcode.hardware.mechanisms;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.stuyfission.fissionlib.util.Mechanism;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class FreightSensor extends Mechanism {
    ColorRangeSensor colorLeft;
    ColorRangeSensor colorRight;

    public static int YELLOW_THRESHOLD = 200;
    public static int WHITE_THRESHOLD = 200;

    public FreightSensor(LinearOpMode opMode){
        this.opMode = opMode;
    }

    @Override
    public void init(HardwareMap hwMap) {

        colorLeft = hwMap.get(ColorRangeSensor.class, "colorLeft");
        colorRight = hwMap.get(ColorRangeSensor.class, "colorRight");

    }

    private int getWhite(ColorRangeSensor color) {
        return (color.red() + color.green() + color.blue()) / 3;
    }

    private int getYellow(ColorRangeSensor color) {
        return (color.red() + color.green()) / 2;
    }

    private double getDistance(ColorRangeSensor color) {
        return color.getDistance(DistanceUnit.CM);
    }

    private double getLight(ColorRangeSensor color) {
        return color.getLightDetected();
    }

    public boolean hasFreightSensor(ColorRangeSensor color) {
        int yellow = getYellow(color);
        int white = getWhite(color);

        return ((yellow > color.blue() && yellow >= YELLOW_THRESHOLD) || white >= WHITE_THRESHOLD);
    }

    public boolean hasFreightLeft() {

        return hasFreightSensor(colorLeft);
    }

    public boolean hasFreightRight() {

        return hasFreightSensor(colorRight);
    }

    public boolean hasFreight(){
        return hasFreightLeft() || hasFreightRight();
    }


    @Override
    public void telemetry(Telemetry telemetry){

        telemetry.addData("yellowLeft",getYellow(colorLeft));
        telemetry.addData("whiteLeft", getWhite(colorLeft));

        telemetry.addData("yellowRight",getYellow(colorRight));
        telemetry.addData("whiteRight", getWhite(colorRight));

//        telemetry.addData("distanceLeft", getDistance(colorLeft));
//        telemetry.addData("lightLeft", getLight(colorLeft));

//        telemetry.addData("distanceRight", getDistance(colorRight));
//        telemetry.addData("lightRight", getLight(colorRight));

    }

}
