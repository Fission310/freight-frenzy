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

    public static int YELLOW_THRESHOLD = 350;
    public static int WHITE_THRESHOLD = 350;

    public FreightSensor(LinearOpMode opMode){
        this.opMode = opMode;
    }

    @Override
    public void init(HardwareMap hwMap) {

        colorLeft = hwMap.get(ColorSensor.class, "colorLeft");
        colorRight = hwMap.get(ColorSensor.class, "colorRight");
    }

    private int getWhite(ColorSensor color){
        return (color.red() + color.green() + color.blue()) / 3;
    }

    private int getYellow(ColorSensor color){
        return (color.red() + color.green()) / 2;
    }

    public boolean hasFreightSensor(ColorSensor color) {
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


    public void telemetry(Telemetry telemetry){

        telemetry.addData("yellowLeft",getYellow(colorLeft));
        telemetry.addData("whiteLeft", getWhite(colorLeft));

        telemetry.addData("yellowRight",getYellow(colorRight));
        telemetry.addData("whiteRight", getWhite(colorRight));


    }

}
