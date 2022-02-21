package org.firstinspires.ftc.teamcode.hardware.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class FreightSensor extends Mechanism {
    ColorSensor sensor;

    public FreightSensor(LinearOpMode opMode){
        this.opMode = opMode;
    }

    @Override
    public void init(HardwareMap hwMap) {
        sensor = hwMap.get(ColorSensor.class, "colorSensor");
    }

    public boolean hasFreightSensor(ColorSensor color) {
        return ((color.red() + color.green()) / 2 > color.blue() && (color.red() + color.green()) / 2 >= 100); //idk how this works mannn
    }

    public boolean hasFreight() {
        return hasFreightSensor(sensor);
    }
}
