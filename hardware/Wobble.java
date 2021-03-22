package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Wobble extends Mechanism{

    public Servo clamp;
    public Servo rotator;
    private State state;

    public enum State{
        REST,
        UP,
        DOWN;

        private State up, down;

        static{
            REST.up = REST;
            REST.down = UP;

            UP.up = REST;
            UP.down = DOWN;

            DOWN.up = UP;
            DOWN.down = DOWN;
        }
    }

    public Wobble(LinearOpMode opMode){this.opMode = opMode;}

    public void init(HardwareMap hwMap){

        rotator = hwMap.servo.get("wobbleRotator");
        clamp = hwMap.servo.get("wobbleClamp");

        updateRotator(State.REST);
        open();
    }


    private void updateRotator(State nextState){
        this.state = nextState;
        if(state == State.DOWN){
            rotator.setPosition(0.85);
        }
        else if(state == State.UP){
            rotator.setPosition(0.51);
        }
        else if(state == State.REST){
            rotator.setPosition(0.31);
        }
    }

    public void rotateUp(){
        updateRotator(state.up);
    }

    public void rotateDown(){
        updateRotator(state.down);
    }

    public void close(){
        clamp.setPosition(0.0);
    }

    public void open(){
        clamp.setPosition(0.46);
    }



    public void moveF(){
        clamp.setPosition(clamp.getPosition() + 0.001);
    }

    public void moveB(){
        clamp.setPosition(clamp.getPosition() - 0.001);
    }


}
