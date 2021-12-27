package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.robot.Robot;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.robot.Subsystem;

import android.util.*;

public class UserInput implements Subsystem {
    boolean debug = false;

    private Gamepad[] gamepads = new Gamepad [2] ;
    private boolean[] pressedGP1 = new boolean [8];
    private boolean[] pressedGP2 = new boolean [8];
    private boolean[] prevGP1 = new boolean[8];
    private boolean[] prevGP2 = new boolean[8];
    /*
    A 0
    B 1
    X 2
    Y 3
    Left bumper 4
    Right bumper 5
    */
    public UserInput (Gamepad gamepad1, Gamepad gamepad2) {
        gamepads[0] = gamepad1;
        gamepads[1] = gamepad2;

    }

    public int buttonNameToIndex(String buttonName) throws IllegalStateException{
        switch (buttonName){
            case "a": return 0;
            case "b": return 1;
            case "x": return 2;
            case "y": return 3;
            case "left_bumper": return 4;
            case "right_bumper": return 5;
            case "left_trigger": return 6;
            case "right_trigger": return 7;
            default: throw new IllegalStateException();
        }
    }

    public boolean buttonPressed (int gamepadNumber, String buttonName) throws IllegalStateException{
        if (gamepadNumber==1){
            return pressedGP1[buttonNameToIndex(buttonName)];
        }
        else if (gamepadNumber == 2) {
            return pressedGP2[buttonNameToIndex(buttonName)];
        }
        else{
            throw new IllegalStateException();
        }
    }

    public void updateGamepad(Gamepad gamepad, boolean[] pressed, boolean[] prev){
        boolean leftTrigger = gamepad.left_trigger > 0;
        boolean rightTrigger = gamepad.right_trigger > 0;

        pressed[0] = gamepad.a && !prev[0];
        pressed[1] = gamepad.b && !prev[1];
        pressed[2] = gamepad.x && !prev[2];
        pressed[3] = gamepad.y && !prev[3];
        pressed[4] = gamepad.left_bumper && !prev[4];
        pressed[5] = gamepad.right_bumper && !prev[5];
        pressed[6] = leftTrigger && !prev[6];
        pressed[7] = rightTrigger && !prev[7];
        prev[0] = gamepad.a;
        prev[1] = gamepad.b;
        prev[2] = gamepad.x;
        prev[3] = gamepad.y;
        prev[4] = gamepad.left_bumper;
        prev[5] = gamepad.right_bumper;
        prev[6] = leftTrigger;
        prev[7] = rightTrigger;
    }

    @Override
    public void update (TelemetryPacket packet) {

        if (debug && gamepads[0].a) {
            debug = false;
        }
        updateGamepad(gamepads[0], pressedGP1, prevGP1);
        updateGamepad(gamepads[1], pressedGP2, prevGP2);
    }
}
