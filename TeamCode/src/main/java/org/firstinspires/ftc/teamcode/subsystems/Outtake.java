package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Subsystem;
import org.firstinspires.ftc.teamcode.Configuration;

import android.graphics.Bitmap;
import android.util.Log;

public class Outtake implements Subsystem {
    //Level LED Definition
    //Green = first level, red = second level, orange = third level
    private LED LevelLEDRed;
    private LED LevelLEDGreen;
    //Hardware: 1 motor, 1 encoder
    private DcMotorEx slideMotor;
    private double slidePower= 0;
    public static final double  TICKS_PER_REV = 537.7;
    public static final double PULLEY_DIAMETER = 38 /25.4;
    public int level = 0;
    public int defaultLevel = 3;
    public static double SLIDE_LENGTH = 15.0;
    private static final double INCHES_PER_LEVEL = 3.5;
    private int targetPosition = 0;
    private Telemetry telemetry;
    private Servo dumpServo;
    //private double dumpInitPos = 0.8; //0.8;
    private int dumpState = 0;

    private double servoTime;
    private double dumpTime = 1.0; //second
    private double dumpRstTime = 0.5; //second
    private double dumpPos = 0.4;     //0.5                          ;
    private double dumpInitPos = 0.8; //0.8; //0.9
    NanoClock clock;
    private double dumpUpPos = 0.7;


    public enum slide_state {
        LEVEL_0,
        LEVEL_1,
        LEVEL_2,
    }

    public Outtake(Robot robot, Telemetry telemetry) {
        this.telemetry = telemetry;
        dumpServo = robot.getServo("dumpServo");
        slideMotor = robot.getMotor("slide");
        LevelLEDGreen = robot.getLED("green1");
        LevelLEDRed = robot.getLED("red1");
        slideMotor.setDirection(DcMotor.Direction.REVERSE);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidePower = 1;
        targetPosition = 0;

    }

    public double getDumpPosition() {
        return dumpServo.getPosition();
    }

    public void setServoPosition(double position) {
        this.dumpInitPos = position;
        // set encode to new position
    }

    public void setPower (double power ){
        slidePower = power;
    }

    public  int inchToTicks ( double inches) {
        return (int) (inches * TICKS_PER_REV / (PULLEY_DIAMETER * Math.PI));
    }

    public int getLevel() {
        return level;
    }
    /*
    0: idle
    1: slider go up
    2: servo dump
    3: servo undump
    4: go down
    */
    public void nextDefault() {
        if (defaultLevel==3){
            defaultLevel=1;
            LevelLEDRed.enableLight(true);
            LevelLEDGreen.enableLight(false);}
        else if(defaultLevel==1){
            defaultLevel++;
            LevelLEDRed.enableLight(false);
            LevelLEDGreen.enableLight(true);}
        else if(defaultLevel==2){
            defaultLevel++;
            LevelLEDRed.enableLight(true);
            LevelLEDGreen.enableLight(true);
        }
    }


    public void setDefault(int level) {
        defaultLevel = level;
    }

    public void dump() {
        if (dumpState == 0){ //go up
            switch (defaultLevel) {
                case 1:
                    targetPosition = inchToTicks(Configuration.SLIDER_FIRST_HEIGHT_INCHES); //3.0 inches
                    LevelLEDRed.enableLight(true);
                    LevelLEDGreen.enableLight(false);
                    break;
                case 2:
                    targetPosition = inchToTicks(Configuration.SLIDER_SECOND_HEIGHT_INCHES); //7.0
                    LevelLEDRed.enableLight(false);
                    LevelLEDGreen.enableLight(true);
                    break;
                case 3:
                    targetPosition = inchToTicks(Configuration.SLIDER_THIRD_HEIGHT_INCHES); //11.0
                    LevelLEDRed.enableLight(true);
                    LevelLEDGreen.enableLight(true);
                }
            dumpState++;
        }
    }


    public void goDown() {
        if (level > 0 && !slideMotor.isBusy()) { // slide_state.LEVEL_0) {
            level = level - 1;
            targetPosition = inchToTicks(INCHES_PER_LEVEL * level);
        }
    }

    public void goAllDown() {
        if (level > 0 && !slideMotor.isBusy()) { // slide_state.LEVEL_0) {
            level = 0;
            targetPosition = 0;//inchToTicks(INCHES_PER_LEVEL * level);
        }
    }

    public boolean dumpDone() {
        return (dumpState == 0 || dumpState >= 3);
    }

    @Override
    public void update(TelemetryPacket packet) {
        switch (dumpState){
            case 0:
                dumpServo.setPosition(dumpInitPos);
                break;
            case 1: //moving slide up
                if (Math.abs(slideMotor.getCurrentPosition()-targetPosition) <= Configuration.SLIDER_ACCEPTABLE_ERROR_TICKS){ //it should be around 1/8 of an inch
                    Log.i("dumpState", "ending 1: "
                            + slideMotor.getCurrentPosition() + " " + targetPosition);
                    dumpServo.setPosition(dumpPos);
                    dumpState++;
                    servoTime = NanoClock.system().seconds();
                    Log.i("servoTime", "1: " + servoTime
                            + " " + NanoClock.system().seconds());
                }
                else if (slidePower != 0) {
                    slideMotor.setPower(slidePower);
                    slideMotor.setTargetPosition(targetPosition);
                    slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    dumpServo.setPosition(dumpUpPos);
                    Log.i("dumpState", "on 1: ");
                }
                break;
            case 2: //dumping
                if (NanoClock.system().seconds() - servoTime >= dumpTime){// FIXME Yujie
                    Log.i("dumpState", "ending 2 " +
                            dumpServo.getPosition() + " " + dumpInitPos);
                    dumpServo.setPosition(dumpInitPos);
                    dumpState++;
                    Log.i("servoTime", "servo 2:"+NanoClock.system().seconds());
                    servoTime = NanoClock.system().seconds();
                }
                else {
                    dumpServo.setPosition(dumpPos);
                    Log.i("dumpState", "on 2: ");
                }
                break;

            case 3: // dump servo reset
                if (NanoClock.system().seconds() - servoTime >= dumpRstTime){
                    Log.i("dumpState", "ending 3 " +
                            dumpServo.getPosition() + " " + dumpInitPos);
                    targetPosition = 0;
                    dumpState++;
                    //dumpServo.setPosition(dumpInitPos);
                    Log.i("servoTime", "servo 3:"+NanoClock.system().seconds());
                    servoTime = NanoClock.system().seconds();
                }
                else {
                    dumpServo.setPosition(dumpInitPos);
                }
                Log.i("dumpState", "on 2: ");
                break;


            case 4: // slide going down
                if (Math.abs(slideMotor.getCurrentPosition()-targetPosition) <= 1){ //what do i do here?
                    Log.i("dumpState", "ending 4");
                    dumpState=0;
                }
                else if (slidePower != 0) {
                    slideMotor.setPower(slidePower);
                    slideMotor.setTargetPosition(targetPosition);
                    slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Log.i("dumpState", "on 3");
                }
                break;
        }

        /*if (level == 0 &&  !slideMotor.isBusy()) {
            slidePower = 0;
        }
         //
        dumpServo.setPosition(servoPosition);
        if (slidePower != 0) {
            slideMotor.setPower(slidePower);
            slideMotor.setTargetPosition(targetPosition);
            slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        */
       //see if the current dumpState is done

       //if  dumpState is done, do the next one.

        // debug only,  remove it on release
        // packet.put("Current Position", slideMotor.getCurrentPosition());
       //  packet.put("target position", slideMotor.getTargetPosition());
    }
}

