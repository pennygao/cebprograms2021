package org.firstinspires.ftc.teamcode.subsystems;
import android.util.Log;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robocol.TelemetryMessage;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.Configuration;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Subsystem;


public class ArmClaw<grabState> implements Subsystem {
    //Hardware: 1 motor, 1 encoder
    private Servo armServo;
    private Servo clawServo;
    private Servo armJoint;
    private double servoclawPosition = 0;
    private double servoarmPosition = 0;
    private double servoarmJointPosition = 0;
    private double armJointMaxPosition = 0.85;
    private double armMaxPosition = 0.85;
    private STATE grabState = STATE.INIT;
    private double servoTime;

    public enum Positions {
        RESET,
        READYGRABE,
        GRAB,
        READYCAP,
        CAP,
        HOLD,
    }

    public ArmClaw(Robot robot) {
        armServo = robot.getServo("servoArm");
        //clawServo = robot.getServo("s1");
        //armJoint = robot.getServo("s2");
        //armJoint.setDirection(Servo.Direction.REVERSE);
        //       servoTime = NanoClock.system().seconds();

    }
    public enum STATE  {INIT, INIT_WAIT, RESET, RESET_WAIT, IDLE, GRAB, GRAB_READY, GRAB_CLOSE, GRAB_HOLD,
        CAP_ARM_DOWN, CAP_CLAW_OPEN};
    /*   public void setTargetPosition(ArmClaw.Positions position) {
           switch (position) {
               case RESET:
                   setArmPosition(Configuration.ARM_RESET);
                   setArmJointPosition(Configuration.ARM_RESET);
                   setClawPosition(Configuration.CLAW_RESET);
                   //setTargetAngle(Configuration.INTAKE_ANGLE_RESET * Math.PI / 180); //0
                   break;
               case GRAB:
                   setArmPosition(Configuration.ARM_HOLD);//-80 deg -> radians
                   setClawPosition(Configuration.CLAW_OPEN);
                   setArmPosition(Configuration.ARM_GRAB);
                   setClawPosition(Configuration.CLAW_CLOSED);
                   setArmPosition(Configuration.ARM_HOLD);
                   break;
               /*case DUMP:
                   setTargetAngle(Configuration.INTAKE_ANGLE_DUMP * Math.PI / 180);//-10 deg
                   break;
               case LIFT:
                   setTargetAngle(Configuration.INTAKE_ANGLE_LIFT * Math.PI / 180); //-5 deg
                   break;
           }
           Log.i("intakePosition", "target position: " + position);
       }
   */
 /*   public void grab() {

        if(grabState == STATE.RESET ) {
            grabState = STATE.RESET_WAIT;
            servoTime = NanoClock.system().seconds();
            //changeArmJointDirection();
            //armJoint.setDirection(Servo.Direction.REVERSE);
            setArmJointPosition(Configuration.ARMJOINT_RESET);
            //armServo.setDirection(Servo.Direction.FORWARD);
            setArmPosition(Configuration.ARM_HOLD);
            //clawServo.setDirection();
            setClawPosition(Configuration.CLAW_RESET);
        }
        else if (grabState == STATE.IDLE) {
            grabState = STATE.GRAB;
            servoTime = NanoClock.system().seconds();
            //changeArmJointDirection();
            //armJoint.setDirection(Servo.Direction.FORWARD);
            setArmPosition(Configuration.ARM_GRAB);
            setArmJointPosition(Configuration.ARMJOINT_RESET);
            setClawPosition(Configuration.CLAW_OPEN);
        }
        else if (grabState == STATE.GRAB_READY) {
            grabState = STATE.GRAB_CLOSE;
            servoTime = NanoClock.system().seconds();
            //changeArmJointDirection();
            //armJoint.setDirection(Servo.Direction.FORWARD);
            //setArmPosition(Configuration.ARM_GRAB);
            //setArmJointPosition(Configuration.ARMJOINT_RESET);
            setClawPosition(Configuration.CLAW_CLOSED);
        }
        else if (grabState == STATE.GRAB_HOLD) {
            grabState = STATE.CAP_ARM_DOWN;
            servoTime = NanoClock.system().seconds();
            //changeArmJointDirection();
            //armJoint.setDirection(Servo.Direction.FORWARD);
            //setArmPosition(Configuration.ARM_GRAB);
            //setArmJointPosition(Configuration.ARMJOINT_RESET);
            setArmPosition(Configuration.ARM_CAP);
        }
    }*/


    public double getarmPosition() {
        return armServo.getPosition();
    }

    public void setArmTargetPosition(double position) {
        if (position > armMaxPosition)
            position = armMaxPosition;
        this.servoarmPosition = position;

    }
    public void changeArmDirection(){
        Servo.Direction direction = Servo.Direction.REVERSE;
        armServo.setDirection(direction);
    }

    @Override
   public void update(TelemetryPacket packet) {
       armServo.setPosition(servoarmPosition);
    }
 /*   public void update(TelemetryPacket packet) {

        switch (grabState){
            case INIT:
                grabState = STATE.INIT_WAIT;
                servoTime = NanoClock.system().seconds();
                setArmPosition(Configuration.ARM_RESET);
                setArmJointPosition(Configuration.ARMJOINT_RESET);
                setClawPosition(Configuration.CLAW_RESET);
                break;
            case INIT_WAIT:
                if (NanoClock.system().seconds() - servoTime >= 5) {
                    grabState = STATE.RESET;
                }
                break;
            case RESET:
                break;
            case RESET_WAIT: //reset
                if (NanoClock.system().seconds() - servoTime >= 5){
                    grabState = STATE.IDLE;
                    Log.i("servoTime", "1: " + servoTime
                            + " " + NanoClock.system().seconds());
                }
                else {

                    //      setArmPosition(Configuration.ARM_RESET);
                    //      setArmJointPosition(Configuration.ARM_RESET);
                    //      setClawPosition(Configuration.CLAW_RESET);

                    Log.i("grabState", "on 1: ");
                }
                break;
            case IDLE:
                break;
            case GRAB:
                if (NanoClock.system().seconds() - servoTime >= 5){
                    grabState = STATE.GRAB_READY;
                    Log.i("servoTime", "3: " + servoTime
                            + " " + NanoClock.system().seconds());
                }
                else {

                    //      setArmPosition(Configuration.ARM_RESET);
                    //      setArmJointPosition(Configuration.ARM_RESET);
                    //      setClawPosition(Configuration.CLAW_RESET);

                    Log.i("grabState", "on 3: ");
                }
                break;
            case GRAB_READY:
                break;
            case GRAB_CLOSE:
                if (NanoClock.system().seconds() - servoTime >= 5){
                    grabState = STATE.GRAB_HOLD;
                    servoTime = NanoClock.system().seconds();
                    //changeArmJointDirection();
                    //armJoint.setDirection(Servo.Direction.REVERSE);
                    //armJoint.setDirection(Servo.Direction.REVERSE);
                    //setArmJointPosition(Configuration.ARMJOINT_CAP);
                    armJoint.setPosition(0.35);

                    setArmPosition(Configuration.ARM_HOLD);
                    //clawServo.setDirection();
                    //setClawPosition(Configuration.CLAW_CLOSED);

                    Log.i("servoTime", "3: " + servoTime
                            + " " + NanoClock.system().seconds());
                }
                else {

                    //      setArmPosition(Configuration.ARM_RESET);
                    //      setArmJointPosition(Configuration.ARM_RESET);
                    //      setClawPosition(Configuration.CLAW_RESET);

                    Log.i("grabState", "on 3: ");
                }
                break;
            case GRAB_HOLD:
                break;
            case CAP_ARM_DOWN:
                if (NanoClock.system().seconds() - servoTime >= 5) {
                    grabState = STATE.CAP_CLAW_OPEN;
                    servoTime = NanoClock.system().seconds();
                    clawServo.setPosition(Configuration.CLAW_OPEN);
                }
                break;
            case CAP_CLAW_OPEN:
                if (NanoClock.system().seconds() - servoTime >= 5) {
                    grabState = STATE.INIT_WAIT;
                    servoTime = NanoClock.system().seconds();
                    //changeArmJointDirection();
                    //armJoint.setDirection(Servo.Direction.REVERSE);
                    setArmJointPosition(Configuration.ARMJOINT_RESET);
                    //armServo.setDirection(Servo.Direction.FORWARD);
                    setArmPosition(Configuration.ARM_RESET);
                    //clawServo.setDirection();
                    setClawPosition(Configuration.CLAW_RESET);
                }
                break;
/*
            case 2: //raise arm
                if (NanoClock.system().seconds() - servoTime >= 5){
                    Log.i("grabState", "ending 2 " +
                            armServo.getPosition() );
                            grabState++;
                    Log.i("servoTime", "servo 2:"+NanoClock.system().seconds());
                    servoTime = NanoClock.system().seconds();
                }
                else {
                    setArmPosition(Configuration.ARM_HOLD);
                    Log.i("grabState", "on 2: ");
                }
                break;
            case 3:

                Log.i("grabState", "on 3: ");

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
