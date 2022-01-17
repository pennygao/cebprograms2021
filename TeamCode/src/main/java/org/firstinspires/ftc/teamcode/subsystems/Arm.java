package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Subsystem;


public class Arm implements Subsystem {
    //Hardware: 1 motor, 1 encoder
    private Servo armServo;
    private Servo clawServo;
    private double servoclawPosition = 0;
    private double servoarmPosition = 0;

    public Arm(Robot robot) {
        armServo = robot.getServo("armServo");
        clawServo = robot.getServo("clawServo");

    }

    public double getarmPosition() {
        return armServo.getPosition();
    }
    public double getclawPosition() {
        return clawServo.getPosition();
    }

    public double getArmTargetPosition() {
        return servoarmPosition;
    }
    public double getClawTargetPosition() {
        return servoclawPosition;
    }

    public void setClawPosition(double position) {
        this.servoclawPosition = position;
        // set encode to new position
    }
    public void changeClawDirection(){
        Servo.Direction direction = Servo.Direction.REVERSE;
        clawServo.setDirection(direction);


    }
    public void setArmPosition(double position) {
        this.servoarmPosition = position;
    }
    public void changeArmDirection(){
        Servo.Direction direction = Servo.Direction.REVERSE;
        armServo.setDirection(direction);
    }


    @Override
    public void update(TelemetryPacket packet) {
        clawServo.setPosition(servoclawPosition);
                armServo.setPosition(servoarmPosition);
        packet.put("clawposition",servoarmPosition);
    }
}
