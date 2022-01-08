package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Subsystem;
import org.firstinspires.ftc.teamcode.Configuration;

import android.util.Log;

public class Intake implements Subsystem {
    //Hardware: 1 motor, 1 encoder
    final private DcMotorEx armMotor;
    final private DcMotorEx sweepMotor;
    final private ColorSensor colorSensor;
    private boolean prevFreightState;
    private boolean freightState;
    private boolean isFreightIn;
    private static final double TICKS_PER_REV = 4592; // 28 * 164=4592

    //PID Stuff
    final private PIDFController armPID;
    private static final PIDCoefficients ARM_PID_COEFFICIENTS = new PIDCoefficients(1, 0, 0);

    private static final double ARM_ACCEPTABLE_ERROR_MARGIN = 0.05;

    public enum Positions {
        RESET,
        INTAKE,
        DUMP,
        LIFT,
    }

    public Intake(Robot robot) {
        armMotor = robot.getMotor("armMotor");
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        colorSensor =robot.getColorSensor("color");

        armPID = new PIDFController(ARM_PID_COEFFICIENTS);
        //In order for the PID controller to find the most efficient way to go to the target position,
        //we need to bound the error inputted to the PID controller from -pi to pi radians
        armPID.setInputBounds(-Math.PI, Math.PI);

        sweepMotor = robot.getMotor("sweeper");

    }

    public void setTargetAngle(double targetAngle) {
        armPID.reset();
        armPID.setTargetPosition(targetAngle);
        Log.i("intake", "angle set to "+targetAngle);
    }
    public void start() {
        setTargetPosition(Positions.INTAKE);
        sweepMotor.setPower(-0.8);
        Log.i("intake", targetReached() +
                " " + getArmAngle() + " " + armMotor.getCurrentPosition());
    }
    public void stop() {
        setTargetPosition(Positions.DUMP);
        sweepMotor.setPower(-0.8);
    }
    public void reset() {
        sweepMotor.setPower(0.0);
        //setTargetPosition(Positions.RESET);
    }

    public void setTargetPosition(Positions position) {
        switch (position) {
            case RESET:
                setTargetAngle(Configuration.INTAKE_ANGLE_RESET * Math.PI / 180); //0
                break;
            case INTAKE:
                setTargetAngle(Configuration.INTAKE_ANGLE_DOWN * Math.PI / 180);//-80 deg -> radians
                break;
            case DUMP:
                setTargetAngle(Configuration.INTAKE_ANGLE_DUMP * Math.PI / 180);//-10 deg
                break;
            case LIFT:
                setTargetAngle(Configuration.INTAKE_ANGLE_LIFT * Math.PI / 180); //-5 deg
                break;
        }
    }

    public boolean checkFreightIn(){
        return isFreightIn;
    }

    public void updateColorSensor() {
        freightState = colorSensor.alpha() > 500;
        isFreightIn = freightState && !prevFreightState;
        prevFreightState = freightState;
    }

    private double getRawArmAngle() {
        // encoder position * (2pi / TICKS_PER_REV)
        return armMotor.getCurrentPosition() * (2 * Math.PI / TICKS_PER_REV);
    }

    public double getArmAngle() {
        return Angle.norm(getRawArmAngle());
    }

    public double getPIDError() { return armPID.getLastError(); }

    public boolean targetReached() {
        return Math.abs(getPIDError()) <= ARM_ACCEPTABLE_ERROR_MARGIN;
    }

    @Override
    public void update(TelemetryPacket packet) {
        double armPower = armPID.update(getArmAngle());
        armMotor.setPower(armPower);
        updateColorSensor();
    }
}
