package org.firstinspires.ftc.teamcode.subsystems;
import android.util.Log;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Subsystem;


public class duckSpinner implements Subsystem {
    //Hardware: 1 motor, 1 encoder
    private DcMotorEx duckSpinner;
    private double motorPosition = 0;
    private double prevTime = 0;
    private double prevPower = 0.1;

    public duckSpinner(Robot robot) {
        duckSpinner = robot.getMotor("duck");
    }

    public void setPower(double power) {
        double currentTime = System.nanoTime();
        double timeDiff = currentTime - prevTime;
        double powerIncrease = timeDiff * 0.0000000007;
        double maxPower = powerIncrease + Math.abs(prevPower);
        double sign = power/Math.abs(power);
        double newPower = Math.min(maxPower, Math.abs(power));
        Log.i("duckSpinner", "power:"+ newPower + " previous: "+ prevPower +
                " increased: "+ powerIncrease + " sign: "+ sign);
        //update values
        prevTime = currentTime;
        prevPower = newPower; //newPower
        //set power
        this.motorPosition = newPower * sign; //newPower
        /*0
        double newPower = Math.abs(prevPower);
        newPower += 0.00000005*(System.nanoTime()-prevTime);
        prevTime = System.nanoTime();
        double spinPower = Math.min(newPower, Math.abs(power)) * (power/Math.abs(power));
         */

        // set encode to new position
    }


    @Override
    public void update(TelemetryPacket packet) {duckSpinner.setPower(motorPosition);}
    }


