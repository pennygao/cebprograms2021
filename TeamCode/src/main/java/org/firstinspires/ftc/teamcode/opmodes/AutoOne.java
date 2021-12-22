package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

//A testing ground for basic autonomous code

@Autonomous
public class AutoOne extends LinearOpMode {
    //Declare hardware
    DcMotor DriveLF = null;

    @Override
    public void runOpMode() throws InterruptedException {
        //Initialize hardware
        DriveLF = hardwareMap.dcMotor.get("DriveLF");
        DriveLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DriveLF.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        SpinSafe(0.2, 10000, 3, 3, 3);
    }

    //Methods
    public void SpinTime(double power, long time) throws InterruptedException {
        DriveLF.setPower(power);
        Thread.sleep(time);
        Stop();
    }
    public void Stop() {
        DriveLF.setPower(0);
    }

    public void SpinSafe(double power, long time, long interval, long threshold, long repeat) throws InterruptedException {
        long old = DriveLF.getCurrentPosition();
        long count = 0;
        DriveLF.setPower(power);
        for (long i=0; i<time; i+=interval) {
            Thread.sleep(interval);
            long cur = DriveLF.getCurrentPosition();
            long delta = cur - old;
            old = cur;
            log("i=" + i + " cur=" + cur + " (" + delta + ")" );
            if (delta < threshold && i > 100) {
                count++;
                log("too slow, #" + count);
                if (count >= repeat) {
                    log("too slow to continue, stop!");
                    break;
                }
            } else {
                count = 0;
            }
        }
        Stop();
    }

    public void log(String msg) {
        Log.i("Auto One", msg);
    }
}