package org.firstinspires.ftc.teamcode.opmodes.testing;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;
import org.firstinspires.ftc.teamcode.subsystems.UserInput;

import android.util.Log;
@TeleOp
public class  ReadColorSensor  extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CrabRobot robot = new CrabRobot(this);
        waitForStart();

        boolean isFreightIn = false;
        int intakeMode = 0;

//ENTER OP MODE LOOP
        while (!isStopRequested()) {
            if(robot.intake.checkFreightIn()){
                isFreightIn = !isFreightIn;
            }
            if(isFreightIn){
                intakeMode = 2;
            }
            telemetry.addLine("color sensor detect freight? "+ isFreightIn);

            if (robot.userInput.buttonPressed(2, "x")) {
                if (intakeMode == 2){
                    intakeMode = 0;
                }
                else intakeMode++;
                if (intakeMode == 1){
                    telemetry.addLine("in buttonX loop " + intakeMode);
                }
                if (intakeMode == 2 ){
                    telemetry.addLine("enter transfer " + intakeMode);
                }
                if (intakeMode == 0){
                    telemetry.addLine("transfer done " + intakeMode);
                }
            }

            robot.update();
        }
    }
}