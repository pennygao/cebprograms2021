package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;
import org.firstinspires.ftc.teamcode.subsystems.UserInput;

import android.util.Log;
@TeleOp
public class   TeleFreight  extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CrabRobot robot = new CrabRobot(this);

        int slidecountup = 0;
        int slidecountdown = 0;
        boolean isApressed = false;
        boolean inTransfer = false;
        boolean slowMode = false;
        boolean toggleSpin = false;


//RESETS
        robot.intake.setTargetPosition(Intake.Positions.RESET);
        robot.outtake.setServoPosition(0.6);

        waitForStart();


//ENTER OP MODE LOOP
        while (!isStopRequested()) {

            boolean buttonA = gamepad2.a; // enter Align
            boolean buttonB = gamepad2.b; // exit Align
            boolean buttonX = gamepad2.x;
            boolean buttonY = gamepad2.y;
            boolean leftBumper = gamepad2.left_bumper;
            boolean rightBumper = gamepad2.right_bumper;


            if (robot.userInput.buttonPressed(1, "a")) {
                slowMode = !slowMode;
            }

            telemetry.addData("dumpServo Position:",robot.outtake.getDumpPosition());
            telemetry.update();

            robot.update();

            //check the bottom of the code (A) for the deleted bit i commented out

//DRIVE MODES
            robot.mecanumDrive.setDrivePower(new Pose2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x));

            if (slowMode) {
                robot.mecanumDrive.setPowerFactor(0.2);
//                telemetry.addLine("slow mode");
                Log.i("drivemode", "slow");
            }
            else {
                robot.mecanumDrive.setPowerFactor(0.8);
//                telemetry.addLine("normie mode");
                Log.i("drivemode", "normie");
            }

//INTAKE
            if(buttonX) {
                robot.intake.start();
                telemetry.addLine("in buttonX loop");
            }

            if(buttonY){
                if (!inTransfer) {
                    robot.intake.stop();
                    inTransfer=true;
                    telemetry.addLine("enter transfer");
                }
                else {
                    robot.intake.reset();
                    inTransfer = false;
                    telemetry.addLine("transfer done");
                }
            }

//SET SLIDE LEVEL


//RAISE SLIDE
            if(buttonA) {
                if (!isApressed) {
                    robot.outtake.dump();
                    isApressed = true;
                }
            } else {
                isApressed = false;
            }
            /*if(buttonB) {
                robot.outtake.goalldown();
                telemetry.addLine("going all down");
            }
            */

//DUMP
            if(leftBumper) {
                int level = robot.outtake.getLevel();
                double servoPosition=0.6;
                switch (level){
                    case 1: servoPosition=0.25; //0.47 before fine-tune; hits the outer part of the tray
                    break;
                    case 2: servoPosition= 0.25;
                    break;
                    case 3: servoPosition= 0.25;
                    break;
                }
                robot.outtake.setServoPosition(servoPosition);
                telemetry.addLine("dumping  ");
            }

            if (rightBumper) {
                robot.outtake.setServoPosition(0.6);
                telemetry.addLine("resetting dumper");
            }


//DUCK SPINNER
            if (robot.userInput.buttonPressed(1, "left_trigger")){
                toggleSpin = !toggleSpin;
            }

            if (toggleSpin) {
                robot.spinner.setPower(0.8);
                Log.i("duckSpin", "spinning");
            }
            else {
                robot.spinner.setPower(0);
                Log.i("duckSpin", "not spinning");
            }


            telemetry.update();
        }
    }
}



// A
// Telemetry print out distL, distR
            /*
            telemetry.addData("distL:", mecanumDrive.getdistL());
            telemetry.addData("distR:", mecanumDrive.getdistR());
            telemetry.addData("Dist reached", mecanumDrive.hubReached());
            telemetry.addData("Turn reached", mecanumDrive.turnReached());

             */

           /* if ( (buttonA || !mecanumDrive.hubReached() || !mecanumDrive.turnReached()) && !buttonB) {
            //if ( (buttonA || !mecanumDrive.hubReached()) && !buttonB) {
                if (!mecanumDrive.getInAlignMode()) {
                    mecanumDrive.setTargetDist(200.0);
                    mecanumDrive.setInAlignMode(true);
                }

                telemetry.addLine("in button A loop");
            } else {
            */
//    mecanumDrive.setInAlignMode(false);