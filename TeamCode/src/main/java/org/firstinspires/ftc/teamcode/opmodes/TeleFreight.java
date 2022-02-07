package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;
import org.firstinspires.ftc.teamcode.subsystems.UserInput;

import android.util.Log;
@TeleOp
public class  TeleFreight  extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CrabRobot robot = new CrabRobot(this);

        boolean isAPressed = false;
        boolean inTransfer = false;
        boolean isFreightIn = false;
        boolean slowMode = false;
        boolean blueToggleSpin = false;
        boolean redToggleSpin = false;
        double gamepadDuckPower=0;
        double duckPower = 0;
        int intakeMode = 0; //toggle between 0(at rest), 1(go down) 2(dump)


//RESETS
        robot.intake.setTargetPosition(Intake.Positions.RESET);
        robot.outtake.setServoPosition(0.8);

        waitForStart();


//ENTER OP MODE LOOP
        while (!isStopRequested()) {

            boolean buttonA = gamepad2.a;
            boolean leftBumper = gamepad2.left_bumper;
            boolean rightBumper = gamepad2.right_bumper;

            robot.update();


            telemetry.addData("dumpServo Position:",robot.outtake.getDumpPosition());



            //check the bottom of the code for the deleted bit i commented out

//DRIVE MODES
            if (robot.userInput.buttonPressed(1, "a")) {
                slowMode = !slowMode;
            }
            robot.mecanumDrive.setDrivePower(new Pose2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x));
            if (slowMode) {
                robot.mecanumDrive.setPowerFactor(0.2);
//                telemetry.addLine("slow mode");
                Log.i("drivemode", "slow");
                telemetry.addLine("Speed: Slow");
            }
            else {
                robot.mecanumDrive.setPowerFactor(0.8);
//                telemetry.addLine("normie mode");
                Log.i("drivemode", "normie");
                telemetry.addLine("Speed: Normie");
            }

//INTAKE
            if(robot.intake.checkFreightIn()){
                robot.intake.stop();
                slowMode = false;
                intakeMode = 2;
            }
            telemetry.addLine("color sensor detect freight? "+ isFreightIn);

            if (robot.userInput.buttonPressed(2, "x")) {
                if (intakeMode == 2){
                    intakeMode = 0;
                }
                else intakeMode++;
                if (intakeMode == 1){
                    robot.intake.start();
                    slowMode = true;
                    telemetry.addLine("in buttonX loop " + intakeMode);
                }
                if (intakeMode == 2 ){
                    robot.intake.stop();
                    telemetry.addLine("enter transfer " + intakeMode);
                }
                if (intakeMode == 0){
                    robot.intake.reset();
                    slowMode = false;
                    telemetry.addLine("transfer done " + intakeMode);
                }
            }

            telemetry.addData("Intake Mode: ", intakeMode);

//SET SLIDE LEVEL
            if (robot.userInput.buttonPressed(2, "b")) {
                robot.outtake.nextDefault();
            }
            Log.i("slide default", ""+robot.outtake.defaultLevel);
            telemetry.addData("slide default level", robot.outtake.defaultLevel );

//RAISE SLIDE
            if (robot.userInput.buttonPressed(2, "a")) {
                isAPressed = !isAPressed;
            }
            if (isAPressed){
                robot.outtake.dump();
                robot.intake.lift();

                isAPressed = false;
            }
            /*
            if(buttonA) {
                if (!isAPressed) {
                    robot.outtake.dump();
                    isAPressed = true;
                }
            } else {
                isAPressed = false;
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
                    case 3: servoPosition= 0.30;
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
            //blue = right; red = left
            /*
            if (robot.userInput.buttonPressed(1, "right_trigger")) {
                duckPower = gamepad1.right_trigger;
                blueToggleSpin = !blueToggleSpin;
            }
            else if (robot.userInput.buttonPressed(1, "left_trigger")) {
                duckPower = gamepad1.left_trigger;
                redToggleSpin = !redToggleSpin;
            }
            if (blueToggleSpin) {
                robot.spinner.setPower(0.5);
                Log.i("duckSpin", "spinning blue");
            }
            else if (redToggleSpin) {
                robot.spinner.setPower(-0.5);
                Log.i("duckSpin", "spinning red");
            }
            else {
                robot.spinner.setPower(0);
                Log.i("duckSpin", "not spinning");
            }
             */
            //read time
            /*
            if (currentTime - previuosTime > )
                increase power by how much time elapsed up to the maxDuckPower determined by
                save currentTime
            */
            gamepadDuckPower = gamepad1.right_trigger - gamepad1.left_trigger;
            robot.spinner.setPower(gamepadDuckPower*0.8);
            Log.i("duckSpinner", "gamepad power: "+gamepadDuckPower);
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