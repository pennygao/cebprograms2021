package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;
@TeleOp
public class   TeleFreight  extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CrabRobot robot = new CrabRobot(this);

        int slidecountup = 0;
        int slidecountdown = 0;
        boolean isApressed = false;
        boolean inTransfer = false;



//RESETS
        robot.intake.setTargetPosition(Intake.Positions.RESET);
        robot.outtake.setServoPosition(0.85);

        waitForStart();

        while (!isStopRequested()) {

            boolean buttonA = gamepad2.a; //enter Align
            boolean buttonB = gamepad2.b; // exit Align
            boolean buttonX = gamepad2.x;
            boolean buttonY = gamepad2.y;
            boolean leftBumper = gamepad2.left_bumper;
            boolean rightBumper = gamepad2.right_bumper;
            boolean slowMode = gamepad1.a;
            boolean normieMode = gamepad1.b;
            float leftTrigger = gamepad1.left_trigger;
            float rightTrigger = gamepad1.right_trigger;

            telemetry.addData("slide level init: ", robot.outtake.getLevel());
            telemetry.addData("dumpServo Position:",robot.outtake.getDumpPosition());
            telemetry.update();

            robot.update();

            //check the bottom of the code (A) for the deleted bit i commented out

//DRIVE
            robot.mecanumDrive.setDrivePower(new Pose2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x));

            if (slowMode) {
                robot.mecanumDrive.setPowerFactor(0.2);
            }
            if (normieMode) {
                robot.mecanumDrive.setPowerFactor(0.8);
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


//RAISE SLIDE
            if(buttonA) {
                if (!isApressed) {
                    robot.outtake.goUp();
                    isApressed = true;
                    telemetry.addData("going up. level: ", robot.outtake.getLevel());
                }
            } else {
                isApressed = false;
            }

            if(buttonB) {
                robot.outtake.goalldown();
                telemetry.addLine("going all down");
            }

//DUMP
            if(leftBumper) {
                int level = robot.outtake.getLevel();
                double servoPosition=0.85;
                switch (level){
                    case 1: servoPosition=0.47;
                    break;
                    case 2: servoPosition= 0.45;
                    break;
                    case 3: servoPosition= 0.48;
                    break;
                }
                robot.outtake.setServoPosition(servoPosition);
                telemetry.addLine("dumping  ");
            }

            if (rightBumper) {
                robot.outtake.setServoPosition(0.85);
                telemetry.addLine("resetting dumper");
            }


//DUCK SPINNER
            if (leftTrigger > 0 ) {
                robot.spinner.setPower(3);
            }
            if (rightTrigger > 0) {
                robot.spinner.setPower(0);
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