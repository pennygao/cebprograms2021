package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.LED;

import org.firstinspires.ftc.teamcode.Configuration;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;
import org.firstinspires.ftc.teamcode.subsystems.UserInput;

import android.util.Log;
@TeleOp
public class  TeleFreight  extends LinearOpMode {
    //Speed LED Definition
    //Green = slow mode, red = fast mode
    private LED SpeedLEDRed;
    private LED SpeedLEDGreen;
    private double intakeDownAngle = Configuration.INTAKE_ANGLE_DOWN;
    @Override
    public void runOpMode() throws InterruptedException {
        CrabRobot robot = new CrabRobot(this);
        SpeedLEDGreen = hardwareMap.get(LED.class, "green");
        SpeedLEDRed = hardwareMap.get(LED.class, "red");
        boolean isAPressed = false;
        boolean inTransfer = false;
        boolean isFreightIn = false;
        boolean slowMode = false;
        double gamepadDuckPower=0;
        double duckPower = 0;
        int intakeMode = 0; //toggle between 0(at rest), 1(go down) 2(dump)
        int clawModes = 0;


//RESETS
        robot.intake.setTargetPosition(Intake.Positions.DUMP);
        robot.outtake.setServoPosition(0.8);

        waitForStart();

        robot.arm.setArmTargetPosition(Configuration.ARM_RESET);

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

                SpeedLEDGreen.enableLight(false);
                SpeedLEDRed.enableLight(true);
            }
            else {
                robot.mecanumDrive.setPowerFactor(0.8);
//                telemetry.addLine("normie mode");
                Log.i("drivemode", "normie");
                telemetry.addLine("Speed: Normie");

                SpeedLEDRed.enableLight(false);
                SpeedLEDGreen.enableLight(true);
            }


//INTAKE

            //lower intake
            if (robot.userInput.buttonPressed(2, "left_bumper")){
                intakeDownAngle -= 3;
                Log.i("intake angle", "down, " + intakeDownAngle);
            }
            if (robot.userInput.buttonPressed(2, "right_bumper")){
                intakeDownAngle += 3;
                Log.i("intake angle", "up, " + intakeDownAngle);
            }
            telemetry.addLine("intake down angle: " + intakeDownAngle);
            Log.i("intake angle", "angle, " + intakeDownAngle);

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
                if (intakeMode == 1){// start intake
                    robot.intake.teleStart(intakeDownAngle);
                    slowMode = true;
                    telemetry.addLine("in buttonX loop " + intakeMode);

                }
                if (intakeMode == 2 ){ // stop intake
                    robot.intake.stop();
                    telemetry.addLine("enter transfer " + intakeMode);
                }
                if (intakeMode == 0){
                    robot.intake.reset();
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
            robot.spinner.setPower(gamepadDuckPower*0.7);
            Log.i("duckSpinner", "gamepad power: "+gamepadDuckPower);
            telemetry.update();

            //claw arm
            /*
            //version a
            if (gamepad1.b) {
                robot.arm.setArmTargetPosition(Configuration.ARM_RESET);
                telemetry.addLine("Arm pos Reset");
            }
            if (gamepad1.x) {
                robot.arm.setArmTargetPosition(Configuration.ARM_GRAB);
                telemetry.addLine("Arm pos Reset");
            }
            if (gamepad1.y) {
                //double servoarmPosition = 0.25;
                robot.arm.setArmTargetPosition(Configuration.ARM_HOLD);
                telemetry.addLine("Arm Pos Hold");
                //  telemetry.addData("armposition5-B:", arm.getarmPosition());
            }
            if (gamepad1.left_bumper) {
                robot.arm.setArmTargetPosition(Configuration.ARM_CAP);
                telemetry.addLine("Arm Pos Placing");
                telemetry.addData("armposition-C:", robot.arm.getarmPosition());
            } */
            //version b
            boolean moveClaw = false;
            if (robot.userInput.buttonPressed(1, "x")){
                if (clawModes == 4){
                    clawModes = 0;
                }
                else clawModes++;
                moveClaw= true;
            }
            if (robot.userInput.buttonPressed(1, "b")){
                if (clawModes == 0){
                    clawModes = 4;
                }
                else clawModes--;
                moveClaw = true;
            }
            if (moveClaw){
                if (clawModes==0) {
                    robot.arm.setArmTargetPosition(Configuration.ARM_RESET);
                    telemetry.addLine("Arm pos Reset");
                }
                if (clawModes==1) {
                    robot.arm.setArmTargetPosition(Configuration.ARM_GRAB);
                    telemetry.addLine("Arm pos Reset");
                }
                if (clawModes==2) {
                    robot.arm.setArmTargetPosition(Configuration.ARM_HOLD);
                    telemetry.addLine("Arm Pos Hold");
                    //  telemetry.addData("armposition-B:", arm.getarmPosition());
                }
                if (clawModes==3) {
                    robot.arm.setArmTargetPosition(Configuration.ARM_CAP);
                    telemetry.addLine("Arm Pos Placing");
                    telemetry.addData("armposition-C:", robot.arm.getarmPosition());
                }
            }
            if (gamepad1.left_bumper){
                robot.arm.setArmTargetPosition(robot.arm.getarmPosition() - 0.007);
            }
            if (gamepad1.right_bumper){
                robot.arm.setArmTargetPosition(robot.arm.getarmPosition() + 0.007);
            }

            Log.i("armClaw", "armposition: " + robot.arm.getarmPosition());

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