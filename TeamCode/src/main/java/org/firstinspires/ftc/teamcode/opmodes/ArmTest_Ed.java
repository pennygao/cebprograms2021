package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Arm;

@TeleOp
public class ArmTest_Ed extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this);
        Arm arm = new Arm(robot);
        arm.setArmPosition(0.0);
        arm.setClawPosition(0.0);
        telemetry.addData("armTarget0: ", arm.getArmTargetPosition());
        waitForStart();
        robot.registerSubsystem(arm);
        telemetry.addData("clawposition0: ", arm.getclawPosition());
     //   telemetry.update();

        while (!isStopRequested()) {
            boolean Abutton = gamepad1.a;
            boolean Bbutton = gamepad1.b;
            boolean Xbutton = gamepad1.x;
            boolean Ybutton = gamepad1.y;

            robot.update();
            telemetry.addData("armTarget-1: ", arm.getArmTargetPosition());
         //   telemetry.update();

            // Telemetry print out distL, distR

            telemetry.addData("armposition:", arm.getarmPosition());

         //   telemetry.addData("clawposition:", arm.getclawPosition());

//            telemetry.update();

           /* if ( (buttonA || !mecanumDrive.hubReached() || !mecanumDrive.turnReached()) && !buttonB) {
            //if ( (buttonA || !mecanumDrive.hubReached()) && !buttonB) {
                if (!mecanumDrive.getInAlignMode()) {
                    mecanumDrive.setTargetDist(200.0);
                    mecanumDrive.setInAlignMode(true);
                }

                telemetry.addLine("in button A loop");
            } else {
            */

            if(Abutton) {
                double servoarmPosition = 0.85;
                arm.setArmPosition(servoarmPosition);
                telemetry.addData("armposition-A:", arm.getarmPosition());
                telemetry.addLine("Arm pos Resting");

            }
            if(Xbutton) {
                double servoarmPosition = 0.6;
                arm.setArmPosition(servoarmPosition);
                telemetry.addLine("Arm Pos grab");
                telemetry.addData("armposition-B:", arm.getarmPosition());
            }
            if(Ybutton) {
                //arm.changeArmDirection();
                double servoarmPosition = 0.01;
                //arm.changeArmDirection();
                arm.setArmPosition(servoarmPosition);
                telemetry.addLine("Arm Pos Placing");
                telemetry.addData("armposition-C:", arm.getarmPosition());
            }

            if(Bbutton) {
                //arm.changeClawDirection();
                double servoclawPosition = 0.25;
                arm.setClawPosition(servoclawPosition);
                telemetry.addLine("Closed");
                telemetry.addData("Claw position:", arm.getclawPosition());
            }
            if(gamepad1.right_bumper) {
                //arm.changeClawDirection();
                double servoclawPosition = 0.01;
                arm.setClawPosition(servoclawPosition);
                telemetry.addLine("Open");
            }
            if(gamepad1.left_bumper){
                arm.setArmPosition(0);
                telemetry.addLine("0");
            }
            if(gamepad1.dpad_down){
                arm.setClawPosition(0.35);
                telemetry.addLine("0");
            }


            telemetry.addData("clawposition0: ", arm.getclawPosition());
            telemetry.update();

        }

    }
}