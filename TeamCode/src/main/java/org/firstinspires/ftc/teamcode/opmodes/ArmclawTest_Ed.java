package org.firstinspires.ftc.teamcode.opmodes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Configuration;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.subsystems.ArmClaw;
@TeleOp
public class ArmclawTest_Ed extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this);
        ArmClaw arm = new ArmClaw(robot);
           waitForStart();
        robot.registerSubsystem(arm);
          while (!isStopRequested()) {
            boolean Abutton = gamepad1.a;
            boolean Bbutton = gamepad1.b;
            boolean Xbutton = gamepad1.x;
            boolean Ybutton = gamepad1.y;

            robot.update();

            telemetry.addData("armposition:", arm.getarmPosition());
            if (Abutton) {
                arm.setArmTargetPosition(Configuration.ARM_RESET);

                telemetry.addLine("Arm pos Reset");

            }
              if (Bbutton) {
                  arm.setArmTargetPosition(Configuration.ARM_GRAB);

                  telemetry.addLine("Arm pos Reset");

              }
            if (Xbutton) {
                //double servoarmPosition = 0.25;
                arm.setArmTargetPosition(Configuration.ARM_HOLD);
                telemetry.addLine("Arm Pos Hold");
              //  telemetry.addData("armposition-B:", arm.getarmPosition());
            }
            if (Ybutton) {
                  arm.setArmTargetPosition(Configuration.ARM_CAP);
                telemetry.addLine("Arm Pos Placing");
                telemetry.addData("armposition-C:", arm.getarmPosition());
            }


 //           if (gamepad1.dpad_down) {
                //              arm.grab();
 //           }

//            telemetry.addData("clawposition0: ", arm.getclawPosition()); }
            telemetry.update();

        }

    }
}

