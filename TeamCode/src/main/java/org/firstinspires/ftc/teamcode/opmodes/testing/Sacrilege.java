package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.DriveTillIntake;
import org.firstinspires.ftc.teamcode.commands.Dump;
//import org.firstinspires.ftc.teamcode.commands.Spin;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.commands.Turn;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TfodCurrentGame;
import org.firstinspires.ftc.teamcode.subsystems.objectDetector;

import java.util.List;
import android.util.Log;

@Autonomous(group = "test")
public class Sacrilege extends LinearOpMode {
    public static double SCAN_FORWARD = -4;
    public static double SCAN_BACKWARD = 1;
    public static double SCAN_RIGHT = 10;
    public static double DUCK_X = -5.7;
    public static double DUCK_Y = 22.5;
    public static double DUCK_BUF = 2.0;
    public static double HUB_X= -30.5; //-21;
    public static double HUB_Y= -12; //1.5; //-25.87;
    public static double HUB_1X= -29; //-21;
    public static double HUB_1Y= -11; //1.5; //-25.87;
    public static double HUB_HEADING= Math.PI + 0.615; //5.7  //1.14;
    public static double HUB_X_RD= -27; //-21;
    public static double HUB_Y_RD= 12; //1.5; //-25.87;
    public static double HUB_HEADING_RD= Math.PI + 5.0; //5.7  //1.14;
    public static double FINAL_HEADING= 62; //125 + 180;
    public static double FREIGHT_HEADING= -30; //125 + 90;

    public static double REPICK_X = -20;
    public static double REPICK_Y = -36;
    public static double REPICK_HEADING = -45;
    public static double REPICK_FWD = 43;
    public static double REPICK_BWD = 40;


    private int elementPos = 3; // 1: LEFT/LOW, 2: MIDDLE/MID, 3: RIGHT/HI
    @Override
    public void runOpMode() throws InterruptedException {
        int elementPos = 3;

        CrabRobot robot = new CrabRobot(this);
        Drivetrain drivetrain = new Drivetrain(robot);
        robot.registerSubsystem((Subsystem) drivetrain);
        objectDetector od = new objectDetector(robot, telemetry);
        robot.registerSubsystem((Subsystem)od);

        od.init();
        // Wait for start command from Driver Station.

        waitForStart();

        // TODO: Redump
        // go out of WH and dump to level 3
        robot.runCommand(drivetrain.followTrajectorySequence(
                drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                        .turn(Math.toRadians(10+FINAL_HEADING - drivetrain.getPoseEstimate().getHeading()))
                        .forward(-(REPICK_FWD+10))
                        .build()));
        od.close();

    }

}

