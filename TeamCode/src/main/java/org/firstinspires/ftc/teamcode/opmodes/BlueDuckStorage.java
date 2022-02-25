package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.Dump;
import org.firstinspires.ftc.teamcode.commands.Spin;
//import org.firstinspires.ftc.teamcode.commands.DriveForTime;

//import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
//import org.firstinspires.ftc.teamcode.commands.Turn;
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
public class BlueDuckStorage extends LinearOpMode {
    public static double SCAN_FORWARD = -4;
    public static double SCAN_BACKWARD = 1;
    public static double SCAN_RIGHT = 10;
    public static double DUCK_X = -5;
    public static double DUCK_Y = 25;
    public static double DUCK_HEADING = Math.toRadians(39); // degree
    public static double DUCK_BUF = 12;
    public static double HUB_X= -29; //-20.1 ;
    public static double HUB_Y= -8; //0.5
    public static double HUB_2X=-28.5;
    public static double HUB_2Y= -7.5;
    public static double HUB_1X= -28; //-21;
    public static double HUB_1Y= -7; //1.5; //-25.87;
    public static double HUB_HEADING= Math.toRadians(140+75);; //1.14;
    public static double FINAL_HEADING= 43;
    public static double DUCK_LEFT_THRESHOLD = 750;
    public static boolean GO_TO_WAREHOUSE = false;
    public static double TO_STORAGE_DIS = 33;
    public static double TO_STORAGE_RIGHT = -20;

    private int elementPos = 3; // 1: LEFT/LOW, 2: MIDDLE/MID, 3: RIGHT/HI
    @Override
    public void runOpMode() throws InterruptedException {
        double driveTime;

        CrabRobot robot = new CrabRobot(this);
        Drivetrain drivetrain = new Drivetrain(robot);
        robot.registerSubsystem((Subsystem) drivetrain);
        objectDetector od = new objectDetector(robot, telemetry);
        robot.registerSubsystem((Subsystem)od);

        od.init();
        // Wait for start command from Driver Station.

        waitForStart();

        if (isStopRequested()) return;

        robot.intake.setTargetPosition(Intake.Positions.DUMP);
        robot.update();

        elementPos = od.checkDuckPresence();

        // move to spinners
        Trajectory traj_duck = drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
                .splineTo(new Vector2d(DUCK_X, DUCK_Y), DUCK_HEADING)
                .build();

        robot.runCommand(drivetrain.followTrajectorySequence(
                drivetrain.trajectorySequenceBuilder(new Pose2d())
                        .forward(-5) //move forward a little bit
                        .addTrajectory(traj_duck)
                        .forward(DUCK_BUF)
                        .build()
        ));

        // spinduck
        double spinPower = 0.4;
        driveTime = 2.5;
        Spin spinDuck = new Spin(robot.spinner,spinPower, driveTime);
        robot.runCommands(spinDuck);

        // move to HUB
        Trajectory traj_hub;
        if (elementPos == 1) {
            traj_hub = drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate(), true)
                    .splineTo(new Vector2d(HUB_1X, HUB_1Y), HUB_HEADING)
                    .build();
        } else if(elementPos == 2) {
            traj_hub = drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate(), true)
                    .splineTo(new Vector2d(HUB_2X, HUB_2Y), HUB_HEADING)
                    .build();
        } else {
            traj_hub = drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate(), true)
                    .splineTo(new Vector2d(HUB_X, HUB_Y), HUB_HEADING)
                    .build();
        }

        // Dump
        robot.intake.setTargetPosition(Intake.Positions.LIFT);
        robot.update();
        Dump dumpL = new Dump(robot, elementPos);

        robot.runCommand(drivetrain.followTrajectorySequence(
                drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                        //.splineTo(new Vector2d(REPICK_X, REPICK_Y), Math.toRadians(REPICK_HEADING))
                        //.forward(-8)
                        .addTrajectory(traj_hub)
                        .addTemporalMarker(1.5, ()->robot.runCommand(dumpL))
                        .build()));



        //go back slightly
        robot.runCommand(drivetrain.followTrajectorySequence(
                drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                        .forward(7)
                        .turn(Math.toRadians(FINAL_HEADING - drivetrain.getPoseEstimate().getHeading()))
                        .forward(TO_STORAGE_DIS)
                        .strafeRight(TO_STORAGE_RIGHT)
                        .build()));



        od.close();

    }

}

