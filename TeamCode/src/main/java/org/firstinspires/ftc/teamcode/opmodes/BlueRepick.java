package org.firstinspires.ftc.teamcode.opmodes;

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
public class BlueRepick extends LinearOpMode {
    public static double SCAN_FORWARD = -4;
    public static double SCAN_BACKWARD = 1;
    public static double SCAN_RIGHT = 10;
    public static double DUCK_X = -5.7;
    public static double DUCK_Y = 22.5;
    public static double DUCK_BUF = 2.0;
    public static double HUB_X= -26.5; //-21;
    public static double HUB_Y= 17; //1.5; //-25.87;
    public static double HUB_1X= -25; //-21;
    public static double HUB_1Y= 15; //1.5; //-25.87;
    public static double HUB_HEADING= Math.PI + 5.4; //5.7  //1.14;
    public static double HUB_X_RD= -27; //-21;
    public static double HUB_Y_RD= 12; //1.5; //-25.87;
    public static double HUB_HEADING_RD= Math.PI + 5.0; //5.7  //1.14;
    public static double FINAL_HEADING= -45; //125 + 180;
    public static double FREIGHT_HEADING= 50; //125 + 90;

    public static double REPICK_X = -20;
    public static double REPICK_Y = -36;
    public static double REPICK_HEADING = -45;
    public static double REPICK_FWD = 45;
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

        if (isStopRequested()) return;

        elementPos = od.checkDuckPresence();

        robot.intake.setTargetPosition(Intake.Positions.LIFT);
        robot.update();

        //TODO: move to hub
        robot.runCommand(drivetrain.followTrajectorySequence(
                drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                        //.splineTo(new Vector2d(REPICK_X, REPICK_Y), Math.toRadians(REPICK_HEADING))
                        .forward(-8)
                        .build()));

        Trajectory traj_hub;
        if (elementPos == 1) {
            traj_hub = drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate(), true)
                    .splineTo(new Vector2d(HUB_1X, HUB_1Y), HUB_HEADING)
                    .build();
        } else {
            traj_hub = drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate(), true)
                    .splineTo(new Vector2d(HUB_X, HUB_Y), HUB_HEADING)
                    .build();
        }

        //TODO: Move to hub and Dump to proper level
        //robot.runCommand(drivetrain.followTrajectory(traj_hub));
        Dump dumpL;
        dumpL = new Dump(robot, elementPos);
        robot.runCommand(drivetrain.followTrajectory(traj_hub));
        robot.runCommand(dumpL);

        // move to WH and re-pick
        robot.runCommand(drivetrain.followTrajectorySequence(
                drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                        //.splineTo(new Vector2d(REPICK_X, REPICK_Y), Math.toRadians(REPICK_HEADING))
                        .forward(9)
                        .turn(Math.toRadians(FINAL_HEADING - drivetrain.getPoseEstimate().getHeading()))
                        .forward(REPICK_FWD)
                        .turn(Math.toRadians(FREIGHT_HEADING - drivetrain.getPoseEstimate().getHeading()))
                        //.forward(2)
                        //.splineTo(new Vector2d(REPICK_X, REPICK_Y), Math.toRadians(REPICK_HEADING))
                        //.strafeLeft(5)
                        //.forward(REPICK_FWD)
                        .build()));
        DriveTillIntake driveTillIntake = new DriveTillIntake(robot, robot.mecanumDrive,
                new Pose2d(0.2,0, Math.toRadians(0)),
                3);
        robot.runCommand(driveTillIntake);

        // TODO: Redump
        // go out of WH and dump to level 3
        robot.runCommand(drivetrain.followTrajectorySequence(
                drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                        .turn(Math.toRadians(FINAL_HEADING - drivetrain.getPoseEstimate().getHeading()))
                        .forward(-(REPICK_FWD+10))
                        .build()));


        robot.intake.setTargetPosition(Intake.Positions.LIFT);
        robot.update();

        traj_hub = drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate(), true)
                .splineTo(new Vector2d(HUB_X, HUB_Y), HUB_HEADING)
                .build();
        //robot.runCommand(drivetrain.followTrajectory(traj_hub));
        Dump re_dump = new Dump(robot, 3);
        robot.runCommand(drivetrain.followTrajectory(traj_hub));
        robot.runCommand( re_dump);

        // go back to WH
        robot.runCommand(drivetrain.followTrajectorySequence(
                drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                        .forward(9)
                        .turn(Math.toRadians(FINAL_HEADING - drivetrain.getPoseEstimate().getHeading()))
                        .forward(REPICK_FWD)
                        .build()));




        /*
        Trajectory revToHub = drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate(), true)
                .splineTo(new Vector2d(HUB_X, HUB_Y), HUB_HEADING)
                .build();
        robot.runCommand(drivetrain.followTrajectorySequence(
                drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                        .forward(-REPICK_BWD)
                        .addTrajectory(revToHub)
                        .build()));

         */

/*
        robot.runCommand(drivetrain.followTrajectorySequence(
                drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                        .splineTo(new Vector2d(REPICK_X, REPICK_Y), REPICK_HEADING)
                        .forward(REPICK_FWD)
                        .build()));

 */




        od.close();

    }

}