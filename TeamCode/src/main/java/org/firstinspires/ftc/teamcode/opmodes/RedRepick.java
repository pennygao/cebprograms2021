package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Configuration;
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
//

@Autonomous(group = "test")
public class RedRepick extends LinearOpMode {
    public static double SCAN_FORWARD = -4;
    public static double SCAN_BACKWARD = 1;
    public static double SCAN_RIGHT = 10;
    public static double DUCK_X = -5.7;
    public static double DUCK_Y = 22.5;
    public static double DUCK_BUF = 2.0;
    public static double HUB_X= -26; //-21;
    public static double HUB_Y= -9; //1.5; //-25.87;
    public static double HUB_1X= -26.5; //-21;
    public static double HUB_1Y= -9; //1.5; //-25.87;
    public static double HUB_HEADING= 180+35;//Math.PI + 0.63; //5.7  //1.14;
    public static double WALL_X = 1;
    public static double WALL_Y = 3;
    public static double WALL_HEADING = 90;
    public static double WAREHOUSE_X = -1;
    public static double WAREHOUSE_Y = 24;
    public static double WAREHOUSE_HEADING = 90;
    public static double WALL_FWD = 24 ;
    public static double HUB_X_RD= -27; //-21;
    public static double HUB_Y_RD= 12; //1.5; //-25.87;
    public static double HUB_HEADING_RD= Math.PI + 5.0; //5.7  //1.14;
    public static double FINAL_HEADING= 60; //125 + 180;
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

        if (isStopRequested()) return;
        elementPos = od.checkDuckPresence();
        robot.intake.setTargetPosition(Intake.Positions.LIFT);
        robot.update();


        //TODO: move to hub
        Trajectory traj_hub;
        if (elementPos == 1) {
            traj_hub = drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate(), true)
                    .splineTo(new Vector2d(HUB_1X, HUB_1Y), Math.toRadians(HUB_HEADING))
                    .build();
        } else {
            traj_hub = drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate(), true)
                    .splineTo(new Vector2d(HUB_X, HUB_Y),Math.toRadians(HUB_HEADING))
                    .build();
        }
        Dump dumpL;
        dumpL = new Dump(robot, elementPos);


        //TODO: Move to hub and Dump to proper level
        robot.runCommand(drivetrain.followTrajectorySequence(
                drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                        //.splineTo(new Vector2d(REPICK_X, REPICK_Y), Math.toRadians(REPICK_HEADING))
                        //.forward(-8)
                        .addTrajectory(traj_hub)
                        .addTemporalMarker(1, ()->robot.runCommand(dumpL))
                        .build()));

        // move to WH and re-pick
        robot.runCommand(drivetrain.followTrajectorySequence(
                drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                        //.forward(2)
                        .splineTo(new Vector2d(WALL_X, WALL_Y), Math.toRadians(WALL_HEADING))
                        //.splineTo(new Vector2d(WAREHOUSE_X, WAREHOUSE_Y), Math.toRadians(WAREHOUSE_HEADING))
                        .strafeRight(5)
                        .forward(WALL_FWD)
                        .build()));


        DriveTillIntake driveTillIntake = new DriveTillIntake(robot, robot.mecanumDrive,
                new Pose2d(0.3,0, Math.toRadians(0)),
                3.5);
        robot.runCommand(driveTillIntake);

        if (Configuration.intakeFreightIn == true) {
            // TODO: Redump
            // go out of WH and dump to level 3
            robot.runCommand(drivetrain.followTrajectorySequence(
                    drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                            //.turn(Math.toRadians(FINAL_HEADING - drivetrain.getPoseEstimate().getHeading()))
                            .strafeRight(2)
                            .forward(-(WALL_FWD + 12))
                            .build()));

            robot.intake.setTargetPosition(Intake.Positions.LIFT);
            robot.update();
            Trajectory traj_hub_repick = drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate(), true)
                    .splineTo(new Vector2d(HUB_X - 3, HUB_Y - 2.5), Math.toRadians(HUB_HEADING-5))
                    .build();


            //robot.runCommand(drivetrain.followTrajectory(traj_hub));
            Dump re_dump = new Dump(robot, 3);
            robot.runCommand(drivetrain.followTrajectorySequence(
                    drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                            //.splineTo(new Vector2d(REPICK_X, REPICK_Y), Math.toRadians(REPICK_HEADING))
                            //.forward(-8)
                            .addTrajectory(traj_hub_repick)
                            .addTemporalMarker(1, () -> robot.runCommand(re_dump))
                            .build()));
            //robot.intake.setTargetPosition(Intake.Positions.RESET);

            // go back to WH
            robot.runCommand(drivetrain.followTrajectorySequence(
                    drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                            .splineTo(new Vector2d(WALL_X, WALL_Y), Math.toRadians(WALL_HEADING))
                            .strafeRight(6)
                            //.splineTo(new Vector2d(WAREHOUSE_X,WAREHOUSE_Y+10),WAREHOUSE_HEADING)
                            .forward(WALL_FWD + 20)
                            .build()));
        }



        od.close();

    }

}
