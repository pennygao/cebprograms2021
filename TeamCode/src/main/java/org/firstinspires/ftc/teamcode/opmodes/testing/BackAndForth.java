package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.commands.Turn;

import org.firstinspires.ftc.teamcode.robot.Command;
import org.firstinspires.ftc.teamcode.robot.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.SimpleMecanumDrive;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(group = "drive")
public class BackAndForth extends LinearOpMode {
    public static double DISTANCE = 48; // deg

    @Override
    public void runOpMode() throws InterruptedException {
        CrabRobot robot = new CrabRobot(this);
        Drivetrain drivetrain = new Drivetrain(robot);
        robot.registerSubsystem((Subsystem) drivetrain);

        Trajectory trajForward = drivetrain.trajectoryBuilder(new Pose2d())
                .forward(DISTANCE)
                .build();
        Trajectory trajBackward = drivetrain.trajectoryBuilder(trajForward.end())
                .back(DISTANCE)
                .build();
        waitForStart();


        if (isStopRequested()) return;

        while (!isStopRequested()){
            robot.runCommand((Command) drivetrain.followTrajectory(trajForward));
            robot.runCommand((Command) drivetrain.followTrajectory(trajBackward));
        }

    }
}
