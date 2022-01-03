package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

@Autonomous(group = "test")
public class SplineTest extends LinearOpMode {
    public static double X_POS = 30.0;
    public static double Y_POS = 30.0;
    @Override
    public void runOpMode() throws InterruptedException {

        CrabRobot robot = new CrabRobot(this);
        Drivetrain drivetrain = new Drivetrain(robot);
        robot.registerSubsystem((Subsystem) drivetrain);

        waitForStart();

        if (isStopRequested()) return;

        Trajectory traj = drivetrain.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(X_POS, Y_POS), 0)
                .build();

        robot.runCommand(drivetrain.followTrajectory(traj));

        sleep(2000);

        robot.runCommand(drivetrain.followTrajectory(
                drivetrain.trajectoryBuilder(traj.end(), true)
                        .splineTo(new Vector2d(0, 0), Math.toRadians(180))
                        .build()
        ));
    }
}