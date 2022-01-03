package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.Spin;
import org.firstinspires.ftc.teamcode.commands.DriveForTime;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

@Autonomous(group = "test")
public class AutoFreight extends LinearOpMode {
    public static double SCAN_FORWARD = -5.7;
    public static double SCAN_RIGHT = 10;
    public static double DUCK_X = -5.7;
    public static double DUCK_Y = 21.0;
    public static double DUCK_BUF = 2.0;
    public static double HUB_X= -21.15;//-20.41;
    public static double HUB_Y= 1.5; //-25.87;
    public static double HUB_HEADING= Math.PI + 0.85; //1.14;
    @Override
    public void runOpMode() throws InterruptedException {
        double driveTime;

        CrabRobot robot = new CrabRobot(this);
        Drivetrain drivetrain = new Drivetrain(robot);
        robot.registerSubsystem((Subsystem) drivetrain);

        waitForStart();

        if (isStopRequested()) return;

        //move forward
        Trajectory traj_forward = drivetrain.trajectoryBuilder(new Pose2d())
                .forward(SCAN_FORWARD)
                .build();

        robot.runCommand(drivetrain.followTrajectory(traj_forward));

        sleep(1000);

        // move right to middle barcode
        Trajectory traj_right = drivetrain.trajectoryBuilder(traj_forward.end())
                .strafeLeft(SCAN_RIGHT)
                .build();

        robot.runCommand(drivetrain.followTrajectory(traj_right));

        sleep(1000);
        // move right to right barcode
        Trajectory traj_right2 = drivetrain.trajectoryBuilder(traj_right.end())
                .strafeLeft(SCAN_RIGHT)
                .build();

        robot.runCommand(drivetrain.followTrajectory(traj_right2));

        sleep(1000);

        // move to spinner
        Trajectory traj_duck = drivetrain.trajectoryBuilder(traj_right2.end())
                .strafeLeft(DUCK_Y)
                .build();

        robot.runCommand(drivetrain.followTrajectory(traj_duck));

        sleep(1000);
/*
        //move to spinner 2.0
        Trajectory traj_duck2 = drivetrain.trajectoryBuilder(traj_duck.end())
                .forward(DUCK_BUF)
                .build();

        robot.runCommand(drivetrain.followTrajectory(traj_duck2));

        sleep(1000);
*/
        double spinPower = 0.5;
        driveTime = 2.5;
        Spin spinDuck = new Spin(robot.spinner,spinPower, driveTime);
        robot.runCommands(spinDuck);

        // move to HUB
        Trajectory traj_hub = drivetrain.trajectoryBuilder(traj_duck.end(), true)
                .splineTo(new Vector2d(HUB_X, HUB_Y), HUB_HEADING)
                .build();

        robot.runCommand(drivetrain.followTrajectory(traj_hub));

        sleep(1000);

        telemetry.addData("end X", traj_hub.end().component1());
        telemetry.addData("end Y", traj_hub.end().component2());
        telemetry.addData("end heading", traj_hub.end().component3());
        telemetry.update();


/*
        robot.runCommand(drivetrain.followTrajectory(
                drivetrain.trajectoryBuilder(traj.end(), true)
                        .splineTo(new Vector2d(0, 0), Math.toRadians(180))
                        .build()
        ));

 */
    }
}


