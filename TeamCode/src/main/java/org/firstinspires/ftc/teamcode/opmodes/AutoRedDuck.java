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
public class AutoRedDuck extends LinearOpMode {
    public static double SCAN_FORWARD = -4;
    public static double SCAN_BACKWARD = 1;
    public static double SCAN_RIGHT = 10;
    public static double DUCK_X = -2.93;
    public static double DUCK_Y = -11.9;
    public static double DUCK_HEADING = Math.toRadians(305); // degree
    public static double DUCK_BUF = 5.5;
    public static double HUB_X= -19.77; //-21;
    public static double HUB_Y= 13.93; //1.5; //-25.87;
    public static double HUB_1X= -18.41; //-21;
    public static double HUB_1Y= 12.43; //1.5; //-25.87;
    public static double HUB_HEADING= Math.PI + Math.toRadians(310); //1.14;
    public static double FINAL_HEADING= 140;
    public static boolean GO_TO_WAREHOUSE = true;
    public static double DUCK_LEFT_THRESHOLD = 750;

    private int adjPos(int Pos) {
        return (4 - Pos);
    }

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

        robot.outtake.setServoPosition(0.60);
        robot.intake.setTargetPosition(Intake.Positions.DUMP);
        robot.update();

        elementPos = od.checkDuckPresence();
        elementPos = adjPos(elementPos);
        telemetry.addData("Level:", elementPos);
        telemetry.update();

        // move to spinners
        Trajectory traj_duck = drivetrain.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(DUCK_X, DUCK_Y), DUCK_HEADING)
                .build();

        robot.runCommand(drivetrain.followTrajectory(traj_duck));

        Trajectory traj_duck_back = drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
                .forward(DUCK_BUF)
                .build();

        robot.runCommand(drivetrain.followTrajectory(traj_duck_back));

        double spinPower = -0.5;
        driveTime = 3;
        Spin spinDuck = new Spin(robot.spinner,spinPower, driveTime);
        robot.runCommands(spinDuck);

        // move to HUB
        Trajectory traj_hub;
        if (elementPos == 1) {
            traj_hub = drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate(), true)
                    .splineTo(new Vector2d(HUB_1X, HUB_1Y), HUB_HEADING)
                    .build();
        } else{
            traj_hub = drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate(), true)
                    .splineTo(new Vector2d(HUB_X, HUB_Y), HUB_HEADING)
                    .build();
        }

        // Dump
        Dump dumpL = new Dump(robot, elementPos);

        robot.runCommand(drivetrain.followTrajectory(traj_hub));
        robot.runCommand(dumpL);



        //go back slightly
        robot.runCommand(drivetrain.followTrajectorySequence(
                drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                        .forward(5)
                        .build()));

        //turn
        robot.runCommand(drivetrain.followTrajectorySequence(
                drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                        .turn(Math.toRadians(FINAL_HEADING - drivetrain.getPoseEstimate().getHeading()))
                        .build()
        ));

        if (GO_TO_WAREHOUSE){
            ////////////// TO WAREHOUSE //////////////
            //move to warehouse
            robot.runCommand(drivetrain.followTrajectorySequence(
                    drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                            .forward(80)
                            .build()
            ));

        }else{
            ////////////// TO STORAGE //////////////
            //move to warehouse
            robot.runCommand(drivetrain.followTrajectorySequence(
                drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                        .forward(-30)
                        .build()
            ));
            robot.runCommand(drivetrain.followTrajectorySequence(
                drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                        .strafeRight(-4.5)
                        .build()
            ));



        }

        od.close();

    }

}