package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.NanoClock;

import org.firstinspires.ftc.teamcode.robot.Command;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;
import org.firstinspires.ftc.teamcode.subsystems.SimpleMecanumDrive;

public class DriveTillIntake implements Command {
    CrabRobot robot;
    SimpleMecanumDrive mecanumDrive;
    NanoClock clock;
    Pose2d drivePower;
    double initialTimeStamp;
    double driveTime;

    public DriveTillIntake (CrabRobot robot, SimpleMecanumDrive drive, Pose2d power, double time){
        this.robot = robot;
        mecanumDrive=drive;
        clock=NanoClock.system();
        drivePower= power;
        driveTime=time;
    }
    @Override
    public void start() {
        mecanumDrive.setDrivePower(drivePower);
        initialTimeStamp=clock.seconds();
        robot.intake.start();
    }

    @Override
    public void update() {
        if (robot.intake.checkFreightIn()) {
            robot.intake.stop();
            mecanumDrive.setDrivePower(new Pose2d(0,0,0));
        }
    }

    @Override
    public void stop() {
        mecanumDrive.setDrivePower(new Pose2d());
    }

    @Override
    public boolean isCompleted() {
        double currentTime=clock.seconds()-initialTimeStamp;
        return (currentTime>=driveTime); // || robot.intake.checkFreightIn());
    }
}
