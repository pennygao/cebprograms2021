package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.util.NanoClock;

import org.firstinspires.ftc.teamcode.robot.Command;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;

public class Dump implements Command {

    CrabRobot robot;
    Outtake outtake;
    Intake intake;
    private double startTime;
    int       level;

    public Dump(CrabRobot robot, int level) {
        this.outtake = robot.outtake;
        this.intake = robot.intake;
        this.level= level;
        this.robot= robot;
    }

    @Override
    public void start() {
        outtake.setDefault(level);
        outtake.dump();
        outtake.setServoPosition(0.61);
        intake.setTargetPosition(Intake.Positions.DUMP);
        startTime = NanoClock.system().seconds();
        robot.update();
    }

    @Override
    public void update() {

    }

    @Override
    public void stop() {

    }

    @Override
    public boolean isCompleted() {
        return (outtake.dumpDone());
        /*
        if (NanoClock.system().seconds() - startTime < 0.8) { // Dump takes at least 1 sec. FIXME
            return false;
        } else {
            return (outtake.dumpDone());
        }

         */
    }
}
