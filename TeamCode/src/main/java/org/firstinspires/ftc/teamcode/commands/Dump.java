package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.util.NanoClock;

import org.firstinspires.ftc.teamcode.robot.Command;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.duckSpinner;

public class Dump implements Command {

    Outtake outtake;
    Intake intake;
    private double startTime;
    int       level;

    public Dump(Outtake outtake, Intake intake, int level) {
        this.outtake = outtake;
        this.intake = intake;
        this.level= level;
    }

    @Override
    public void start() {
        outtake.setDefault(level);
        outtake.dump();
        outtake.setServoPosition(0.6);
        intake.setTargetPosition(Intake.Positions.DUMP);
        startTime = NanoClock.system().seconds();
    }

    @Override
    public void update() {

    }

    @Override
    public void stop() {

    }

    @Override
    public boolean isCompleted() {
        if (NanoClock.system().seconds() - startTime < 1) { // Dump takes at least 1 sec. FIXME
            return false;
        } else {
            return (outtake.dumpDone());
        }
    }
}
