package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Robot;

public class CrabRobot extends Robot {
    public final SimpleMecanumDrive mecanumDrive;
    public final Outtake outtake;
    public final duckSpinner spinner;
    public final Intake intake;
    public final UserInput userInput;
    public final ArmClaw arm;
    public CrabRobot(LinearOpMode opMode) {
        super(opMode);
        mecanumDrive = new SimpleMecanumDrive(this);
        registerSubsystem(mecanumDrive);
        outtake = new Outtake(this, opMode.telemetry);
        registerSubsystem(outtake);
        spinner = new duckSpinner(this);
        registerSubsystem(spinner);
        intake = new Intake(this);
        registerSubsystem(intake);
        arm = new ArmClaw(this);
        registerSubsystem(arm);

        userInput = new UserInput(opMode.gamepad1, opMode.gamepad2);
        registerSubsystem(userInput);
    }
}
