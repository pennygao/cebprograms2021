package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.Dump;
import org.firstinspires.ftc.teamcode.commands.Spin;
import org.firstinspires.ftc.teamcode.commands.DriveForTime;

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

import java.util.List;

@Autonomous(group = "test")
public class AutoFreightDUCK extends LinearOpMode {
    public static double SCAN_FORWARD = -4;
    public static double SCAN_BACKWARD = 1;
    public static double SCAN_RIGHT = 10;
    public static double DUCK_X = -5.7;
    public static double DUCK_Y = 22.5;
    public static double DUCK_BUF = 2.0;
    public static double HUB_X= -20.41; //-21;
    public static double HUB_Y= 0; //1.5; //-25.87;
    public static double HUB_1X= -18.41; //-21;
    public static double HUB_1Y= -2; //1.5; //-25.87;
    public static double HUB_HEADING= Math.PI + 0.85; //1.14;
    public static double FINAL_HEADING= 45;
    private VuforiaCurrentGame vuforiaFreightFrenzy;
    private TfodCurrentGame tfodFreightFrenzy;
    boolean isDuckDetected = false;
    Recognition recognition;
    boolean go = true;

    private int elementPos = 3; // 1: LEFT/LOW, 2: MIDDLE/MID, 3: RIGHT/HI
    @Override
    public void runOpMode() throws InterruptedException {
        double driveTime;

        CrabRobot robot = new CrabRobot(this);
        Drivetrain drivetrain = new Drivetrain(robot);
        robot.registerSubsystem((Subsystem) drivetrain);
        List<Recognition> recognitions;
        int index;
        int position = 0;
        vuforiaFreightFrenzy = new VuforiaCurrentGame();
        tfodFreightFrenzy = new TfodCurrentGame();


        vuforiaFreightFrenzy.initialize(
                "", // vuforiaLicenseKey
                VuforiaLocalizer.CameraDirection.BACK, // cameraDirection
                false, // useExtendedTracking
                false, // enableCameraMonitoring
                VuforiaLocalizer.Parameters.CameraMonitorFeedback.NONE, // cameraMonitorFeedback
                0, // dx
                0, // dy
                0, // dz
                AxesOrder.XYZ, // axesOrder
                0, // firstAngle
                -90, // secondAngle
                0, // thirdAngle
                true); // useCompetitionFieldTargetLocations
        // Set min confidence threshold to 0.7
        tfodFreightFrenzy.initialize(vuforiaFreightFrenzy, (float) 0.7, true, true);
        // Initialize TFOD before waitForStart.
        // Init TFOD here so the object detection labels are visible
        // in the Camera Stream preview window on the Driver Station.
        tfodFreightFrenzy.activate();
        // Enable following block to zoom in on target.
        tfodFreightFrenzy.setZoom(2.5, 16 / 9);
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        // Wait for start command from Driver Station.



        waitForStart();

        if (isStopRequested()) return;

        robot.outtake.setServoPosition(0.60);
        robot.intake.setTargetPosition(Intake.Positions.DUMP);
        robot.update();


        //move forward
        Trajectory traj_forward = drivetrain.trajectoryBuilder(new Pose2d())
                .forward(SCAN_FORWARD)
                .build();

        // TODO: check if element is at LEFT
        if(checkDuckPresence(robot)){
            elementPos = 1;
        }

        // move right to middle barcode
        Trajectory traj_right = drivetrain.trajectoryBuilder(traj_forward.end())
                    .strafeLeft(SCAN_RIGHT)
                    .build();
        // check if element is at MIDDLE
        robot.runCommand(drivetrain.followTrajectory(traj_right));
        if(checkDuckPresence(robot)) {
            //sleep(100);
            elementPos = 2;

        }

        // move to spinners
        Trajectory traj_duck = drivetrain.trajectoryBuilder(traj_right.end())
                .strafeLeft(DUCK_Y+SCAN_RIGHT)
                .build();

        robot.runCommand(drivetrain.followTrajectory(traj_duck));

        /*
        //move to spinner 2.0
        Trajectory traj_duck2 = drivetrain.trajectoryBuilder(traj_duck.end())
                .forward(DUCK_BUF)
                .build();

        robot.runCommand(drivetrain.followTrajectory(traj_duck2));
         */

        double spinPower = 0.5;
        driveTime = 2.2;
        Spin spinDuck = new Spin(robot.spinner,spinPower, driveTime);
        robot.runCommands(spinDuck);

        // move to HUB
        Trajectory traj_hub;
        if (elementPos == 1) {
            traj_hub = drivetrain.trajectoryBuilder(traj_duck.end(), true)
                    .splineTo(new Vector2d(HUB_1X, HUB_1Y), HUB_HEADING)
                    .build();
        } else {
            traj_hub = drivetrain.trajectoryBuilder(traj_duck.end(), true)
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

        //move to warehouse
        robot.runCommand(drivetrain.followTrajectorySequence(
                drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                        .forward(-80)
                        .build()
        ));

        tfodFreightFrenzy.deactivate();
        vuforiaFreightFrenzy.close();
        tfodFreightFrenzy.close();

    }
    private void displayInfo (int i){
        // Display label info.
        // Display the label and index number for the recognition.
        telemetry.addData("label "+i, recognition.getLabel());
        // Display upper corner info.
        // Display the location of the top left corner
        // of the detection boundary for the recognition
        telemetry.addData("Left, Top "+i, Double.parseDouble(JavaUtil.formatNumber(recognition.getLeft(), 0)) + ", " + Double.parseDouble(JavaUtil.formatNumber(recognition.getTop(), 0)));
        // Display lower corner info.
        // Display the location of the bottom right corner
        // of the detection boundary for the recognition
        telemetry.addData("Right, Bottom " +i, Double.parseDouble(JavaUtil.formatNumber(recognition.getRight(), 0)) + ", " + Double.parseDouble(JavaUtil.formatNumber(recognition.getBottom(), 0)));
        // Display Recognition of Duck or Team Element
        if (recognition.getLabel().equals("Duck")||recognition.getLabel().equals("Cube") ) {

            isDuckDetected = true;
            telemetry.addData("Object Detected: Duck", "Duck");
        } else {
            isDuckDetected = false;
        }
    }



    private boolean checkDuckPresence(CrabRobot robot) {
        int index;
        double driveTime;
        //CrabRobot robot = new CrabRobot(this);
        List<Recognition> recognitions;
        recognitions = tfodFreightFrenzy.getRecognitions();
        int position = 1;
        //for (position = 1; position <= 3; position++) {
            for (int i = 0; i < 20; ++i) {
                recognitions = tfodFreightFrenzy.getRecognitions();
                //If list is empty, inform the user. Otherwise, go
                //through list and display info for each recognition.
                if (recognitions.size() == 0) {
                    telemetry.addData("TFOD", "No items detected.");
                } else {
                    index = 0;
                    isDuckDetected = true;
                    // Iterate through list and call a function to
                    // display info for each recognized object.
                    for (Recognition recognition_item : recognitions) {
                        recognition = recognition_item;
                        // Display info.
                        displayInfo(index);
                        // Increment index.
                        index = index + 1;
                    }
                }
                sleep(10);
                telemetry.update();
            }


            if (recognitions.size() == 0) {
                isDuckDetected = false;
            } else {
                isDuckDetected = true;
                index = 0;
                displayInfo(index);
                index = index + 1;
            }
        //}

        return (isDuckDetected);
    }
}


