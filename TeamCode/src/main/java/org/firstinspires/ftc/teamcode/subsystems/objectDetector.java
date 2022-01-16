package org.firstinspires.ftc.teamcode.subsystems;
import android.util.Log;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TfodCurrentGame;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Subsystem;

import java.util.List;


public class objectDetector implements Subsystem {
    public static int    NUM_TRY = 10;
    public static double DUCK_LEFT_THRESHOLD = 750;
    public static double CONFIDENCE = 0.7;

    private VuforiaCurrentGame vuforiaFreightFrenzy;
    private TfodCurrentGame tfodFreightFrenzy;
    Recognition recognition;
    Telemetry telemetry;
    Robot     robot;

    public objectDetector(Robot robot, Telemetry telemetry) {
        vuforiaFreightFrenzy = new VuforiaCurrentGame();
        tfodFreightFrenzy = new TfodCurrentGame();
        this.telemetry = telemetry;
        this.robot = robot;
    }

    public void init() {
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
        tfodFreightFrenzy.initialize(vuforiaFreightFrenzy, (float)CONFIDENCE, true, true);
        // Initialize TFOD before waitForStart.
        // Init TFOD here so the object detection labels are visible
        // in the Camera Stream preview window on the Driver Station.
        tfodFreightFrenzy.activate();
        // Enable following block to zoom in on target.
        //tfodFreightFrenzy.setZoom(2.5, 16 / 9);
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Press Play to start");
        telemetry.update();
    }

    public void close() {
        tfodFreightFrenzy.deactivate();
        vuforiaFreightFrenzy.close();
        tfodFreightFrenzy.close();
    }



    private void displayInfo (int i, Recognition recognition){
        // Display label info.
        // Display the label and index number for the recognition.
        Log.i("Index", "%0d" + i);
        Log.i("Label %s", recognition.getLabel());
        // Display upper corner info.
        // Display the location of the top left corner
        // of the detection boundary for the recognition
        Log.i("Left, Top ", Double.parseDouble(JavaUtil.formatNumber(recognition.getLeft(), 0)) + ", " + Double.parseDouble(JavaUtil.formatNumber(recognition.getTop(), 0)));
        // Display lower corner info.
        // Display the location of the bottom right corner
        // of the detection boundary for the recognition
        Log.i("Right, Bottom ", Double.parseDouble(JavaUtil.formatNumber(recognition.getRight(), 0)) + ", " + Double.parseDouble(JavaUtil.formatNumber(recognition.getBottom(), 0)));
        // Display Recognition of Duck or Team Element
        if (recognition.getLabel().equals("Duck")||recognition.getLabel().equals("Cube") ) {

            telemetry.addData("Object Detected: Duck", "Duck");
        } else {
        }
    }



    // if Lable is Duck, and left < threshold, pos = 1
    // if Label is Duck, and left >= threshold pos = 2
    // if Label is not Duck, or nothing, pos = 3
    // recognitions.size() == 0
    // recognition.getLabel().equals("Duck")
    // recognition.getLeft()
    public int checkDuckPresence() {
        int pos = 3;
        List<Recognition> recognitions;
        int index;

        double left = 0;
        boolean foundDuck = false;

        for (int i=0; i< NUM_TRY; i++) {
            recognitions = tfodFreightFrenzy.getRecognitions();
            //If list is empty, inform the user. Otherwise, go
            //through list and display info for each recognition.
            if (recognitions.size() == 0) {
                telemetry.addData("TFOD", "No items detected.");
                pos = 3;
            } else {

                // Iterate through list and check if Label is "Duck"
                // If Lable found, check left, break
                index = 0;
                for (Recognition recognition_item : recognitions) {
                    recognition = recognition_item;
                    // Display info.
                    displayInfo(index, recognition_item);

                    telemetry.addData("Label", recognition.getLabel());
                    // Check if label is Duck
                    if (recognition.getLabel().equals("Duck")) {
                        left = Double.parseDouble(JavaUtil.formatNumber(recognition.getLeft(), 0));
                        telemetry.addData("Left", left);
                        if (left < DUCK_LEFT_THRESHOLD) {
                            pos = 1;
                        } else {
                            pos = 2;
                        }
                        foundDuck = true;
                        break;
                    }
                    // Increment index.
                    index = index + 1;
                }
                if (foundDuck == false) {
                    pos = 3;
                }
            }
            //sleep(10);
            telemetry.update();
        }



        return (pos);
    }


    @Override
    public void update(TelemetryPacket packet) {

    }
}
