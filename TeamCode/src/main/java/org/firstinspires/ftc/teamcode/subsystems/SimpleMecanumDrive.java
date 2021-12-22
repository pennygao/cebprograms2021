 package org.firstinspires.ftc.teamcode.subsystems;

 import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
 import com.acmerobotics.roadrunner.control.PIDCoefficients;
 import com.acmerobotics.roadrunner.control.PIDFController;
 import com.acmerobotics.roadrunner.geometry.Pose2d;
 import com.qualcomm.hardware.bosch.BNO055IMU;
 import com.qualcomm.robotcore.hardware.DcMotor;
 import com.qualcomm.robotcore.hardware.DcMotorEx;
 import com.qualcomm.robotcore.hardware.DcMotorSimple;
 import com.qualcomm.robotcore.hardware.DistanceSensor;

 import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
 import org.firstinspires.ftc.teamcode.hardware.CachingSensor;
 import org.firstinspires.ftc.teamcode.robot.Robot;
 import org.firstinspires.ftc.teamcode.robot.Subsystem;

 public class SimpleMecanumDrive implements Subsystem {
     public double powerFactor = 0.5;


    private int MOTOR_LF= 0;
    private int MOTOR_LR= 3;
    private int MOTOR_RR =2;
    private int MOTOR_RF= 1;
            
    private DcMotorEx[] motors = new DcMotorEx[4];
    private CachingSensor<Float> headingSensor;

    private DistanceSensor distL;// = robot.get//hardwareMap.get(DistanceSensor.class, "distanceL");
    private DistanceSensor distR; // = hardwareMap.get(DistanceSensor.class, "distanceR");

     //PID Stuff
     private PIDFController hubPID;
     private PIDFController turnPID;
     private static final PIDCoefficients HUB_PID_COEFFICIENTS = new PIDCoefficients(0.0006, 0.000, 0.0001);

     private static final double HUB_ACCEPTABLE_ERROR_MARGIN = 20.0;
     private static final PIDCoefficients TURN_PID_COEFFICIENTS = new PIDCoefficients(0.008, 0.0, 0.1);

     private static final double TURN_ACCEPTABLE_ERROR_MARGIN = 0.15;

     private boolean inAlignMode = false;
     private double  targetDist = 0.0;



    private Double[] powers = {0.0, 0.0, 0.0, 0.0};

    public SimpleMecanumDrive (Robot robot) {
        motors[0] = robot.getMotor("DriveLF");
        motors[1] = robot.getMotor("DriveLR");
        motors[2] = robot.getMotor("DriveRR");
        motors[3] = robot.getMotor("DriveRF");



        BNO055IMU imu = robot.getIMU("imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        headingSensor = new CachingSensor<>(() -> imu.getAngularOrientation().firstAngle);
        robot.addListener(headingSensor);
        //scale_n = robot.getAnalogSensor("NInput");
        //scale_p = robot.getAnalogSensor("PInput");

        motors[0].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[1].setDirection(DcMotorSimple.Direction.REVERSE);
        for (DcMotorEx motor:motors){
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // distance related
        distL = robot.getDistanceSensor("distanceL");
        distR = robot.getDistanceSensor("distanceR");
        hubPID = new PIDFController(HUB_PID_COEFFICIENTS);
        hubPID.setOutputBounds(-0.5, 0.5);

        turnPID = new PIDFController(TURN_PID_COEFFICIENTS);
        turnPID.setOutputBounds(-0.1, 0.1);
    }

    public void setDrivePower(Pose2d drivePower) {
        powers[0] = drivePower.getX() - drivePower.getY() - drivePower.getHeading();
        powers[1] = drivePower.getX() + drivePower.getY() - drivePower.getHeading();

        powers[2] = drivePower.getX() - drivePower.getY() + drivePower.getHeading();
        powers[3] = drivePower.getX() + drivePower.getY() + drivePower.getHeading();
    }
    public void setPowerFactor(double factor){
        this.powerFactor=factor;
    }

    public double MMtoticks(double distance){
        return 0.0;
    }
    public void driveForDistance(int leftFrontDistance, int rightFrontDistance, int leftBackDistance, int rightBackDistance, double power){
        for (int i = 0; i<4 ; i++){
            motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public double getHeading() {
        return headingSensor.getValue();
    }

    // implement getDistL(), return distance in mm
     public double getdistL() {return distL.getDistance(DistanceUnit.MM);}

    // implement getDistR(), return distance in mm
    public double getdistR() {return distR.getDistance(DistanceUnit.MM);}

    public void setTargetDist(double targetDist) {
        hubPID.reset();
        turnPID.reset();
        hubPID.setTargetPosition(targetDist);
        //hubPID.setTargetVelocity(0.0);
        //hubPID.setTargetAcceleration(0.0);
        turnPID.setTargetPosition(0.0);
    }

    public void setInAlignMode(boolean inAlignMode) {
        this.inAlignMode = inAlignMode;
    }

    public boolean getInAlignMode() { return this.inAlignMode;}

     private void moveToHub() {
         double hubPower; // = hubPID.update((getdistL()+ getdistR())/2);
         double turnPower;
         //hubPID.reset();

         hubPower = hubPID.update((getdistL()+ getdistR())/2);
         turnPower = turnPID.update(Math.atan((getdistL() - getdistR())/152.4));
         setDrivePower(new Pose2d (hubPower, 0, turnPower));
     }

     public boolean hubReached() {
         return ((Math.abs(getDistPIDError())) <= HUB_ACCEPTABLE_ERROR_MARGIN);

     }
     public boolean turnReached() {
         return ((Math.abs(getTurnPIDError())) <= TURN_ACCEPTABLE_ERROR_MARGIN);
     }

     public double getDistPIDError() { return hubPID.getLastError(); }
     public double getTurnPIDError() { return turnPID.getLastError(); }

    @Override
    public void update(TelemetryPacket packet) {
        if (inAlignMode) {
            moveToHub();
        }

        for (int i = 0; i < 4; i++){
            motors[i].setPower(powers[i] * this.powerFactor);
        }



        //packet.put("Scale N", scale_n.readRawVoltage());
        //packet.put("Scale P", scale_p.readRawVoltage());
    }


}
