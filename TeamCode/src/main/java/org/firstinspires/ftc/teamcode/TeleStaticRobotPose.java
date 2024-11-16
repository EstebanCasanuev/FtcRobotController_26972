package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.DashboardCore;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "OdometryTest")
public class TeleStaticRobotPose extends LinearOpMode {

    private MotorEx frontRightMotor, rearRightMotor, frontLeftMotor, rearLeftMotor;
    MecanumDrive m_Drive;
    private HolonomicOdometry odometry;

    PIDController PidX;
    PIDController PidY;
    PIDController PidZ;

    TelemetryPacket packet = new TelemetryPacket();
    FtcDashboard dashboard = FtcDashboard.getInstance();

    //dashboard.sendTelemetryPacket(packet);
    //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    //OpenCvWebcam camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);


    Telemetry dashboardTelemetry = dashboard.getTelemetry();



    public static final double TRACKWIDTH = 18;
    public static final double CENTER_WHEEL_OFFSET = 4;
    public static final double WHEEL_DIAMETER = 1.25;
    // if needed, one can add a gearing term here
    public static final double TICKS_PER_REV = 2000;
    public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;

    @Override
    public void runOpMode() throws InterruptedException {

    FtcDashboard.getInstance().startCameraStream(camera, 0);



        packet.fieldOverlay()
                .setFill("blue")
                .fillRect(-20, -20, 40, 40);


        frontLeftMotor = new MotorEx(hardwareMap, "frontLeftMotor");
        frontRightMotor = new MotorEx(hardwareMap, "frontRightMotor");
        rearRightMotor = new MotorEx(hardwareMap, "rearRightMotor");
        rearLeftMotor = new MotorEx(hardwareMap, "rearLeftMotor");

        m_Drive = new MecanumDrive(frontLeftMotor, frontRightMotor, rearLeftMotor, rearRightMotor);

        PidX = new PIDController(0.1, 0, 0);
        PidY = new PIDController(0.1, 0, 0);
        PidZ = new PIDController(0.1, 0, 0);


        frontLeftMotor.setDistancePerPulse(DISTANCE_PER_PULSE);
        frontRightMotor.setDistancePerPulse(DISTANCE_PER_PULSE);
        rearRightMotor.setDistancePerPulse(DISTANCE_PER_PULSE);

        frontRightMotor.resetEncoder();
        frontLeftMotor.resetEncoder();
        rearRightMotor.resetEncoder();



        odometry = new HolonomicOdometry(
                frontLeftMotor::getDistance,
                frontRightMotor::getDistance,
                rearRightMotor::getDistance,
                TRACKWIDTH,
                CENTER_WHEEL_OFFSET
        );

        // read the current position from the position tracker
        odometry.updatePose(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));


        dashboardTelemetry.addData("Robot Position at Init: ", odometry.getPose());

        dashboardTelemetry.update();


        setBot_Setpoint(0, 0, 0);

        waitForStart();

        while (opModeIsActive()) {
            // teleop things

            // update position
            odometry.updatePose();
            dashboardTelemetry.addData("X:", odometry.getPose().getX());
            dashboardTelemetry.addData("Y:", odometry.getPose().getY());
            dashboardTelemetry.addData("Angulo:", odometry.getPose().getRotation().getDegrees());
            dashboardTelemetry.addData("X Setpoint:", PidX.getSetPoint());
            dashboardTelemetry.addData("Y Setpoint:", PidY.getSetPoint());
            dashboardTelemetry.addData("Z Setpoint:", PidZ.getSetPoint());

            dashboardTelemetry.update();

            if (gamepad1.a){
                setBot_Setpoint(0, 24, 0);
            } else if (gamepad1.b) {
                setBot_Setpoint(0, 0, 0);
            }

            /*m_Drive.driveRobotCentric(
                    PidY.calculate(odometry.getPose().getX()),
                    PidX.calculate(odometry.getPose().getY()),
                    -PidZ.calculate(odometry.getPose().getRotation().getDegrees())
            );*/

            /*m_Drive.driveRobotCentric(
                    -gamepad1.left_stick_x,
                    gamepad1.left_stick_y,
                    -gamepad1.right_stick_x
            );*/

            if(odometry.getPose().getX() < 25){
                m_Drive.driveRobotCentric(
                        0,
                        0.2,
                        0
                );

            }else if(odometry.getPose().getX() > 25){
                m_Drive.driveRobotCentric(
                        0,
                        -0.2,
                        0
                );
            }else{
                m_Drive.driveRobotCentric(
                        0,
                        0,
                        0
                );
            }

        }
    }

    public void setBot_Setpoint(double X, double Y, double Z){
        PidX.setSetPoint(Y);
        PidY.setSetPoint(X);
        PidZ.setSetPoint(Z);
    }

}
