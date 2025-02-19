package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.AutonomousSetpoint.AUTONOMOUS_FIRSTSWING_SETPOINT;
import static org.firstinspires.ftc.teamcode.Constants.AutonomousSetpoint.AUTONOMOUS_SECONDSWING_SETPOINT;
import static org.firstinspires.ftc.teamcode.Constants.AutonomousSetpoint.AUTONOMOUS_SWINGTAKEIN_SETPOINT;
import static org.firstinspires.ftc.teamcode.Constants.Drivetrain_PIDContants.PidX_D;
import static org.firstinspires.ftc.teamcode.Constants.Drivetrain_PIDContants.PidX_I;
import static org.firstinspires.ftc.teamcode.Constants.Drivetrain_PIDContants.PidX_P;
import static org.firstinspires.ftc.teamcode.Constants.Drivetrain_PIDContants.PidY_D;
import static org.firstinspires.ftc.teamcode.Constants.Drivetrain_PIDContants.PidY_I;
import static org.firstinspires.ftc.teamcode.Constants.Drivetrain_PIDContants.PidY_P;
import static org.firstinspires.ftc.teamcode.Constants.Drivetrain_PIDContants.PidZ_D;
import static org.firstinspires.ftc.teamcode.Constants.Drivetrain_PIDContants.PidZ_I;
import static org.firstinspires.ftc.teamcode.Constants.Drivetrain_PIDContants.PidZ_P;
import static org.firstinspires.ftc.teamcode.Constants.OdometryConstants.CENTER_WHEEL_OFFSET;
import static org.firstinspires.ftc.teamcode.Constants.OdometryConstants.TRACKWIDTH;
import static org.firstinspires.ftc.teamcode.Constants.SlideSetpoints.ALL_IN;
import static org.firstinspires.ftc.teamcode.Constants.SlideSetpoints.IN_THE_MIDDLE;
import static org.firstinspires.ftc.teamcode.Constants.Slide_PIDConstants.D_SLIDEMOTION;
import static org.firstinspires.ftc.teamcode.Constants.Slide_PIDConstants.I_SLIDEMOTION;
import static org.firstinspires.ftc.teamcode.Constants.Slide_PIDConstants.P_SLIDEMOTION;
import static org.firstinspires.ftc.teamcode.Constants.SwingSetpoints.THREE_QUARTERS;
import static org.firstinspires.ftc.teamcode.Constants.Swing_PIDConstants.D_SWINGMOTION;
import static org.firstinspires.ftc.teamcode.Constants.Swing_PIDConstants.I_SWINGMOTION;
import static org.firstinspires.ftc.teamcode.Constants.Swing_PIDConstants.P_SWINGMOTION;
import static org.firstinspires.ftc.teamcode.OdometryRead.DISTANCE_PER_PULSE;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Objects;

@Autonomous(name = "OneSpecimen")
public class OneSpecimen extends LinearOpMode {

    private MotorEx frontRightMotor, rearRightMotor, frontLeftMotor, rearLeftMotor;
    private MotorEx SwingSlider, SlideMotion;

    private CRServo Intake;

    MecanumDrive m_Drive;
    private HolonomicOdometry odometry;

    ElapsedTime Millis = new ElapsedTime();

    PIDController PidX;
    PIDController PidY;
    PIDController PidZ;

    PIDController SlidePID;
    PIDController SwingSlidePID;

    RevColorSensorV3 colorSensor;
    NormalizedRGBA myNormalizedColors;

    //private MotorEx frontRightMotor, rearRightMotor, frontLeftMotor, rearLeftMotor;
    /*private MotorEx rightSlider, leftSlider;
    private MotorEx SlideMotion;*/


    //telemetryPacket packet = new telemetryPacket();


    //dashboard.sendtelemetryPacket(packet);
    //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    //OpenCvWebcam camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);


    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry telemetry = dashboard.getTelemetry();

    int myColor;

    String ActualColor;
    float hue;
    float saturation;
    float value;

    double xPos;
    double yPos;
    double zAngle;

    int autonomous = -1;

    @Override
    public void runOpMode() throws InterruptedException {

        colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");


        frontLeftMotor = new MotorEx(hardwareMap, "frontLeftMotor");
        frontRightMotor = new MotorEx(hardwareMap, "frontRightMotor");
        rearRightMotor = new MotorEx(hardwareMap, "rearRightMotor");
        rearLeftMotor = new MotorEx(hardwareMap, "rearLeftMotor");


        m_Drive = new MecanumDrive(frontLeftMotor, frontRightMotor, rearLeftMotor, rearRightMotor);

        PidX = new PIDController(PidX_P, PidX_I, PidX_D);
        PidY = new PIDController(PidY_P, PidY_I, PidY_D);
        PidZ = new PIDController(PidZ_P, PidZ_I, PidZ_D);

        SlideMotion = new MotorEx(hardwareMap, "SlideMotion");
        SwingSlider = new MotorEx(hardwareMap, "Swing");

        SlidePID = new PIDController(P_SLIDEMOTION, I_SLIDEMOTION, D_SLIDEMOTION);
        SwingSlidePID = new PIDController(P_SWINGMOTION, I_SWINGMOTION, D_SWINGMOTION);


        rearRightMotor.setInverted(true);
        frontLeftMotor.setInverted(true);

        rearLeftMotor.setDistancePerPulse(DISTANCE_PER_PULSE);
        frontRightMotor.setDistancePerPulse(DISTANCE_PER_PULSE);
        rearRightMotor.setDistancePerPulse(DISTANCE_PER_PULSE);

        frontRightMotor.resetEncoder();
        rearLeftMotor.resetEncoder();
        rearRightMotor.resetEncoder();

        SwingSlider.resetEncoder();
        SlideMotion.resetEncoder();


        Intake = new CRServo(hardwareMap, "IntakeServo");

        odometry = new HolonomicOdometry(
                ()->-rearLeftMotor.getDistance(),
                ()->-frontRightMotor.getDistance(),
                ()->rearRightMotor.getDistance(),
                TRACKWIDTH,
                CENTER_WHEEL_OFFSET
        );

        odometry.updatePose(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));


        telemetry.addData("Robot Position at Init: ", odometry.getPose());

        telemetry.update();


        setBot_Setpoint(0, 0, 0);
        SwingSlidePID.setSetPoint(700);

        waitForStart();


        while (opModeIsActive()) {

            odometry.updatePose();

            xPos = odometry.getPose().getY();
            yPos = odometry.getPose().getX();
            zAngle = getAngle(odometry.getPose().getRotation().getDegrees());

        myNormalizedColors = ((NormalizedColorSensor) colorSensor).getNormalizedColors();
        myColor = myNormalizedColors.toColor();
        hue = JavaUtil.rgbToHue(Color.red(myColor), Color.green(myColor), Color.blue(myColor));
        saturation = JavaUtil.rgbToSaturation(Color.red(myColor), Color.green(myColor), Color.blue(myColor));
        value = JavaUtil.rgbToValue(Color.red(myColor), Color.green(myColor), Color.blue(myColor));

        if (red() > blue() && red() > green()) {
            ActualColor = "red";
        } else if (green() > blue() && green() > red()) {
            ActualColor = "green";
        } else if (blue() > green() && blue() > red()) {
            ActualColor = "blue";
        }
        telemetry.addData("X:" , xPos);
        telemetry.addData("Y:", yPos);
        telemetry.addData("Angulo:", zAngle);
        telemetry.addData("X Setpoint:", PidX.getSetPoint());
        telemetry.addData("Y Setpoint:", PidY.getSetPoint());
        telemetry.addData("Z Setpoint:", PidZ.getSetPoint());
        telemetry.addData("Swing Encoder: ", SwingSlider.getCurrentPosition());
        telemetry.addData("Swing Power: ", SwingSlidePID.calculate(SwingSlider.getCurrentPosition()));
        telemetry.addData("Swing Setpoint: ", SwingSlidePID.getSetPoint());
        telemetry.addData("Slide Encoder: ", SlideMotion.getCurrentPosition());
        telemetry.addData("Slide Setpoint: ", SlidePID.getSetPoint());
        telemetry.addData("Stage", autonomous);
        telemetry.update();


            switch(autonomous){
                case 0:
                    setBot_Setpoint(-23, 29, 0);
                    break;
                case 1:
                    setSwing_Setpoint(AUTONOMOUS_FIRSTSWING_SETPOINT);
                    break;
                case 2:
                    setSlider_Setpoint(IN_THE_MIDDLE);
                    break;
                case 3:
                    SwingSlidePID.setSetPoint(AUTONOMOUS_SECONDSWING_SETPOINT);
                    setSlider_Setpoint(ALL_IN);
                    break;
                case 4:
                    setSwing_Setpoint(AUTONOMOUS_SWINGTAKEIN_SETPOINT);
                    break;
                case 5:
                    setBot_Setpoint(6, 19, 0);
                    break;
                case 6:
                    setBot_Setpoint(10, 31, 0);
                    break;
                case 7:
                    setBot_Setpoint(10, 50, 0);
                    break;
                case 8:
                    setBot_Setpoint(23, 46, 0);
                    break;
                case 9:
                    setBot_Setpoint(23, 10, 0);
                    break;
                case 10:
                    setBot_Setpoint(31, 20, 0);
                    break;
                case 11:
                    setBot_Setpoint(31, 20, 90);
                    if(Millis.milliseconds() > 1500){
                        autonomous += 1;
                    }
                    break;
                case 12:
                    setBot_Setpoint(31, 20, 170);
                    break;
                case 13:
                    setBot_Setpoint(30, 10, 170);
                    break;
                case 14:
                    setBot_Setpoint(30, 0.5, 170);
                    Intake.set(-1);
                    break;
                case 15:
                    setBot_Setpoint(30, 0.2, 170);
                    SlidePID.setSetPoint(50);
                    Intake.set(-1);
                    break;
                case 16:
                    //turnIntake(true);
                    turnIntake();
                    break;
                case 17:
                    Intake.set(0);
                    setSwing_Setpoint(THREE_QUARTERS);
                    break;
                case 18:
                    setBot_Setpoint(-15, 15, 90);
                    break;
                case 19:
                    setBot_Setpoint(-9, 10, 0);
                    break;
                case 20:
                    setBot_Setpoint(-25, 26.5, 0);
                    break;
                case 21:
                    setSwing_Setpoint(AUTONOMOUS_FIRSTSWING_SETPOINT);
                    break;
                case 22:
                    setSlider_Setpoint(IN_THE_MIDDLE);
                    break;
                case 23:
                    SwingSlidePID.setSetPoint(AUTONOMOUS_SECONDSWING_SETPOINT);
                    setSlider_Setpoint(ALL_IN);
                    break;
                case 24:
                    setSwing_Setpoint(100);
                    break;
                case 25:
                    setBot_Setpoint(20, 10, 0);
                    break;
                case 26:
                    setSwing_Setpoint(1);
                default:
                    m_Drive.driveRobotCentric(0, 0, 0);
                    break;
            }

            double YPower = PidY.calculate(odometry.getPose().getX());
            double XPower = PidX.calculate(odometry.getPose().getY());
            double ZPower = PidZ.calculate(getAngle(odometry.getPose().getRotation().getDegrees()));

            if(PidZ.getSetPoint() == 90){
                m_Drive.driveRobotCentric(
                        -PidY.calculate(odometry.getPose().getX())*0.6,
                        PidX.calculate(odometry.getPose().getY())*0.6,
                        PidZ.calculate(getAngle(odometry.getPose().getRotation().getDegrees()))*0.5
                );

            } else if(PidZ.getSetPoint() == 170 || PidZ.getSetPoint() == 180){
                m_Drive.driveRobotCentric(
                        -PidX.calculate(odometry.getPose().getY()) * 0.7,
                        -PidY.calculate(odometry.getPose().getX()) * 0.4,
                        PidZ.calculate(getAngle(odometry.getPose().getRotation().getDegrees())) * 0.5
                );

                PidY.setPID(0.15, PidY_I, PidY_D);
            } else if(PidY.getSetPoint() < 32){
                m_Drive.driveRobotCentric(
                        PidX.calculate(odometry.getPose().getY()) * 0.3,
                        PidY.calculate(odometry.getPose().getX()) * 0.6,
                        PidZ.calculate(getAngle(odometry.getPose().getRotation().getDegrees())) * 0.5
                );
            }else {
                m_Drive.driveRobotCentric(
                        PidX.calculate(odometry.getPose().getY()) * 0.6,
                        PidY.calculate(odometry.getPose().getX()) * 0.6,
                        PidZ.calculate(getAngle(odometry.getPose().getRotation().getDegrees())) * 0.5
                );
            }

            if(SwingSlidePID.getSetPoint() < 500 && SwingSlider.getCurrentPosition() > 500){
                SwingSlidePID.setPID(0.0001, I_SWINGMOTION, D_SWINGMOTION);
            }else{
                SwingSlidePID.setPID(P_SWINGMOTION, I_SWINGMOTION, D_SWINGMOTION);

            }

            SlideMotion.set(SlidePID.calculate(SlideMotion.getCurrentPosition()));
            SwingSlider.set(-SwingSlidePID.calculate(SwingSlider.getCurrentPosition()) * 0.7);


        }

    }

    public void setBot_Setpoint(double X, double Y, double Z){

        PidX.setSetPoint(X);
        PidY.setSetPoint(Y);
        PidZ.setSetPoint(Z);

        if(atSetpoint(X, Y, Z)){
            autonomous += 1;
        }
    }

    public boolean atSetpoint(double X_Setpoint, double Y_Setpoint, double Z_Setpoint){
        return xPos > X_Setpoint - 1.7
                && xPos < X_Setpoint + 1.7
                && yPos > Y_Setpoint - 1.7
                && yPos < Y_Setpoint + 1.7
                && zAngle < Z_Setpoint + 3.5
                && zAngle > Z_Setpoint - 3.5;

    }
    public void setSwing_Setpoint(double Setpoint){
        SwingSlidePID.setSetPoint(Setpoint);
        if(atSetpoint(Setpoint, SwingSlider.getCurrentPosition())){
            autonomous += 1;
        }
    }

    public void turnIntake(){
        Intake.set(-1);

        if(Objects.equals(ActualColor, "blue")){
        autonomous += 1;
        }
    }

    public void setSlider_Setpoint(double Setpoint){
        SlidePID.setSetPoint(Setpoint);
        if(atSetpoint(Setpoint, SlideMotion.getCurrentPosition())){
            autonomous += 1;
        }
    }


    public boolean atSetpoint(double Setpoint, double input){
        return input > Setpoint - 50
                && input <+
                Setpoint + 50;
    }

    public double getAngle(double ActualAngle) {
        return (ActualAngle+90) - 360 * Math.floor((ActualAngle+90)/360) - 90;
    }

    public float red(){
        return Float.parseFloat(JavaUtil.formatNumber(myNormalizedColors.red, 3));
    }
    public float green(){
        return Float.parseFloat(JavaUtil.formatNumber(myNormalizedColors.green, 3));
    }
    public float blue(){
        return Float.parseFloat(JavaUtil.formatNumber(myNormalizedColors.blue, 3));
    }

}