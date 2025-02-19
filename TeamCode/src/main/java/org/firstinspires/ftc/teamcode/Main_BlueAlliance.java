package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.OdometryConstants.CENTER_WHEEL_OFFSET;
import static org.firstinspires.ftc.teamcode.Constants.OdometryConstants.TICKS_PER_REV;
import static org.firstinspires.ftc.teamcode.Constants.OdometryConstants.TRACKWIDTH;
import static org.firstinspires.ftc.teamcode.Constants.SlideSetpoints.ALL_IN;
import static org.firstinspires.ftc.teamcode.Constants.SlideSetpoints.ALL_THE_WAY;
import static org.firstinspires.ftc.teamcode.Constants.SlideSetpoints.HORIZONTAL_ALL_THE_WAY;
import static org.firstinspires.ftc.teamcode.Constants.SlideSetpoints.IN_THE_MIDDLE;
import static org.firstinspires.ftc.teamcode.Constants.Slide_PIDConstants.D_SLIDEMOTION;
import static org.firstinspires.ftc.teamcode.Constants.Slide_PIDConstants.I_SLIDEMOTION;
import static org.firstinspires.ftc.teamcode.Constants.Slide_PIDConstants.P_SLIDEMOTION;
import static org.firstinspires.ftc.teamcode.Constants.SwingSetpoints.DOWN;
import static org.firstinspires.ftc.teamcode.Constants.SwingSetpoints.THREE_QUARTERS;
import static org.firstinspires.ftc.teamcode.Constants.SwingSetpoints.UP;
import static org.firstinspires.ftc.teamcode.Constants.Swing_PIDConstants.DOWN_P_SWINGMOTION;
import static org.firstinspires.ftc.teamcode.Constants.Swing_PIDConstants.D_SWINGMOTION;
import static org.firstinspires.ftc.teamcode.Constants.Swing_PIDConstants.I_SWINGMOTION;
import static org.firstinspires.ftc.teamcode.Constants.Swing_PIDConstants.P_SWINGMOTION;
import static org.firstinspires.ftc.teamcode.OdometryRead.DISTANCE_PER_PULSE;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Objects;

@Config
@TeleOp(name = "Main_BlueAlliance")
public class Main_BlueAlliance extends LinearOpMode {

    private HolonomicOdometry odometry;

    private MotorEx frontRightMotor, rearRightMotor, frontLeftMotor, rearLeftMotor;
    private MotorEx Swing, SlideMotion;

    private CRServo Intake;

    //RevIMU imu;
    RevColorSensorV3 colorSensor;
    NormalizedRGBA myNormalizedColors;
    //DistanceSensor distanceSensor;

    DigitalChannel LimitDown;

    int myColor;

    String ActualColor;
    float hue;
    float saturation;
    float value;

    double PositiveSlideVariableSetpoint;
    double NegativeSlideVariableSetpoint;

    boolean SwingDown = false;
    boolean SampleIn;
    boolean driveOrientation = false;

    PIDController SlidePID = new PIDController(P_SLIDEMOTION, I_SLIDEMOTION, D_SLIDEMOTION);
    PIDController SwingSlidePID = new PIDController(P_SWINGMOTION, I_SWINGMOTION, D_SWINGMOTION);

    MecanumDrive m_Drive;
    @Override
    public void runOpMode() {

        colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");
        //distanceSensor = hardwareMap.get(DistanceSensor.class, "colorSensor");

        frontLeftMotor = new MotorEx(hardwareMap, "frontLeftMotor");
        frontRightMotor = new MotorEx(hardwareMap, "frontRightMotor");
        rearRightMotor = new MotorEx(hardwareMap, "rearRightMotor");
        rearLeftMotor = new MotorEx(hardwareMap, "rearLeftMotor");
        m_Drive = new MecanumDrive(frontLeftMotor, frontRightMotor, rearLeftMotor, rearRightMotor);

        rearRightMotor.setInverted(true);
        frontLeftMotor.setInverted(true);

        rearLeftMotor.setDistancePerPulse(DISTANCE_PER_PULSE);
        frontRightMotor.setDistancePerPulse(DISTANCE_PER_PULSE);
        rearRightMotor.setDistancePerPulse(DISTANCE_PER_PULSE);

        frontRightMotor.resetEncoder();
        rearLeftMotor.resetEncoder();
        rearRightMotor.resetEncoder();



        odometry = new HolonomicOdometry(
                ()->-rearLeftMotor.getDistance(),
                ()->-frontRightMotor.getDistance(),
                ()->-rearRightMotor.getDistance(),
                TRACKWIDTH,
                Math.PI * CENTER_WHEEL_OFFSET/ TICKS_PER_REV
        );

        Swing = new MotorEx(hardwareMap, "Swing");
        Intake = new CRServo(hardwareMap, "IntakeServo");
        SlideMotion = new MotorEx(hardwareMap, "SlideMotion");

        Swing.resetEncoder();
        SlideMotion.resetEncoder();

        LimitDown = hardwareMap.get(DigitalChannel.class, "LimitDown");

        //imu = new RevIMU(hardwareMap, "imu");

        //imu.init();

        //telemetry.addData("Imu Enabled", imu.getDeviceType());
        telemetry.addData("Color Sensor Enabled: ", colorSensor.initialize());
        //telemetry.addData("Distance Sensor Enabled: ", distanceSensor.getConnectionInfo());
        telemetry.update();

        waitForStart();
        if (opModeIsActive()) {

            while (opModeIsActive()) {
                //odometry.updatePose();

                /*if(driveOrientation){
                }else{
                    m_Drive.driveFieldCentric(
                            gamepad1.left_stick_x,
                            -gamepad1.left_stick_y,
                            gamepad1.right_stick_x,
                            imu.getHeading()
                    );
                }*/
                    m_Drive.driveRobotCentric(
                            gamepad1.left_stick_x,
                            -gamepad1.left_stick_y,
                            gamepad1.right_stick_x
                    );

                if(gamepad1.a){
                    driveOrientation = !driveOrientation;
                }

                if(SlideMotion.getCurrentPosition() < -1000){
                    if (gamepad2.dpad_left){
                        SwingSlidePID.setSetPoint(THREE_QUARTERS);
                    }else if(gamepad2.dpad_up){
                        SwingSlidePID.setSetPoint(UP);
                    }

                }else if(SlideMotion.getCurrentPosition() > -1000){
                    if (gamepad2.dpad_left){
                        SwingSlidePID.setSetPoint(THREE_QUARTERS);
                    }else if(gamepad2.dpad_up){
                        SwingSlidePID.setSetPoint(UP);
                    }else if(gamepad2.dpad_right){
                        SwingSlidePID.setSetPoint(DOWN);
                    }
                }

                if(Swing.getCurrentPosition() > 1000){
                    if (gamepad2.a) {
                        SlidePID.setSetPoint(IN_THE_MIDDLE);
                    } else if (gamepad2.b) {
                        SlidePID.setSetPoint(ALL_IN);
                    } else if (gamepad2.x) {
                        SlidePID.setSetPoint(ALL_THE_WAY);
                    }
                }else if(Swing.getCurrentPosition() < 1000){
                    if (gamepad2.a) {
                        SlidePID.setSetPoint(IN_THE_MIDDLE);
                    } else if (gamepad2.b) {
                        SlidePID.setSetPoint(ALL_IN);
                    } else if (gamepad2.y) {
                        SlidePID.setSetPoint(HORIZONTAL_ALL_THE_WAY);
                    }
                }


                if (gamepad2.left_stick_button){
                    SwingSlidePID.setSetPoint(map(-gamepad2.left_stick_y, -1, 1, DOWN, 700));
                }

                /*if (gamepad2.left_trigger > 0.1 || gamepad2.right_trigger > 0.1) {
                    SlidePID.setSetPoint(map(gamepad2.left_trigger - gamepad2.right_trigger, -1, 1, ALL_THE_WAY, ALL_IN));
                }*/


                PositiveSlideVariableSetpoint += gamepad2.right_trigger * 100;
                NegativeSlideVariableSetpoint += gamepad2.left_trigger * 100;

                SlidePID.setSetPoint(PositiveSlideVariableSetpoint - NegativeSlideVariableSetpoint);


                Intake.set(
                        (gamepad2.left_bumper  ? 1 : 0) -
                        (gamepad2.right_bumper  ? 1 : 0) +
                        ((Objects.equals(ActualColor, "red")) ? 1: 0) * 2
                    );

                Swing.set(-SwingSlidePID.calculate(Swing.getCurrentPosition()));
                SlideMotion.set(map(-SwingSlidePID.calculate(SlideMotion.getCurrentPosition()), -1000, 1000, -1, 1));

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

                //5telemetry.addData("IMU Heading", imu.getHeading());
                telemetry.addData("Limit Down", LimitDown.getState());
                telemetry.addData("Custom Swing Setpoint: ", map(-gamepad2.left_stick_y, -1, 1, DOWN, 1000));
                telemetry.addData("Slide Encoder: ", SlideMotion.getCurrentPosition());
                telemetry.addData("Slide Setpoint: ", SlidePID.getSetPoint());
                telemetry.addData("Color: ", ActualColor);
                telemetry.addData("Swing Encoder: ", Swing.getCurrentPosition());
                telemetry.addData("Swing Power: ", SwingSlidePID.calculate(Swing.getCurrentPosition()));
                telemetry.addData("Swing Setpoint: ", SwingSlidePID.getSetPoint());
                telemetry.update();
            }
        }
    }

    public double map(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
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
