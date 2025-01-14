package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.OdometryConstants.CENTER_WHEEL_OFFSET;
import static org.firstinspires.ftc.teamcode.Constants.OdometryConstants.TICKS_PER_REV;
import static org.firstinspires.ftc.teamcode.Constants.OdometryConstants.TRACKWIDTH;
import static org.firstinspires.ftc.teamcode.Constants.SlideSetpoints.ALL_IN;
import static org.firstinspires.ftc.teamcode.Constants.SlideSetpoints.ALL_THE_WAY;
import static org.firstinspires.ftc.teamcode.Constants.SlideSetpoints.IN_THE_MIDDLE;
import static org.firstinspires.ftc.teamcode.Constants.Slide_PIDConstants.D_SLIDEMOTION;
import static org.firstinspires.ftc.teamcode.Constants.Slide_PIDConstants.I_SLIDEMOTION;
import static org.firstinspires.ftc.teamcode.Constants.Slide_PIDConstants.P_SLIDEMOTION;
import static org.firstinspires.ftc.teamcode.Constants.SwingSetpoints.DOWN;
import static org.firstinspires.ftc.teamcode.Constants.SwingSetpoints.MIDDLE;
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
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.SensorColor;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

@Config
@TeleOp(name = "Main")
public class Main extends LinearOpMode {
    
    private HolonomicOdometry odometry;

    private MotorEx frontRightMotor, rearRightMotor, frontLeftMotor, rearLeftMotor;
    private MotorEx rightSlider, SlideMotion;

    private CRServo Intake;


    RevColorSensorV3 colorSensor;
        NormalizedRGBA myNormalizedColors;
        int myColor;

        String ActualColor;
        float hue;
        float saturation;
        float value;

    PIDController SlidePID = new PIDController(P_SLIDEMOTION, I_SLIDEMOTION, D_SLIDEMOTION);
    PIDController SwingSlidePID = new PIDController(P_SWINGMOTION, I_SWINGMOTION, D_SWINGMOTION);

    MecanumDrive m_Drive;
    @Override
    public void runOpMode() {

        //colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");

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

        rightSlider = new MotorEx(hardwareMap, "Swing");

        Intake = new CRServo(hardwareMap, "IntakeServo");

        SlideMotion = new MotorEx(hardwareMap, "SlideMotion");

        rightSlider.resetEncoder();
        SlideMotion.resetEncoder();

        //odometry.updatePose(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

        //telemetry.addData("Robot Position at Init: ", odometry.getPose());

        //telemetry.addData("Color Sensor Enabled: ", colorSensor.initialize());
        telemetry.update();

        waitForStart();
        if (opModeIsActive()) {

            while (opModeIsActive()) {
                //odometry.updatePose();

                m_Drive.driveRobotCentric(
                        gamepad1.left_stick_x,
                        -gamepad1.left_stick_y,
                        gamepad1.right_stick_x
                );

                rightSlider.set(-SwingSlidePID.calculate(rightSlider.getCurrentPosition()) * 0.7);
                telemetry.addData("Swing Encoder: ", rightSlider.getCurrentPosition());
                telemetry.addData("Swing Power: ", SwingSlidePID.calculate(rightSlider.getCurrentPosition()));
                telemetry.addData("Swing Setpoint: ", SwingSlidePID.getSetPoint());



                if(SwingSlidePID.getSetPoint() < 500 && rightSlider.getCurrentPosition() > 500){
                    SwingSlidePID.setPID(DOWN_P_SWINGMOTION, I_SWINGMOTION, D_SWINGMOTION);
                }else{
                    SwingSlidePID.setPID(P_SWINGMOTION, I_SWINGMOTION, D_SWINGMOTION);

                }


                if (gamepad2.dpad_down) {
                    SlideMotion.set(gamepad2.right_trigger - gamepad2.left_trigger);
                } else {
                    SlideMotion.set(SlidePID.calculate(SlideMotion.getCurrentPosition()));
                }



                if (gamepad2.dpad_left){
                    SwingSlidePID.setSetPoint(MIDDLE);
                }else if(gamepad2.dpad_up){
                    SwingSlidePID.setSetPoint(UP);
                }else if(gamepad2.dpad_right){
                    SwingSlidePID.setSetPoint(DOWN);
                }else if (gamepad2.a) {
                    SlidePID.setSetPoint(IN_THE_MIDDLE);
                    //SwingSlidePID.setSetPoint(0);
                } else if (gamepad2.b) {
                    SlidePID.setSetPoint(ALL_IN);
                    //SwingSlidePID.setSetPoint(500);
                } else if (gamepad2.x) {
                    //SwingSlidePID.setSetPoint(8000);
                    SlidePID.setSetPoint(ALL_THE_WAY);
                }

                if (gamepad2.left_stick_button){
                    SwingSlidePID.setSetPoint(map(gamepad2.left_stick_y, -1, 1, 200, 1500));
                }

                int Intake_Power = (gamepad2.right_bumper  ? 1 : 0) - (gamepad2.left_bumper  ? 1 : 0);

                Intake.set(Intake_Power);


                telemetry.addData("Slide Encoder: ", SlideMotion.getCurrentPosition());
                telemetry.addData("Slide Setpoint: ", SlidePID.getSetPoint());


                /*myNormalizedColors = ((NormalizedColorSensor) colorSensor).getNormalizedColors();
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
                telemetry.addData("Color: ", ActualColor);*/



                /*telemetry.addData("X:", odometry.getPose().getX());
                telemetry.addData("Y:", odometry.getPose().getY());
                telemetry.addData("Angulo:", odometry.getPose().getRotation());*/
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
