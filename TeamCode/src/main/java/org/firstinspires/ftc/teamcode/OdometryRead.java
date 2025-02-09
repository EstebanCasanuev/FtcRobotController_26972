package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "OdometryRead")
public class OdometryRead extends LinearOpMode {


    private MotorEx leftEncoder, rightEncoder, perpEncoder;
    private HolonomicOdometry odometry;

    public static final double TRACKWIDTH = 13.75;
    public static final double CENTER_WHEEL_OFFSET = 1.25;
    public static final double WHEEL_DIAMETER = 1.25;
    // if needed, one can add a gearing term here
    public static final double TICKS_PER_REV = 2000;
    public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;

    @Override
    public void runOpMode() throws InterruptedException {
        leftEncoder = new MotorEx(hardwareMap, "frontLeftMotor");
        rightEncoder = new MotorEx(hardwareMap, "rearRightMotor");
        perpEncoder = new MotorEx(hardwareMap, "rearLeftMotor");

        leftEncoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        rightEncoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        perpEncoder.setDistancePerPulse(DISTANCE_PER_PULSE);

        leftEncoder.resetEncoder();
        rightEncoder.resetEncoder();
        perpEncoder.resetEncoder();


        odometry = new HolonomicOdometry(
                leftEncoder::getDistance,
                rightEncoder::getDistance,
                perpEncoder::getDistance,
                TRACKWIDTH,
                CENTER_WHEEL_OFFSET
        );

        // read the current position from the position tracker
        odometry.updatePose();

        telemetry.addData("Robot Position at Init: ", odometry.getPose());
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
           double lastAngle = odometry.getPose().getRotation().getDegrees();


            // update position
            odometry.updatePose();
            telemetry.addData("X:", odometry.getPose().getX());
            telemetry.addData("Y:", odometry.getPose().getY());
            telemetry.addData("Angulo:", odometry.getPose().getRotation());
            //telemetry.addData("IMU Angulo:", Yaw);
            telemetry.addData("Repaired Angulo:", getAngle(odometry.getPose().getRotation().getDegrees()));

            telemetry.update();

        }
    }

    /*public double Angle(double Angle, double lastAngle){
        if(lastAngle > Angle && Angle < 0){
            return lastAngle + (Angle + 180);
        }else{
            return lastAngle - Angle;
        }
    }*/

    public double getAngle(double ActualAngle) {
        return (ActualAngle+90) - 360 * Math.floor((ActualAngle+90)/360) - 90;
    }

}
