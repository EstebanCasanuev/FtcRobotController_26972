package org.firstinspires.ftc.teamcode.Subsystems;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Constants;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

public class ArmSubsystem extends SubsystemBase {

    private final Motor ArmMotor = new Motor(hardwareMap, "ArmMotor");
    private final DcMotor Motor = ArmMotor.motor;

    private final Motor.Encoder Encoder = ArmMotor.encoder;

    public PIDController PID;
    public ArmSubsystem(){

        PID = new PIDController(
                Constants.PIDConstants.kP,
                Constants.PIDConstants.kI,
                Constants.PIDConstants.kD);

    }

    @Override
    public void periodic() {
        Motor.setPower(PID.calculate(Encoder.getPosition()));
        super.periodic();
    }

    public void Setpos(IntSupplier Setpoint){
        PID.setSetPoint(Setpoint.getAsInt());
    }
}
