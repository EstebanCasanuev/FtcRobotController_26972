package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.graphics.Color;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

public class SensorColorSubsystem extends SubsystemBase {


    RevColorSensorV3 colorSensor;
    NormalizedRGBA myNormalizedColors;
    int myColor;

    float hue;
    float saturation;
    float value;

    public SensorColorSubsystem() {
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");

        telemetry.addData("Color Sensor Enabled: ", colorSensor.initialize());
    }

    @Override
    public void periodic() {
        myNormalizedColors = ((NormalizedColorSensor) colorSensor).getNormalizedColors();
        myColor = myNormalizedColors.toColor();
        hue = JavaUtil.rgbToHue(Color.red(myColor), Color.green(myColor), Color.blue(myColor));
        saturation = JavaUtil.rgbToSaturation(Color.red(myColor), Color.green(myColor), Color.blue(myColor));
        value = JavaUtil.rgbToValue(Color.red(myColor), Color.green(myColor), Color.blue(myColor));
    }
    public String Color(){
        if(Float.parseFloat(JavaUtil.formatNumber(myNormalizedColors.red, 3)) > Float.parseFloat(JavaUtil.formatNumber(myNormalizedColors.blue, 3)) &&
                Float.parseFloat(JavaUtil.formatNumber(myNormalizedColors.red, 3)) > Float.parseFloat(JavaUtil.formatNumber(myNormalizedColors.green, 3))){
            return "red";
        } else if (Float.parseFloat(JavaUtil.formatNumber(myNormalizedColors.green, 3)) > Float.parseFloat(JavaUtil.formatNumber(myNormalizedColors.blue, 3)) &&
                Float.parseFloat(JavaUtil.formatNumber(myNormalizedColors.green, 3)) > Float.parseFloat(JavaUtil.formatNumber(myNormalizedColors.red, 3))) {
            return "green";
        }else if (Float.parseFloat(JavaUtil.formatNumber(myNormalizedColors.blue, 3)) > Float.parseFloat(JavaUtil.formatNumber(myNormalizedColors.green, 3)) &&
                Float.parseFloat(JavaUtil.formatNumber(myNormalizedColors.blue, 3)) > Float.parseFloat(JavaUtil.formatNumber(myNormalizedColors.red, 3))) {
            return "blue";
        }else{
            return "blank";
        }
    }
}
