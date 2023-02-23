// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    TalonFX armMotor = new TalonFX(9);
    CANCoder enc = new CANCoder(19);
    DigitalInput limitUpper = new DigitalInput(0);
    DigitalInput limitLower = new DigitalInput(1);
    double absPos, relPos;
    double armkP = 0, armkD = 0, armkI = 0, armkIZone, armkF = 2;
    double armkPdown = 0.15, armkDdown = 10, armkIdown = 2, armkIZonedown, armkFdown = 0;
    // double armkG = 0.05, armkS = 0.005, armkV = 0, armkA = 0;
    double armkG = 0.0721, armkS = -0.0138, armkV = 0, armkA = 0;
    double armkG2 = 0.0729, armkS2 = -0.0424, armkV2 = 0, armkA2 = 0;
    double armkG3 = 0.0721, armkS3 = -0.0138, armkV3 = 0, armkA3 = 0;
    double armPos1 = 1170, armPos2 = 1170, armPos3 = 2081;
    double armMMVel = 120, armMMAcc = 100;
    double armMMVeldown = 30, armMMAccdown = 40;
    double armForwardSensorLim = 2600, armReverseSensorLim = 1090;
    double armMaxOutput = .4;

    double angle, angleCounts;
    double angleFromHorizontalDeg, angleFromHorizontalCounts, angleFromHorizontalRad;
    double cosAngleFromHorizontal;
    double angleOffsetDeg = 186.3;
    double angleOffsetCounts = angleOffsetDeg * 4096 / 360.;
    double angleRate, angleRatePrev = 0, angleRate2;
    double voltage;
    double time = 0, timePrev = 0;
    int button = 0;
    double stick;
    boolean logging = false, loggingPrev = false;
    boolean ls_upper = true, ls_lower = false, ls_upperActive = false;

    public ArmSubsystem() {

        enc.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        enc.setPositionToAbsolute(20);
        enc.configSensorDirection(true);
        enc.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 5, 20);

        armMotor.configFactoryDefault();

        // set voltage compensation
        armMotor.configVoltageCompSaturation(11.5);
        armMotor.enableVoltageCompensation(true);

        // Set encoder values as limit switch
        armMotor.configForwardSoftLimitThreshold(armForwardSensorLim);
        armMotor.configReverseSoftLimitThreshold(armReverseSensorLim);
        armMotor.configForwardSoftLimitEnable(true);
        armMotor.configReverseSoftLimitEnable(true);

        // set to brake mode
        armMotor.setNeutralMode(NeutralMode.Brake);

        // set feedbak sensor to cancoder
        armMotor.configRemoteFeedbackFilter(enc, 0);
        armMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.RemoteSensor0, 0, 20);

        // set max closed loop output
        armMotor.configClosedLoopPeakOutput(0, armMaxOutput);
        armMotor.configPeakOutputForward(armMaxOutput);
        armMotor.configPeakOutputReverse(-armMaxOutput);

        // set current limit
        StatorCurrentLimitConfiguration currentConfig = new StatorCurrentLimitConfiguration(true, 20,
                25, 1);
        // armMotor.configGetStatorCurrentLimit(currentConfig) ;

        armMotor.setStatusFramePeriod(StatusFrame.Status_10_Targets, 5);
        armMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 5);

        SmartDashboard.putNumber("arm kP", armkP);
        SmartDashboard.putNumber("arm kI", armkI);
        SmartDashboard.putNumber("arm kD", armkD);
        SmartDashboard.putNumber("arm kF", armkF);
        SmartDashboard.putNumber("arm kIzone", armkIZone);

        SmartDashboard.putNumber("arm kP down", armkPdown);
        SmartDashboard.putNumber("arm kI down ", armkIdown);
        SmartDashboard.putNumber("arm kD down", armkDdown);
        SmartDashboard.putNumber("arm kF down", armkFdown);
        SmartDashboard.putNumber("arm kIzone down", armkIZonedown);

        SmartDashboard.putNumber("arm kG", armkG);
        SmartDashboard.putNumber("arm kS", armkS);
        SmartDashboard.putNumber("arm kV", armkV);
        SmartDashboard.putNumber("arm kA", armkA);

        SmartDashboard.putNumber("arm kG2", armkG2);
        SmartDashboard.putNumber("arm kS2", armkS2);
        SmartDashboard.putNumber("arm kV2", armkV2);
        SmartDashboard.putNumber("arm kA2", armkA2);

        SmartDashboard.putNumber("arm kG3", armkG3);
        SmartDashboard.putNumber("arm kS3", armkS3);
        SmartDashboard.putNumber("arm kV3", armkV3);
        SmartDashboard.putNumber("arm kA3", armkA3);

        SmartDashboard.putNumber("arm stow pos", armPos1);
        SmartDashboard.putNumber("arm intake pos", armPos2);
        SmartDashboard.putNumber("arm cone pick pos", armPos3);
        SmartDashboard.putNumber("arm MMvel", armMMVel);
        SmartDashboard.putNumber("arm MMacc", armMMAcc);
        SmartDashboard.putNumber("arm MMveldown", armMMVeldown);
        SmartDashboard.putNumber("arm MMaccdown", armMMAccdown);
        SmartDashboard.putNumber("arm ForSensorLim", armForwardSensorLim);
        SmartDashboard.putNumber("arm RevSensorLim", armReverseSensorLim);
        SmartDashboard.putNumber("arm MaxOutput", armMaxOutput);

        SmartDashboard.putBoolean("logging", false);

        updateConstants();

    }

    public void setTgtPositionInCounts(double counts) {
        armMotor.setIntegralAccumulator(0, 0, 20);

        double arbff;
        if (angleFromHorizontalDeg < -69)
            arbff = +armkG2 * Math.cos(angleFromHorizontalRad) + armkS2;
        else if (angleFromHorizontalDeg >= -69 && angleFromHorizontalDeg < 0)
            arbff = +armkG * Math.cos(angleFromHorizontalRad) + armkS;
        else
            arbff = +armkG3 * Math.cos(angleFromHorizontalRad) + armkS3;

        armMotor.set(ControlMode.MotionMagic, armPos2, DemandType.ArbitraryFeedForward, arbff);

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void updateConstants() {
        armkG = SmartDashboard.getNumber("arm kG", 0);
        armkS = SmartDashboard.getNumber("arm kS", 0);

        armkG2 = SmartDashboard.getNumber("arm kG2`", 0);
        armkS2 = SmartDashboard.getNumber("arm kS2", 0);

        armkG3 = SmartDashboard.getNumber("arm kG3`", 0);
        armkS3 = SmartDashboard.getNumber("arm kS3", 0);

        armkP = SmartDashboard.getNumber("arm kP", 0);
        armkI = SmartDashboard.getNumber("arm kI", 0);
        armkD = SmartDashboard.getNumber("arm kD", 0);
        armkF = SmartDashboard.getNumber("arm kF", 0);
        armkIZone = SmartDashboard.getNumber("arm kIzone", 0);

        armkPdown = SmartDashboard.getNumber("arm kP down", 0);
        armkIdown = SmartDashboard.getNumber("arm kI down", 0);
        armkDdown = SmartDashboard.getNumber("arm kD down", 0);
        armkFdown = SmartDashboard.getNumber("arm kF down", 0);
        armkIZonedown = SmartDashboard.getNumber("arm kIzone down", 0);

        armMotor.config_kP(0, armkP);
        armMotor.config_kI(0, armkI);
        armMotor.config_kD(0, armkD);
        armMotor.config_kF(0, armkF);
        armMotor.config_IntegralZone(0, armkIZone);

        armMotor.config_kP(1, armkPdown);
        armMotor.config_kI(1, armkIdown);
        armMotor.config_kD(1, armkDdown);
        armMotor.config_kF(1, armkFdown);
        armMotor.config_IntegralZone(1, armkIZonedown);

        armMMVel = SmartDashboard.getNumber("arm MMvel", 0);
        armMMAcc = SmartDashboard.getNumber("arm MMacc", 0);
        armMMVeldown = SmartDashboard.getNumber("arm MMveldown", 0);
        armMMAccdown = SmartDashboard.getNumber("arm MMaccdown", 0);
        armMotor.configMotionCruiseVelocity(armMMVel);
        armMotor.configMotionAcceleration(armMMAcc);

        armForwardSensorLim = SmartDashboard.getNumber("arm ForSensorLim", 0);
        armReverseSensorLim = SmartDashboard.getNumber("arm RevSensorLim", 0);
        armMotor.configForwardSoftLimitThreshold(armForwardSensorLim);
        armMotor.configReverseSoftLimitThreshold(armReverseSensorLim);

        armMaxOutput = SmartDashboard.getNumber("arm MaxOutput", 0);
        armMotor.configClosedLoopPeakOutput(0, armMaxOutput);
        armMotor.configPeakOutputForward(armMaxOutput);
        armMotor.configPeakOutputReverse(-armMaxOutput);

        armPos1 = SmartDashboard.getNumber("arm pos1", 0);
        armPos2 = SmartDashboard.getNumber("arm pos2", 0);
        armPos3 = SmartDashboard.getNumber("arm pos3", 0);

    }

    public void isUpperLimitActive(double stick) {
        if (ls_upper)
            ls_upperActive = true;
        else if (ls_upperActive && stick > 0)
            ls_upperActive = true;
        else
            ls_upperActive = false;
    }

    public static double map(double i, double b1, double t1, double b2, double t2) {
        return (i - b1) / (t1 - b1) * (t2 - b2) + b2;
    }
}
