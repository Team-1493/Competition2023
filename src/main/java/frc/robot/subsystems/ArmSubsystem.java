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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    TalonFX armMotor = new TalonFX(9);
    CANCoder enc = new CANCoder(19);
    DigitalInput limitUpper = new DigitalInput(0);
    DigitalInput limitLower = new DigitalInput(1);
    // Gains for position control
    double armkP = 1.7, armkD = 80, armkI = 0.0012, armkIZone=70, armkF = 0;
    // feedforward accounts for gravity, friction and spring
    // 3 sets of gains, depending on where arm is relative to neutral position -69 deg
    // region 1, -69 to 0 deg   
    // region 2, -69 to -90 deg
    // region 3, >0 
    double armkG = 0.0721, armkS = -0.0138, armkV = 0, armkA = 0;
    double armkG2 = 0.0729, armkS2 = -0.0424, armkV2 = 0, armkA2 = 0;
    double armkG3 = 0.0721, armkS3 = -0.0138, armkV3 = 0, armkA3 = 0;
    double armForwardSensorLim = 2600, armReverseSensorLim = 1140;
    double armMaxOutput = .15;
    public double posStow=1110,posStowFinish=1090;
    public double posCubeIntake=1275,posConeGrab=2200,posConePlace=2300;
    public double angle, angleCounts;
    double angleFromHorizontalDeg, angleFromHorizontalCounts, angleFromHorizontalRad;
    double cosAngleFromHorizontal;
    double angleOffsetDeg = 186.3;
    double angleOffsetCounts = angleOffsetDeg * 4096 / 360.;
    double angleRate, angleRatePrev = 0, angleRate2;
    double time = 0, timePrev = 0;
    int button = 0;
    double stick=0;
    boolean ls_upper = true, ls_lower = false, ls_upperActive = false;
    double arbff;
    boolean active = false;

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
    StatorCurrentLimitConfiguration currentConfig = 
        new StatorCurrentLimitConfiguration(true, 30, 
        32, .1);
    armMotor.configStatorCurrentLimit(currentConfig);

        armMotor.setStatusFramePeriod(StatusFrame.Status_10_Targets, 5);
        armMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 5);

        SmartDashboard.putNumber("arm kP", armkP);
        SmartDashboard.putNumber("arm kI", armkI);
        SmartDashboard.putNumber("arm kD", armkD);
        SmartDashboard.putNumber("arm kF", armkF);
        SmartDashboard.putNumber("arm kIzone", armkIZone);

        SmartDashboard.putNumber("arm Pos Stow", posStow);
        SmartDashboard.putNumber("arm Pos StowFinish", posStowFinish);
        SmartDashboard.putNumber("arm Pos Cube", posCubeIntake);
        SmartDashboard.putNumber("arm Pos Cone Grab", posConeGrab);
        SmartDashboard.putNumber("arm Pos Cone Place", posConePlace);

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

        SmartDashboard.putNumber("arm ForSensorLim", armForwardSensorLim);
        SmartDashboard.putNumber("arm RevSensorLim", armReverseSensorLim);
        SmartDashboard.putNumber("arm MaxOutput", armMaxOutput);
        updateConstants();

    }

    @Override
    public void periodic() {
        armMotor.getSelectedSensorPosition();
        angle = enc.getPosition();
        angleCounts = armMotor.getSelectedSensorPosition(); 
        angleFromHorizontalDeg = angle - angleOffsetDeg;

        ls_upper = limitUpper.get();
        ls_lower = limitLower.get();
        isUpperLimitActive(stick);

        SmartDashboard.putNumber("arm voltage", armMotor.getMotorOutputVoltage());
        SmartDashboard.putNumber("arm current", armMotor.getStatorCurrent());
        SmartDashboard.putNumber("arm Pos Counts", angleCounts);
        SmartDashboard.putNumber("arm Pos DegFromHor", angleFromHorizontalDeg);
        SmartDashboard.putNumber("arm CLE", armMotor.getClosedLoopError());
        SmartDashboard.putBoolean("arm Upper LS", ls_upper);
        SmartDashboard.putBoolean("arm Lower LS", ls_lower);
    }


    public void StopMotors() {
        armMotor.set(ControlMode.PercentOutput, 0);
    }

    public void resetIntegralAccumulator(){
        armMotor.setIntegralAccumulator(0, 0, 20);
    }


    public void setPositionInCounts(double counts) {    
        stick=0;          
        calculateFeedfwd();
        if (active) 
            armMotor.set(ControlMode.Position, counts, DemandType.ArbitraryFeedForward, arbff);
        else armMotor.set(ControlMode.PercentOutput, 0);
    }

    public void setPositionStick(double setPoint) {
        stick=setPoint;              
        calculateFeedfwd();
        if (active) 
            armMotor.set(ControlMode.Position, setPoint, DemandType.ArbitraryFeedForward, arbff);
        else armMotor.set(ControlMode.PercentOutput, 0);
    }


    public double getCounts() {
        return angleCounts;
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

        armMotor.config_kP(0, armkP);
        armMotor.config_kI(0, armkI);
        armMotor.config_kD(0, armkD);
        armMotor.config_kF(0, armkF);
        armMotor.config_IntegralZone(0, armkIZone);

        armForwardSensorLim = SmartDashboard.getNumber("arm ForSensorLim", 0);
        armReverseSensorLim = SmartDashboard.getNumber("arm RevSensorLim", 0);
        armMotor.configForwardSoftLimitThreshold(armForwardSensorLim);
        armMotor.configReverseSoftLimitThreshold(armReverseSensorLim);

        armMaxOutput = SmartDashboard.getNumber("arm MaxOutput", 0);
        armMotor.configClosedLoopPeakOutput(0, armMaxOutput);
        armMotor.configPeakOutputForward(armMaxOutput);
        armMotor.configPeakOutputReverse(-armMaxOutput);

        posStow=SmartDashboard.getNumber("arm Pos Stow", posStow);
        posStowFinish=SmartDashboard.getNumber("arm Pos StowFinish", posStowFinish);
        posCubeIntake=SmartDashboard.getNumber("arm Pos Cube", posCubeIntake);
        posConeGrab=SmartDashboard.getNumber("arm Pos Cone Grab", posConeGrab);
        posConePlace=SmartDashboard.getNumber("arm Pos Cone Place", posConePlace);s
    }

    public CommandBase UpdateConstants() {
        return runOnce(() -> {
            updateConstants();
        });
    }

    public void isUpperLimitActive(double stick) {
        if (ls_upper)
            ls_upperActive = true;
        else if (ls_upperActive && stick > 0)
            ls_upperActive = true;
        else
            ls_upperActive = false;

        active=!ls_upperActive;
    }

    public void calculateFeedfwd(){
        angleFromHorizontalRad = angleFromHorizontalDeg * Math.PI / 180.;
        if (angleFromHorizontalDeg < -69)
            arbff = +armkG2 * Math.cos(angleFromHorizontalRad) + armkS2;
        else if (angleFromHorizontalDeg >= -69 && angleFromHorizontalDeg < 0)
            arbff = +armkG * Math.cos(angleFromHorizontalRad) + armkS;
        else
            arbff = +armkG3 * Math.cos(angleFromHorizontalRad) + armkS3;
    }


}
