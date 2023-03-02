// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Limelight extends SubsystemBase {
  private double kPshootDist = 1;
  /** Creates a new ExampleSubsystem. */
  public Limelight() {
    SmartDashboard.putNumber("kP Limelight Auto Shoot", kPshootDist);
  }
  NetworkTable Ftable = NetworkTableInstance.getDefault().getTable("limelight-front");
  NetworkTableEntry Ftx = Ftable.getEntry("tx");//X
  NetworkTableEntry Fty = Ftable.getEntry("ty");//Y
  NetworkTableEntry Fta = Ftable.getEntry("ta");//Area
  NetworkTableEntry Ftv = Ftable.getEntry("tv");//bool target seen
  NetworkTableEntry Fts = Ftable.getEntry("ts");//rotation
  NetworkTable Btable = NetworkTableInstance.getDefault().getTable("limelight-back");
  NetworkTableEntry Btx = Btable.getEntry("tx");//X
  NetworkTableEntry Bty = Btable.getEntry("ty");//Y
  NetworkTableEntry Bta = Btable.getEntry("ta");//Area
  NetworkTableEntry Btv = Btable.getEntry("tv");//bool target seen
  NetworkTableEntry Bts = Btable.getEntry("ts");//rotation
 
  public double[] getFrontLimelight() {
    double[] targetInfo = new double[5];
    targetInfo[0] = Ftv.getDouble(0.0);//Target Seen
    targetInfo[1] = Ftx.getDouble(0.0);//X
    targetInfo[2] = Fty.getDouble(0.0);//Y
    targetInfo[3] = Fta.getDouble(0.0);//Area
    targetInfo[4] = Fts.getDouble(0.0);//Rotation
    return targetInfo;
  }
  public double[] getBackLimelight() {
    double[] targetInfo = new double[5];
    targetInfo[0] = Btv.getDouble(0.0);//Target Seen
    targetInfo[1] = Btx.getDouble(0.0);//X
    targetInfo[2] = Bty.getDouble(0.0);//Y
    targetInfo[3] = Bta.getDouble(0.0);//Area
    targetInfo[4] = Bts.getDouble(0.0);//Rotation
    return targetInfo;
  }



  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Limelight Shoot Speed", kPshootDist/getFrontLimelight()[3]);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
