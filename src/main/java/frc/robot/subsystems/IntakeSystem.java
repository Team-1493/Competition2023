// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSystem extends SubsystemBase {
    final TalonFX TopConveyor;
    final TalonFX BotConveyor;
    final TalonFX FrontIntakeBar;
    final TalonFX RearIntakeBars;
    final DigitalInput irSensorL,irSensorR;
    
    private double topConveyorSpeed = 500;
    private double botConveyorSpeed = 500;
    private double topConveyorShootSpeed=700;
    private double LimelightShootSpeed = 1200;
    private double botConveyorShootSpeed1=700;
    private double botConveyorShootSpeed2=1200;
    private double botConveyorShootSpeed3=1600;
    private double frontIntakePower = 0.3;
    private double rearIntakePower = 0.2;

    private double botConveyorKf = .244;
    private double topConveyorKp = 0;
    
    private double topConveyorKf = .244;
    private double botConveyorKp = 0.012;
    private double botConveyorKi = 0.0;
    private double botConveyorKizone = 0.0;


    private double speed=0;
    
    public IntakeSystem() {
        SmartDashboard.putNumber("Top Conveyor Speed",topConveyorSpeed);
        SmartDashboard.putNumber("Bot Conveyor Speed",botConveyorSpeed);
        SmartDashboard.putNumber("Front Intake Power",frontIntakePower);
        SmartDashboard.putNumber("Rear Intake Power",rearIntakePower);
        SmartDashboard.putNumber("Top Conveyor Shoot Speed", topConveyorShootSpeed);
        SmartDashboard.putNumber("Limelight Shoot Speed", LimelightShootSpeed);
        SmartDashboard.putNumber("Bot Conveyor Shoot Speed1", botConveyorShootSpeed1);
        SmartDashboard.putNumber("Bot Conveyor Shoot Speed2", botConveyorShootSpeed2);
        SmartDashboard.putNumber("Bot Conveyor Shoot Speed3", botConveyorShootSpeed3);

        SmartDashboard.putNumber("Top Conveyor kF", topConveyorKf);
        SmartDashboard.putNumber("Top Conveyor kP", topConveyorKp);

        SmartDashboard.putNumber("Bot Conveyor kF", botConveyorKf);
        SmartDashboard.putNumber("Bot Conveyor kP", botConveyorKp);
        SmartDashboard.putNumber("Bot Conveyor kI", botConveyorKi);
        SmartDashboard.putNumber("Bot Conveyor kIzone", botConveyorKizone);




        TopConveyor = new TalonFX(12);
        BotConveyor = new TalonFX(13);
        FrontIntakeBar = new TalonFX(10);
        RearIntakeBars = new TalonFX(11);

        RearIntakeBars.configFactoryDefault();
        FrontIntakeBar.configFactoryDefault();

        TopConveyor.config_kF(0,topConveyorKf );
        BotConveyor.config_kF(0,botConveyorKf );
        TopConveyor.config_kP(0,topConveyorKp );
        BotConveyor.config_kP(0,botConveyorKp );

        TopConveyor.setInverted(InvertType.InvertMotorOutput);
        FrontIntakeBar.setInverted(InvertType.InvertMotorOutput);
        
        irSensorL = new DigitalInput(2);
        irSensorR = new DigitalInput(3);
        
    }

  /**
   * Example command factory method.
   *
   * @return a command
   */

  public void UpdateConstants() {
      topConveyorSpeed = SmartDashboard.getNumber("Top Conveyor Speed",topConveyorSpeed);
      botConveyorSpeed = SmartDashboard.getNumber("Bot Conveyor Speed",botConveyorSpeed);
      frontIntakePower = SmartDashboard.getNumber("Front Intake Power",frontIntakePower);
      rearIntakePower = SmartDashboard.getNumber("Rear Intake Power",rearIntakePower);
     
      topConveyorShootSpeed = 
          SmartDashboard.getNumber("Top Conveyor Shoot Speed", topConveyorShootSpeed);
      
      botConveyorShootSpeed1 = 
        SmartDashboard.getNumber("Shoot Speed 1", botConveyorShootSpeed1);

      botConveyorShootSpeed2 = 
        SmartDashboard.getNumber("Shoot Speed 2", botConveyorShootSpeed2);

      botConveyorShootSpeed3 = 
        SmartDashboard.getNumber("Shoot Speed 3", botConveyorShootSpeed3);

      TopConveyor.config_kF(0, 
          SmartDashboard.getNumber("Top Conveyor kF", topConveyorKf));
      
      TopConveyor.config_kP(0, 
          SmartDashboard.getNumber("Top Conveyor kP", topConveyorKp));


      botConveyorKf=SmartDashboard.getNumber("Bot Conveyor kF", botConveyorKf);
      BotConveyor.config_kF(0,botConveyorKf);

      BotConveyor.config_kP(0, 
          SmartDashboard.getNumber("Bot Conveyor kP", botConveyorKp));    

      BotConveyor.config_kI(0, 
          SmartDashboard.getNumber("Bot Conveyor kI", botConveyorKi));    

      BotConveyor.config_IntegralZone(0, 
          SmartDashboard.getNumber("Bot Conveyor kIzone", botConveyorKizone));    
      
  }

  public void runFrontIntakeBack() {

      FrontIntakeBar.set(ControlMode.PercentOutput, -frontIntakePower);
  }

  public void IntakeCube() {
      TopConveyor.set(ControlMode.Velocity, topConveyorSpeed);
      FrontIntakeBar.set(ControlMode.PercentOutput, frontIntakePower);
      RearIntakeBars.set(ControlMode.PercentOutput, rearIntakePower);
  }

  public void runFrontIntakeBar() {
    FrontIntakeBar.set(ControlMode.PercentOutput, frontIntakePower);
}


  public double getCurrent(){
    return Math.max(FrontIntakeBar.getStatorCurrent(),RearIntakeBars.getStatorCurrent()); 
  }
  public void GrabCone() {
      FrontIntakeBar.set(ControlMode.PercentOutput, frontIntakePower);
      RearIntakeBars.set(ControlMode.PercentOutput, -rearIntakePower);
  }
  public void DropCone() {
      FrontIntakeBar.set(ControlMode.PercentOutput, -frontIntakePower);
      RearIntakeBars.set(ControlMode.PercentOutput, rearIntakePower);
  }

  public void topConveyorShootCube(){
    TopConveyor.set(ControlMode.Velocity, topConveyorShootSpeed);
  }

  public boolean botConveyorAtShootSpeed(){
    return 600*BotConveyor.getSelectedSensorVelocity()/2048>=0.9*speed;

  }

  public void botConveyorShootCube(int speedLevel){
    if (speedLevel==0){
      speed = SmartDashboard.getNumber("Limelight Shoot Speed", LimelightShootSpeed);
    }
    else if(speedLevel==1)speed=botConveyorShootSpeed1;
    else if(speedLevel==2)speed=botConveyorShootSpeed2;
    else speed=botConveyorShootSpeed3;
    
    BotConveyor.set(ControlMode.Velocity, speed);
  }

  public double getBottomConveyorSpeed(){
    return BotConveyor.getSelectedSensorVelocity();
  }

  public double getTopConveyorSpeed(){
    return TopConveyor.getSelectedSensorVelocity();
  }

  public double getBottomConveyorCLE(){
    return BotConveyor.getClosedLoopError();
  }



  public void StopMotors(){
    TopConveyor.set(ControlMode.PercentOutput, 0);
    BotConveyor.set(ControlMode.PercentOutput, 0);
    FrontIntakeBar.set(ControlMode.PercentOutput, 0);
    RearIntakeBars.set(ControlMode.PercentOutput, 0);
    }

  public boolean HasCube(){
    return (!irSensorL.get() || !irSensorR.get());
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
