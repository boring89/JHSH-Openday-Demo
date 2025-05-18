// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.Drivetain.SwerveSubsystem;
import frc.robot.subsystems.Mechanism.Elevator;
import frc.robot.subsystems.Mechanism.Shooter;
import frc.robot.subsystems.Vision.Limelight;



public class RobotContainer {

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final Limelight limelight = new Limelight();
  private final Elevator elevator = new Elevator();
  private final Shooter shooter = new Shooter(elevator);

  private final Joystick m_Joystick = new Joystick(OIConstants.kDriverControllerPort);
  public RobotContainer() {

    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
      swerveSubsystem, 
      () -> -m_Joystick.getRawAxis(OIConstants.kDriverYAxis), 
      () -> m_Joystick.getRawAxis(OIConstants.kDriverXAxis), 
      () -> m_Joystick.getRawAxis(OIConstants.kDriverRotAxis), 
      () -> true));

    configureButtonBindings();
  }


  private void configureButtonBindings() {
    new POVButton(m_Joystick, 0).whileTrue(new SwerveJoystickCmd(
      swerveSubsystem, 
      () -> -limelight.yOut(), 
      () -> limelight.xOut(), 
      () -> limelight.rotOut(), 
      () -> false));

    new JoystickButton(m_Joystick, 1).whileTrue(
      new InstantCommand(() -> elevator.changeLevel()));
    
    new JoystickButton(m_Joystick, 3).whileTrue(
      new InstantCommand(() -> shooter.shoot()))
    .whileFalse(new InstantCommand(() -> shooter.stop()));
  }

  public Command getAutonomousCommand(){
    return null;
  }
}

