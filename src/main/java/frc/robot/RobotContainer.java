// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Drive.SwerveJoystickCmd;
import frc.robot.Drive.SwerveSubsystem;

public class RobotContainer {

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController addOnssController = new CommandXboxController(1);

  private final SendableChooser<Command> autoChooser;
  
    public RobotContainer() {
  
      // Build an auto chooser. This will use Commands.none() as the default option.
      autoChooser = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    SmartDashboard.putData("Auto Chooser", autoChooser);

    swerveSubsystem.setDefaultCommand(
      new SwerveJoystickCmd(
        swerveSubsystem
        , () -> driverController.getLeftX()
        , () -> driverController.getLeftY()
        , () -> driverController.getRightX()
        , null
        )
    );

    configureBindings();
  }

  private void configureBindings() {

    driverController.y().onTrue(new InstantCommand( ()-> swerveSubsystem.ZeroHeading() ));

  }

  public void mResetEncoders(){

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
