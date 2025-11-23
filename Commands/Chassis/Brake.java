package frc.robot.Commands.Chassis;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ChassisSubsystem;

public class Brake extends Command {
  private ChassisSubsystem chassisSubsystem;

  public Brake(ChassisSubsystem chassisSubsystem) {
    this.chassisSubsystem = chassisSubsystem;
    addRequirements(chassisSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // 強制將所有馬達輸出設為 0
    chassisSubsystem.drive(0, 0);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}