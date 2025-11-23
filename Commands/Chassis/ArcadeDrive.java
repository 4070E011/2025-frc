package frc.robot.Commands.Chassis;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ChassisSubsystem;

public class ArcadeDrive extends Command {
  private ChassisSubsystem chassisSubsystem;
  private Supplier<Double> xSupplier, ySupplier;

  // 建構子：接收控制變數與子系統
  public ArcadeDrive(Supplier<Double> xSupplier, Supplier<Double> ySupplier, ChassisSubsystem chassisSubsystem) {
    this.chassisSubsystem = chassisSubsystem;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    addRequirements(chassisSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // 呼叫傳統底盤的 drive 方法
    chassisSubsystem.drive(xSupplier.get(), ySupplier.get()); 
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false; // 駕駛指令通常不會自動結束，直到被中斷
  }
}