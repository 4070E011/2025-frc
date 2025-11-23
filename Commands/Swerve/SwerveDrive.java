package frc.robot.Commands.Swerve;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Swerve;

public class SwerveDrive extends Command {
  // Supplier 是一種「供應者」，讓我們可以隨時獲取最新的搖桿數值
  private Supplier<Double> xSupplier, ySupplier, turnSupplier;
  private Swerve swerve;

  // 建構子
  public SwerveDrive(Supplier<Double> xSupplier, Supplier<Double> ySupplier, Supplier<Double> turnSupplier, Swerve swerve) {
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.turnSupplier = turnSupplier;
    this.swerve = swerve;
    
    // 宣告這個指令會佔用 Swerve 子系統，避免其他指令同時控制底盤
    addRequirements(swerve);
  }

  // 每個週期 (20ms) 執行一次
  @Override
  public void execute() {
    // 從 Supplier 獲取當下數值，並傳入 subsystem 的 drive 函式
    swerve.drive(xSupplier.get(), ySupplier.get(), turnSupplier.get(), true);
  }
}