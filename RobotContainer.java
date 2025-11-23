package frc.robot;
import javax.naming.InitialContext;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.Swerve.SwerveDrive;
import frc.robot.Commands.Swerve.ZeroSwerveHeading;
import frc.robot.Constanats.DriverConstanats;
import frc.robot.Subsystems.Swerve;

public class RobotContainer {
  // 實例化 Swerve 子系統
  private Swerve swerve = new Swerve();

  // 實例化搖桿 (Port 0)
  private Joystick js = new Joystick(0);

  public RobotContainer() {
    setupDefaultCommands(); // 設定預設指令
    configureBindings();    // 設定按鈕綁定
    
  }
  
  private void setupDefaultCommands() {
    // 設定 Swerve 的預設指令：只要沒有其他指令在使用底盤，就會執行這個
    swerve.setDefaultCommand(
      new SwerveDrive(
        // 使用 Lambda 表達式 ()->{...} 動態獲取搖桿數值
        ()->{return deadband(js.getRawAxis(1) * -1.0 * DriverConstanats.kDriveSpeed, 0.05);}, // Y軸 (前後)，乘 -1 是因為搖桿往前是負值
        ()->{return deadband(js.getRawAxis(0) * -1.0  * DriverConstanats.kDriveSpeed, 0.05);} ,       // X軸 (左右)
        ()->{return deadband(js.getRawAxis(4) * DriverConstanats.kTurnSpeed, 0.05);},         // 右邊X軸 (旋轉)
        swerve // 傳入子系統
      )
    );
  } 
 
  private void configureBindings() {
    // 在這裡綁定按鈕功能，例如：
    //  new JoystickButton(js, 1).onTrue(new SomeCommand());
    new JoystickButton(js, 1).onTrue(new ZeroSwerveHeading(swerve)) ;
   }

  // 自動階段的指令
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  private double deadband(double value, double range) {
    if (value < range && value > -range) return 0;
    else return value;
  }
}