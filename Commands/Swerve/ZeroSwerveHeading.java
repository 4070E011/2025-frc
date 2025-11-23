package frc.robot.Commands.Swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Swerve;

public class ZeroSwerveHeading extends Command {
    private final Swerve swerve;

    // 建構子：指令需要知道要操作哪個 Swerve 子系統
    public ZeroSwerveHeading(Swerve swerve) {
        this.swerve = swerve;
        // 宣告這個指令會佔用 Swerve 子系統
        addRequirements(swerve);
    }

    // 指令開始執行時 (按鈕按下時)
    @Override
    public void initialize() {
        // 【關鍵】呼叫子系統中的陀螺儀歸零函式
        swerve.zeroHeading();
    }

    // 這是瞬時指令 (執行一次就結束)，所以直接回傳 true
    @Override
    public boolean isFinished() {
        return true;
    }
}