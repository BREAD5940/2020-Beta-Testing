package frc.robot

import edu.wpi.first.wpilibj.geometry.Translation2d
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics
import org.ghrobotics.lib.mathematics.units.inMeters
import org.ghrobotics.lib.mathematics.units.inches

object Constants {

    val kBumperThickness = 3.25.inches
    val baseWidth = 24.inches // TODO check
    val baseLen = 24.inches // TODO check

    /** Module speeds as (fl, fr, br, bl) */
    val kModulePositions = listOf(
            Translation2d(baseWidth.inMeters() / 2.0, baseLen.inMeters() / 2.0),
            Translation2d(baseWidth.inMeters() / 2.0, -baseLen.inMeters() / 2.0),
            Translation2d(-baseWidth.inMeters() / 2.0, -baseLen.inMeters() / 2.0),
            Translation2d(-baseWidth.inMeters() / 2.0, baseLen.inMeters() / 2.0)
    )

    val kinematics = SwerveDriveKinematics(
            kModulePositions[0],
            kModulePositions[1],
            kModulePositions[2],
            kModulePositions[3]
    )

}
