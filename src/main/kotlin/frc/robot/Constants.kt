package frc.robot

import edu.wpi.first.wpilibj.geometry.Translation2d
import org.ghrobotics.lib.mathematics.units.inMeters
import org.ghrobotics.lib.mathematics.units.inches

object Constants {

    val kBumperThickness = 3.25.inches
    val baseWidth = 24.inches // TODO check
    val baseLen = 24.inches // TODO check

    /** Module speeds as (fl, fr, br, bl) */
    val kModulePositions = listOf(
            Translation2d(baseWidth.inMeters() / 2.0, baseLen.inMeters() / 2.0), // fl
            Translation2d(baseWidth.inMeters() / 2.0, -baseLen.inMeters() / 2.0), // fr
            Translation2d(-baseWidth.inMeters() / 2.0, -baseLen.inMeters() / 2.0), // br
            Translation2d(-baseWidth.inMeters() / 2.0, baseLen.inMeters() / 2.0) // bl
    )

}
