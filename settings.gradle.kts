pluginManagement {
    val frcYear: String by settings
    val kspVersion: String by settings
    val kotlinVersion: String by settings
    val wpilibVersion: String by settings

    plugins {
        kotlin("jvm") version kotlinVersion
        id("com.google.devtools.ksp") version kspVersion
        id("edu.wpi.first.GradleRIO") version wpilibVersion
    }

    repositories {
        mavenLocal()
        gradlePluginPortal()

        val frcHome = if (System.getProperty("os.name").contains("windows", ignoreCase = true)) {
            file(System.getenv("PUBLIC") ?: """C:\Users\Public""")
        } else {
            file(System.getProperty("user.home"))
        }
            .resolve("wpilib")
            .resolve(frcYear)

        maven {
            name = "frcHome"
            url = uri(frcHome.resolve("maven"))
        }
    }
}

include(":annotation")

System.getProperties().apply {
    setProperty("org.gradle.internal.native.headers.unresolved.dependencies.ignore", "true")
}

plugins {
    id("org.gradle.toolchains.foojay-resolver-convention") version ("0.9.0")
}
