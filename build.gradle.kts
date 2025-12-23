import edu.wpi.first.deployutils.deploy.artifact.FileTreeArtifact
import edu.wpi.first.gradlerio.GradleRIOPlugin
import edu.wpi.first.gradlerio.deploy.roborio.FRCJavaArtifact
import edu.wpi.first.gradlerio.deploy.roborio.RoboRIO
import edu.wpi.first.toolchain.NativePlatforms
import org.gradle.plugins.ide.idea.model.IdeaLanguageLevel
import org.jetbrains.kotlin.gradle.dsl.JvmTarget

plugins {
    java
    idea
    kotlin("jvm")
    id("com.google.devtools.ksp")
    id("edu.wpi.first.GradleRIO")
    id("com.peterabeles.gversion") version "1.10.3"
}

val javaVersion by extra(17)
val jvmVendor: JvmVendorSpec by extra { JvmVendorSpec.ADOPTIUM }

@Suppress("PropertyName")
val ROBOT_MAIN_CLASS = "com.frcteam3636.swervebase.Main"

gversion {
    srcDir = layout.buildDirectory.dir("generated/gversion/main/kotlin").get().toString()
    language = "kotlin"
    classPackage = "com.frcteam3636.version"
    dateFormat = "yyyy-MM-dd HH:mm:ss z"
    timeZone = "America/Los_Angeles" // Use preferred time zone
    indent = "    "
}

sourceSets {
    main {
        kotlin {
            srcDir(gversion.srcDir)
        }
    }
}

tasks {
    test {
        useJUnitPlatform()
        systemProperty("junit.jupiter.extensions.autodetection.enabled", "true")
    }

    compileKotlin {
        dependsOn(createVersionFile)
    }

    compileJava {
        options.encoding = Charsets.UTF_8.name()
        // Configure string concat to always inline compile
        options.compilerArgs.add("-XDstringConcat=inline")
    }

    // Setting up my Jar File. In this case, adding all libraries into the main jar ('fat jar')
    // in order to make them all available at runtime. Also adding the manifest so WPILib
    // knows where to look for our Robot Class.
    jar {
        group = "build"
        manifest(GradleRIOPlugin.javaManifest(ROBOT_MAIN_CLASS))
        duplicatesStrategy = DuplicatesStrategy.INCLUDE

        // Adding this closure makes this expression lazy, allowing GradleRIO to add
        // its dependencies before the jar task is fully configured.
        from({
            configurations
                .runtimeClasspath
                .get()
                .map { if (it.isDirectory) it else zipTree(it) }
        })
    }
}

wpi {
    with(java) {
        // Set to true to use debug for JNI.
        debugJni = false
        configureExecutableTasks(tasks.jar.get())
        configureTestTasks(tasks.test.get())
    }

    // Simulation configuration (e.g. environment variables).
    with(sim) {
        addGui().apply {
            defaultEnabled = true
        }
        addDriverstation()
    }
}

// Define my targets (RoboRIO) and artifacts (deployable files)
// This is added by GradleRIO's backing project DeployUtils.
deploy {
    targets {
        val roborio by register<RoboRIO>("roborio") {
            // Team number is loaded either from the .wpilib/wpilib_preferences.json
            // or from command line. If not found an exception will be thrown.
            // You can use project.frc.getTeamOrDefault(####) instead of project.frc.teamNumber
            // if you want to store a team number in this file.
            team = frc.teamNumber
            debug = frc.getDebugOrDefault(false)
        }

        roborio.artifacts {
            register<FRCJavaArtifact>("frcJava") {
                //jvmArgs.add("-Dcom.sun.management.jmxremote=true")
                //jvmArgs.add("-Dcom.sun.management.jmxremote.port=1198")
                //jvmArgs.add("-Dcom.sun.management.jmxremote.local.only=false")
                //jvmArgs.add("-Dcom.sun.management.jmxremote.ssl=false")
                //jvmArgs.add("-Dcom.sun.management.jmxremote.authenticate=false")
                //jvmArgs.add("-Djava.rmi.server.hostname=10.36.36.2")
                jvmArgs.add("-XX:+UnlockExperimentalVMOptions")
                jvmArgs.add("-XX:GCTimeRatio=5")
                jvmArgs.add("-XX:+UseSerialGC")
                jvmArgs.add("-XX:MaxGCPauseMillis=50")

                // Comment lines below if running on a Rio 1
                val maxJavaHeapSize = 100
                jvmArgs.add("-Xmx${maxJavaHeapSize}M")
                jvmArgs.add("-Xms${maxJavaHeapSize}M")
                jvmArgs.add("-XX:+AlwaysPreTouch")

                // Enable asserts
                jvmArgs.add("-ea")
                setJarTask(tasks.jar)
            }

            register<FileTreeArtifact>("frcStaticFileDeploy") {
                files = project.fileTree("src/main/deploy")
                directory = "/home/lvuser/deploy"
                // Change to true to delete files on roboRIO that no longer exist in deploy directory of this project
                deleteOldFiles = true
            }
        }
    }
}

dependencies {
    annotationProcessor(wpi.java.deps.wpilibAnnotations())
    implementation(wpi.java.deps.wpilib())
    implementation(wpi.java.vendor.java())

    roborioDebug(wpi.java.deps.wpilibJniDebug(NativePlatforms.roborio))
    roborioDebug(wpi.java.vendor.jniDebug(NativePlatforms.roborio))

    roborioRelease(wpi.java.deps.wpilibJniRelease(NativePlatforms.roborio))
    roborioRelease(wpi.java.vendor.jniRelease(NativePlatforms.roborio))

    nativeDebug(wpi.java.deps.wpilibJniDebug(NativePlatforms.desktop))
    nativeDebug(wpi.java.vendor.jniDebug(NativePlatforms.desktop))
    simulationDebug(wpi.sim.enableDebug())

    nativeRelease(wpi.java.deps.wpilibJniRelease(NativePlatforms.desktop))
    nativeRelease(wpi.java.vendor.jniRelease(NativePlatforms.desktop))
    simulationRelease(wpi.sim.enableRelease())

    testImplementation(platform("org.junit:junit-bom:5.11.4"))
    testImplementation("org.junit.jupiter:junit-jupiter-api")
    testImplementation("org.junit.jupiter:junit-jupiter-params")
    testRuntimeOnly("org.junit.jupiter:junit-jupiter-engine")

    implementation(project(":annotation"))
    ksp(project(":annotation"))
}

allprojects {
    plugins.apply("java")
    plugins.apply("org.jetbrains.kotlin.jvm")

    java {
        toolchain {
            languageVersion = JavaLanguageVersion.of(javaVersion)
            vendor = jvmVendor
        }
    }

    kotlin {
        compilerOptions {
            jvmTarget = JvmTarget.fromTarget(javaVersion.toString())
        }

        jvmToolchain {
            // https://kotlinlang.org/docs/gradle-configure-project.html#gradle-java-toolchains-support
            languageVersion = JavaLanguageVersion.of(javaVersion)
            vendor = jvmVendor
        }
    }

    repositories {
        mavenLocal()
        mavenCentral()
    }
}

idea {
    project {
        // The project.sourceCompatibility setting is not always picked up, so we set explicitly
        languageLevel = IdeaLanguageLevel(javaVersion)
    }
    module {
        // Improve development & (especially) debugging experience (and IDEA's capabilities) by having libraries' source & javadoc attached
        isDownloadJavadoc = true
        isDownloadSources = true
        // Exclude the .vscode directory from indexing and search
        excludeDirs.add(file(".run"))
        excludeDirs.add(file(".vscode"))
    }
}


// Set this to true to enable desktop support.
val includeDesktopSupport = true

// Helper Functions to keep syntax cleaner
// @formatter:off
fun DependencyHandler.addDependencies(configurationName: String, dependencies: List<Provider<String>>) = dependencies.forEach { add(configurationName, it) }
fun DependencyHandler.roborioDebug(dependencies: List<Provider<String>>) = addDependencies("roborioDebug", dependencies)
fun DependencyHandler.roborioRelease(dependencies: List<Provider<String>>) = addDependencies("roborioRelease", dependencies)
fun DependencyHandler.nativeDebug(dependencies: List<Provider<String>>) = addDependencies("nativeDebug", dependencies)
fun DependencyHandler.simulationDebug(dependencies: List<Provider<String>>) = addDependencies("simulationDebug", dependencies)
fun DependencyHandler.nativeRelease(dependencies: List<Provider<String>>) = addDependencies("nativeRelease", dependencies)
fun DependencyHandler.simulationRelease(dependencies: List<Provider<String>>) = addDependencies("simulationRelease", dependencies)
fun DependencyHandler.implementation(dependencies: List<Provider<String>>) = dependencies.forEach{ implementation(it) }
fun DependencyHandler.annotationProcessor(dependencies: List<Provider<String>>) = dependencies.forEach{ annotationProcessor(it) }
// @formatter:on
