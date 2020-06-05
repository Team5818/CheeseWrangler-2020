import com.techshroom.inciseblue.commonLib
import org.rivierarobotics.gradlerioredux.artifactsKt
import org.rivierarobotics.gradlerioredux.tasks.PathWeaverSourceSetExtension

plugins {
    id("org.rivierarobotics.gradlerioredux") version "0.7.8"
}

gradleRioRedux {
    robotClass = "org.rivierarobotics.robot.Robot"
    teamNumber = 5818
    pathWeaverProjectProperty.set(project.layout.projectDirectory.dir("PathWeaver"))
}

val pathWeaver = sourceSets.main.get().extensions.getByType<PathWeaverSourceSetExtension>().pathWeaver
pathWeaver.srcDir(file("PathWeaver/Paths"))

afterEvaluate {
    deploy {
        artifactsKt {
            fileTreeArtifact("frcPathsDeploy") {
                files.set(fileTree(pathWeaver.destinationDirectory))
                targets.add("roboRio")
                directory = "/home/lvuser/deploy/paths"
            }
        }
    }
}

val includeDesktopSupport = true
val platform = wpi.platforms.javaClass.getDeclaredField("desktop").get(null) as String

tasks.register("launchSimulation") {
    doLast {
        project.exec{
            workingDir = file("./build/")
            val fileType = if (platform.contains("windows")) ".bat" else ".sh"
            commandLine("cmd", "/C", "start", "gradlerio_simulateJava$fileType")
        }
    }
}
tasks.getByName("simulateJava").finalizedBy(tasks.getByName("launchSimulation"))

dependencies {
    for (depJni: String in (wpi.deps.wpilibJni(platform) + wpi.deps.vendor.jni(platform))) {
        nativeDesktopZip(depJni)
    }
    simulation("edu.wpi.first.halsim:halsim_gui:${wpi.wpilibVersion}:$platform@zip")

    implementation("org.rivierarobotics.apparjacktus:apparjacktus:0.1.1")
    commonLib("net.octyl.apt-creator", "apt-creator", "0.1.4") {
        compileOnly(lib("annotations"))
        annotationProcessor(lib("processor"))
    }
    commonLib("com.google.dagger", "dagger", "2.25.4") {
        implementation(lib())
        annotationProcessor(lib("compiler"))
    }
}

repositories {
    jcenter()
}