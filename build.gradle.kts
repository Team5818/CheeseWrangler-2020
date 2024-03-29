import com.techshroom.inciseblue.commonLib
import org.rivierarobotics.gradlerioredux.artifactsKt
import org.rivierarobotics.gradlerioredux.tasks.PathWeaverSourceSetExtension

plugins {
    id("org.rivierarobotics.gradlerioredux") version "0.8.0"
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
                files.set(fileTree(pathWeaver.srcDirs.last()))
                targets.add("roboRio")
                directory = "/home/lvuser/deploy/paths"
            }
        }
    }
}

tasks.register<Copy>("copyPathTracerLocal") {
    from("PathWeaver/Paths")
    into("$buildDir/pathWeaver/main")
}
tasks.getByName("compilePathWeaver").finalizedBy(tasks.getByName("copyPathTracerLocal"))

// Errors with libjpeg8 (default), manually set LD_PRELOAD to libjpeg7
// https://github.com/wpilibsuite/PathWeaver/issues/246
task("PathWeaver-libjpeg7-force") {
    group = "GradleRIO"
    doLast {
        project.exec {
            environment("LD_PRELOAD", "/usr/lib/libjpeg.so.7")
            commandLine("./gradlew", "PathWeaver")
        }
    }
}

dependencies {
    simulation("edu.wpi.first.halsim:halsim_ds_socket:${wpi.wpilibVersion}:${edu.wpi.first.toolchain.NativePlatforms.desktop}@zip")
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