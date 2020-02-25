import com.techshroom.inciseblue.commonLib
import org.rivierarobotics.gradlerioredux.artifactsKt

plugins {
    id("org.rivierarobotics.gradlerioredux") version "0.7.6"
}

gradleRioRedux {
    robotClass = "org.rivierarobotics.robot.Robot"
    teamNumber = 5818
}

afterEvaluate {
    deploy {
        artifactsKt {
            fileTreeArtifact("frcStaticFileDeploy") {
                files.set(fileTree("PathWeaver/output"))
                targets.add("roboRio")
                directory = "/home/lvuser/deploy/paths"
            }
        }
    }
}

dependencies {
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
