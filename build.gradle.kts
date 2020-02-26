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
