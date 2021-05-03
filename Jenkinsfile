@Library('bitbots_jenkins_library') import de.bitbots.jenkins.*;

defineProperties()

def pipeline = new BitbotsPipeline(this, env, currentBuild, scm)
pipeline.configurePipelineForPackage(new PackagePipelineSettings(new PackageDefinition("bitbots_buttons")))
pipeline.configurePipelineForPackage(new PackagePipelineSettings(new PackageDefinition("bitbots_hardware_rqt")).withoutDocumentation())
pipeline.execute()
