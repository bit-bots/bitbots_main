@Library('bitbots_jenkins_library') import de.bitbots.jenkins.*;

defineProperties()

def pipeline = new BitbotsPipeline(this, env, currentBuild, scm)
pipeline.configurePipelineForPackage(new PackagePipelineSettings(new PackageDefinition("system_monitor")))
pipeline.configurePipelineForPackage(new PackagePipelineSettings(new PackageDefinition("bitbots_time_constraint")))
pipeline.configurePipelineForPackage(new PackagePipelineSettings(new PackageDefinition("bitbots_live_tool_rqt")))
pipeline.configurePipelineForPackage(new PackagePipelineSettings(new PackageDefinition("bitbots_ceiling_cam")).withoutDocumentation().withoutPublishing())
pipeline.configurePipelineForPackage(new PackagePipelineSettings(new PackageDefinition("bitbots_bringup")))
pipeline.execute()
