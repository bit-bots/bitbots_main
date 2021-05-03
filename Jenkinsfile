@Library('bitbots_jenkins_library') import de.bitbots.jenkins.*;

defineProperties()

def pipeline = new BitbotsPipeline(this, env, currentBuild, scm)
pipeline.configurePipelineForPackage(new PackagePipelineSettings(new PackageDefinition("bitbots_local_planner")).withoutDocumentation())
pipeline.configurePipelineForPackage(new PackagePipelineSettings(new PackageDefinition("bitbots_localization")))
pipeline.configurePipelineForPackage(new PackagePipelineSettings(new PackageDefinition("bitbots_move_base")))
pipeline.execute()