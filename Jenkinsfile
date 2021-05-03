@Library('bitbots_jenkins_library') import de.bitbots.jenkins.*;

defineProperties()

def pipeline = new BitbotsPipeline(this, env, currentBuild, scm)
pipeline.configurePipelineForPackage(new PackagePipelineSettings(new PackageDefinition("humanoid_league_game_controller")))
pipeline.configurePipelineForPackage(new PackagePipelineSettings(new PackageDefinition("humanoid_league_speaker")))
pipeline.configurePipelineForPackage(new PackagePipelineSettings(new PackageDefinition("humanoid_league_team_communication")))
pipeline.configurePipelineForPackage(new PackagePipelineSettings(new PackageDefinition("humanoid_league_transform")))
pipeline.execute()