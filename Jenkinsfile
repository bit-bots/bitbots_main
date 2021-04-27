// vim: set filetype=groovy:

def images = [
    "jenkins": "registry.bit-bots.de/jenkins",
    "bitbots_builder": "registry.bit-bots.de/bitbots_builder"
]

pipeline {
    agent {
        kubernetes {
            yaml """
kind: Pod
spec:
  containers:
    - name: podman
      image: quay.io/podman/stable
      tty: true
      securityContext:
        privileged: true
      command:
      - cat
"""
        }
    }
    triggers {
      cron 'H 2 * * *'          // build periodically every day at 2 am
    }
    stages {
        stage("Update containers") {
            steps {
                script {
                    println "env ${env}"
                    println "env.CHANGE_ID ${env.CHANGE_ID}"
                    def parallels = [:]
                    images.each{ folder, image_name ->
                            parallels[folder] = {
                                stage("Build $folder Image") {
                                    gitStatusWrapper(
                                        credentialsId: "github-credentials",
                                        description: "building $folder container",
                                        failureDescription: "could not build $folder container",
                                        successDescription: "$folder container successfully built",
                                        gitHubContext: "$folder") {
                                        dir(folder) {
                                            container("podman") {
                                                sh "podman build -t $image_name ."
                                            }
                                        }
                                    }
                                }
                                if (env.CHANGE_ID == null) {
                                    stage("Upload $folder Image") {
                                        dir(folder) {
                                            container("podman") {
                                                sh "podman push $image_name"
                                            }
                                        }
                                    }
                                }
                            }
                    }

                    parallel parallels
                }
            }
        }
    }
}
