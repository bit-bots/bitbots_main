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
        runAsUser: 1000
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
                    def parallels = [:]
                    images.each{ folder, image_name ->
                            parallels[folder] = {
                                stage("Build $folder Image") {
                                    dir(folder) {
                                        container("podman") {
                                            sh "podman build -t $image_name ."
                                        }
                                    }
                                }
                                stage("Upload $folder Image") {
                                    dir(folder) {
                                        container("podman") {
                                            sh "podman push $image_name"
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
