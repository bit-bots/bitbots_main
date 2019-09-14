@Library('bitbots_jenkins_library@implement_first_version')_

pipeline {
    agent any

	triggers {
		cron 'H 6 * * * '
	}

    stages {
        stage('Build docker container') {
            steps {
                sh 'docker build -t bitbots_builder --no-cache docker_builder'
                sh 'docker tag bitbots_builder registry.bit-bots.de:5000/bitbots_builder'
                sh 'docker push registry.bit-bots.de:5000/bitbots_builder'
            }
        }

        stage('Build packages') {
            agent { docker image: 'bitbots_builder', registryUrl: 'http://registry.bit-bots.de:5000', alwaysPull: true }
            steps {
                linkCatkinWorkspace()
                sh 'rosdep update'
                sh 'rosdep install -iry --from-paths /catkin_ws/src'
                catkinBuild()
            }
        }

        stage('Document') {
            agent { docker image: 'bitbots_builder', registryUrl: 'http://registry.bit-bots.de:5000', alwaysPull: true }
            steps {
                linkCatkinWorkspace()
                catkinBuild("Documentation")
                archiveArtifacts artifacts: '**/docs/_out/**', onlyIfSuccessful: true
            }
        }
    }

    post {
        cleanup {
            sh 'docker container prune -f'
            sh 'docker image prune -f'
        }
    }
}
