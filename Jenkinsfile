@Library('bitbots_jenkins_library')_

pipeline {
    agent any

	triggers {
		cron 'H 6 * * * '
	}

    stages {
        stage('Build docker container') {
            when { branch 'master' }
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

                stash includes: '**/docs/_out/**', name: 'docs_output'
                archiveArtifacts artifacts: '**/docs/_out/**', onlyIfSuccessful: true
            }
        }

        stage('Deploy') {
            agent { label 'webserver' }
            when { branch 'master' }
            steps {
                unstash 'docs_output'
                deployDocs('bitbots_docs')
            }
        }
    }
}
