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
                // 'sometimes' the pipeline fails because sphinx is installed via python2
                sh 'sudo apt remove -y python-sphinx || true'
                catkinBuild("Documentation")

                stash includes: 'bitbots_docs/docs/_out/**', name: 'docs_output'
                publishHTML([allowMissing: false, alwaysLinkToLastBuild: false, keepAll: false, reportDir: 'bitbots_docs/docs/_out/',
                            reportFiles: 'index.html', reportName: 'Built Documentation', reportTitles: ''])
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
