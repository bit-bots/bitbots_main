@Library('bitbots_jenkins_library')_

pipeline {
    agent any

	triggers {
		cron 'H 6 * * * '
	}

    stages {
        stage('Build docker container') {
            steps {
                sh 'docker build -t registry.bit-bots.de:5000/bitbots_builder --no-cache docker_builder'
                sh 'docker push registry.bit-bots.de:5000/bitbots_builder'
                sh 'docker image prune -f'
                sh 'docker container prune -f'
            }
        }

        stage('Document') {
            agent { docker image: 'registry.bit-bots.de:5000/bitbots_builder', registryUrl: 'http://registry.bit-bots.de:5000', alwaysPull: true, args: '--volume /srv/shared_catkin_install_space:/srv/catkin_install' }
            steps {
                linkCatkinWorkspace()
                catkinBuild("Documentation")

                stash includes: 'bitbots_docs/docs/_out/**', name: 'docs_output'
                publishHTML([allowMissing: false, alwaysLinkToLastBuild: false, keepAll: false, reportDir: 'bitbots_docs/docs/_out/',
                            reportFiles: 'index.html', reportName: 'Built Documentation', reportTitles: ''])
            }
        }

		stage('Build') {
            agent { docker image: 'registry.bit-bots.de:5000/bitbots_builder', registryUrl: 'http://registry.bit-bots.de:5000', alwaysPull: true, args: '--volume /srv/shared_catkin_install_space:/srv/catkin_install' }
            steps {
                linkCatkinWorkspace()
                catkinBuild()

				sh 'cp -r /catkin_ws/install/bitbots_docs ./installed'
				stash name: 'bitbots_docs_installed', includes: 'installed/**'
				cleanWs()
            }
		}

		stage('Install') {
            agent { docker image: 'registry.bit-bots.de:5000/bitbots_builder', registryUrl: 'http://registry.bit-bots.de:5000', alwaysPull: true, args: '--volume /srv/shared_catkin_install_space:/srv/catkin_install' }
			steps {
				lock('shared_catkin_install_space') {
					unstash 'bitbots_docs_installed'
					sh 'cp -r installed/lib /srv/catkin_install'
					sh 'cp -r installed/share /srv/catkin_install'
					cleanWs()
				}
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
