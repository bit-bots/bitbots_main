@Library("bitbots_jenkins_library")_

pipeline {
    agent any

	triggers {
		cron "H 6 * * * "
	}

    stages {
        stage("Build docker container") {
            //when { branch "master" }
            steps {
                sh "docker build -t registry.bit-bots.de:5000/bitbots_builder --no-cache docker_builder"
                sh "docker push registry.bit-bots.de:5000/bitbots_builder"
                sh "docker image prune -f"
                sh "docker container prune -f"
            }
        }
		
		stage("Build package bitbots_docs") {
            agent { 
				docker {
					image "registry.bit-bots.de:5000/bitbots_builder"
					registryUrl "http://registry.bit-bots.de:5000"
					alwaysPull true
					args "--volume /srv/shared_catkin_install_space:/srv/catkin_install"
				}
			}

            steps {
                linkCatkinWorkspace("bitbots_docs")
                catkinBuild("bitbots_docs")
				catkinInstall("bitbots_docs")
				cleanWs()
            }
		}

		stage("Build main documentation") {
            agent { 
				docker {
					image "registry.bit-bots.de:5000/bitbots_builder"
					registryUrl "http://registry.bit-bots.de:5000"
					alwaysPull true
					args "--volume /srv/shared_catkin_install_space:/srv/catkin_install:ro"
				}
			}

			steps {
				linkCatkinWorkspace("bitbots_docs")
				installRosdeps("bitbots_docs")
				catkinBuild("bitbots_docs", "Documentation")
				
				stash name: "bitbots_docs_docs", includes: "bitbots_docs/docs/_out/**"
				cleanWs()
			}
		}


        stage("Deploy main documentation") {
            agent { label "webserver" }
            when { branch "master" }
            steps {
                unstash "bitbots_docs_docs"
                deployDocs("bitbots_docs")
                publishHTML([allowMissing: false, alwaysLinkToLastBuild: false, keepAll: false, reportDir: "bitbots_docs/docs/_out/",
                            reportFiles: "index.html", reportName: "Built Documentation", reportTitles: ""])
            }
        }
    }
}
