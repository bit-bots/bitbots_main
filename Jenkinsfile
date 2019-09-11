pipeline {
    agent any

	triggers {
		cron 'H 6 * * * '
	}

    stages {
    stage('Build') {
        steps {
            sh 'docker build -t bitbots_builder --no-cache docker_builder'
        }
    }

    stage('Publish') {
        steps {
            sh 'docker tag bitbots_builder registry.bit-bots.de:5000/bitbots_builder'
            sh 'docker push registry.bit-bots.de:5000/bitbots_builder'
        }
    }
    }

    post {
        cleanup {
            sh 'docker container prune -f'
        }
    }
}

