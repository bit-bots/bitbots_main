pipeline {
    agent none

	triggers {
		cron 'H 6 * * * '
	}

    stages {
        stage('Build documentation') {
        agent { docker image: 'bitbots_builder', registryUrl: 'http://registry.bit-bots.de:5000' }
        steps {
            sh './scripts/build-doc.py --meta -v'
            stash name: 'html', includes: 'doc/_build/**'
        }
        }

        stage('Deploy') {
        agent {
            label 'webserver'
        }
        steps {
            unstash 'html'
			sh 'rsync --delete -v -r ./doc/_build/ /srv/data/doku.bit-bots.de/meta/'
        }
        }
    }
}
