pipeline {
    agent none

    stages {
        stage('Build documentation') {
        agent { docker image: 'bitbots_builder', registryUrl: 'http://localhost:5000' }
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
            sh 'rm -rf /data/doku.bit-bots.de/*'
            sh 'cp -r ./doc/_build/* /data/doku.bit-bots.de'
        }
        }
    }
}