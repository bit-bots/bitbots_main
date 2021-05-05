FROM docker.io/jenkins/jenkins

# these are the UID and GIDs of jenkins_master on rzrobo
USER root
ARG UID=997
ARG GID=994
RUN groupmod -g $GID jenkins
RUN usermod -u $UID -g $GID jenkins
RUN chown -R $UID:$GID /var/jenkins_home /usr/share/jenkins
USER jenkins

ENV PLUGINS="\
    antisamy-markup-formatter \
    blueocean \
    build-timeout \
    cloudbees-folder \
    configuration-as-code \
    credentials-binding \
    email-ext \
    embeddable-build-status \
    git \
    git-parameter \
    github \
    github-branch-source \
    junit \
    kubernetes \
    ldap \
    matrix-project \
    pipeline-githubnotify-step \
    pipeline-gitstatuswrapper \
    pipeline-stage-view \
    rebuild \
    timestamper \
    workflow-aggregator \
    ws-cleanup \
"

ENV PLUGIN_UPDATES="\
    trilead-api:latest \
    bouncycastle-api:latest"

RUN install-plugins.sh $PLUGINS
RUN install-plugins.sh $PLUGIN_UPDATES
