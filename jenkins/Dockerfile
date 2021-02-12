FROM docker.io/jenkins/jenkins

ENV PLUGINS="\
    antisamy-markup-formatter \
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
