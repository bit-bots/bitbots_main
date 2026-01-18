# Login

First, you have to get the docker password for AWS. Installation instructions for the CLI can be
found [here](https://docs.aws.amazon.com/cli/latest/userguide/getting-started-install.html).
Then, you have to configure your account data using `aws configure`. The credentials are provided
by the TC.

After that, you can use a command similar to the following to log into the registry:
```
aws ecr get-login-password --region us-east-1 | docker login --username AWS --password-stdin <url>
```
The `<url>` is provided by the TC and looks like `123456789.dkr.ecr.us-east-1.amazonaws.com/cbr2021-bit-bots`.

# Build
`<url>` is the image url provided by the TC.

```
docker build -t <url> .
```

If you are in the lab network, you can speed up the download of apt packages by adding `--add-host apt-proxy:172.20.0.1`.

# Push
```
docker push <url>
```

# Run
Example:
```
docker run --gpus all --name bot_red_1 -d -e ROS_DOMAIN_ID 21 -e ROBOCUP_TEAM_COLOR=red -e ROBOCUP_ROBOT_ID=1 -e ROBOCUP_SIMULATOR_ADDR=127.0.0.1:10001 -e ROBOCUP_MIRROR_SERVER_IP=127.0.0.1 --net=host -v /tmp/logs:/robocup-logs <url> start offense 1 --team 12
```
The logs will be saved to `/tmp/logs`.
