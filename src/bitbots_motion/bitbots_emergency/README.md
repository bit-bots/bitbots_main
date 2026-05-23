# bitbots emergency

This package contains logic for an emergency button that shuts off a robot's motors. It is divided into two parts: the publisher and the subscriber.

The publisher should be started on your local machine. After 3 seconds, it begins sending a continuous stream of messages via Zenoh to the subscriber as long as the emergency button is pressed.

The subscriber should be started on the robot with a Zenoh node. It waits for the first message from the publisher. As long as new messages continue to arrive, it does nothing, but when messages stop arriving, it shuts off the motors.

