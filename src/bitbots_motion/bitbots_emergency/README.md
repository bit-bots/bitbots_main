# bitbots_emergency

This package contains logic for an fail safe emergency button that shuts off a robot's motors. If the continous signal from is interrupted (lost remote connection, release of emergency button), the motors power is shut off.

It is divided into two parts: the publisher and the subscriber.

- The publisher should be started on your local machine. After 3 seconds, it begins sending a continuous stream of messages via Zenoh to the subscriber as long as the emergency button (space) is pressed.

- The subscriber node should be started on the robot with Zenoh DDS. It waits for the first message from the publisher. As long as new messages continue to arrive, it does nothing, but when messages stop arriving, it shuts off the motors.

