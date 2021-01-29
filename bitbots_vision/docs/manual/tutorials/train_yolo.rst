=================
How to train YOLO
=================

We currently use YOLO as our neural network for ball and goalpost detection.
We are considering to use more classes e.g. robots, but you should be mostly fine to just follow this tutorial even after those changes.
Our images and annotations are from the `ImageTagger <https://imagetagger.bit-bots.de>`_.

After following this tutorial you should have a trained YOLO network that is able to detect balls and goalposts at a decent framerate in the RoboCup Soccer context.

Select image sets
-----------------

Since image sets and annotations are constantly added, it is hard to give an up to date list of which image sets make sense to use for your training.
An example of image sets that you could use for training of balls and goalposts is the following list:

``6 160 162 184 185 186 188 189 191 607 608 609 610 611 612 613 614 615``

To find these image sets you can replace [id] in the following link with the id you want to open:

``https://imagetagger.bit-bots.de/images/imageset/[id]/``

If you want to select your own image sets it is probably advisable to look for image sets which have all your classes annotated.
Otherwise if your neural network detects a ball accurately but the ball is not labeled, the training would try to get the neural network to not detect the ball if it sees the same image again.

Download Images
---------------

You should download the images to the workstation you intend to train your YOLO on instead of your local computer.
If you are a BitBot this is probably the CL04 or CL05.
After you now know which image sets you want to use, it is time to download the images of these sets.
You can just download them using the provided `script
<https://imagetagger.bit-bots.de/images/imageset/imagetagger_dl_script.py>`_.
If you execute it with
``./imagetagger_dl_script.py id1 id2 id3``
you can download all of your image sets with one command.
You have to replace the ``ids`` with your corresponding id and can use as many or as few image sets as you want for this command.

Download Annotations
--------------------

On each image set page on the right you can select an export format to which you want to export your data.
For our approach we use ``Bit-Bots/goalpostBallToYaml``.
Now you have to go to each image set you selected and select the format and press ``export``.
After a short loading time your new export will show up on the right.
You should download this and preferably name it <id>.yaml
These yamls then have to be transferred to the work station and put into the folder as the image set with the same id.
You can then use the `createYoloLabels.py
<https://github.com/bit-bots/AutoImageLabeler/blob/master/yolo/createYoloLabels.py>`_
script provided in the `AutoImageLabeler
<https://github.com/bit-bots/AutoImageLabeler>`_
repository.

Train YOLO
----------

The ``createYoloLabels.py`` script creates a ``train.txt`` which you will have to use later.
Now we have prepared everything and can now start training the YOLO.
For these steps you can just follow the guide provided in the `official repository
<https://github.com/AlexeyAB/darknet#how-to-train-tiny-yolo-to-detect-your-custom-objects>`_.
In the end you will find a trained yolo in the backup folder of your darknet directory.
If you want to use the OpenVINO toolkit to execute the YOLO on the Intel Neural Compute Stick2 like we do, you should have a look at
:doc:`OpenVINO <../yolo_open_vino.rst>`
Congrats, you have successfully trained a YOLO for the RoboCup Soccer context.

Troubleshooting
---------------

If there is an error when you start the training, you might have to specify you want to use CUDA-10.
This can be done with: ``export PATH=/usr/local/cuda-10.0/bin:$PATH``

If you want to use multiple GPUs, you can use ``-gpus 0,1`` to use both gpus or e.g. ``-gpus 1`` if you only want to use the gpu with ID 1.