======
AI
======

Setup
-----
*  Explained in the Repo ReadMe
* Environment Variables:
    * ROBO_AI_DATA -> absolute path to dataset root
    * ROBO_AI_CODE -> absolute path to repository
    * ROBO_AI_LOG  -> unified log path (choose anything)

Repo-Structure
--------------
* benchmark
    * all benchmark related scripts are found here
* blur_autoencoder
    * deprecated (will be removed)
* datasets
    * static classes containing all image dataset information
    * do not use hardcoded paths - always use dataset classes
* detection
    * all scripts related to object detection
    * currently consists of:
        * bachelor thesis (Speck2016) **deprecated**
        * classifier approach **deprecated**
        * FCNN (Speck2018)
* distributions
    * scripts for probability distributions (-> noise)
* helper
    * all sorts of helper classes
    * classes
        * cluster2d_fcnn_output-> helper to find clusters in fcnn output
        * tf_helper -> various static methods for Tensorflow
            * will be split up in different helper classes
* legacy
    * deprecated implementations
    * saved for possible future reference
* model
    * deprecated
* notebooks
    * jupyter notebooks; mainly used for teaching
* robocup_image_reader
    * reader class for robocup image datasets (e.g. from Imagetagger)
* test_implementations
    * all new network models
    * used to try out new concepts
    * stored here until usable on robot

WS16 - Titan X Workstation
--------------------------
Please do not reboot / turn off / log off

experiment may be running

* usage instructions
    * save environments, libs, ... to /srv/ssd
    * save image sets to /srv/ssd_nvm
    * save logs, results, ... to /srv/hdd
    * do **not** save data in other locations


Ball Detection
--------------
* State of the art: FCNN03
* trainer.py is used for training
* predict.py to generate plot of prediction output


How-to: Training FCNN
    ``python trainer.py --save_path $ROBO_AI_LOG/my_experiment --model FCNN03 --epochs 20 --batch_size 16 --mode train --trainsets leipzig --clean 0``
