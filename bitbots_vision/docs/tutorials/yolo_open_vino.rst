=========
OpenVINO
=========

Convert Darknet to OpenVINO models
==================================

Install OpenVINO
----------------
You can skip this step, if you have OpenVINO installed or access to a robot.

Otherwise visit the `OpenVINO Install Page <https://docs.openvinotoolkit.org/latest/_docs_install_guides_installing_openvino_linux.html>`_ and follow the instructions.


.. Tip:: If you use some other sytem like "Arch Linux" there is also a OpenVINO Docker container provided by Intel.

Convert weights to Tensorflow format
------------------------------------
1. Clone `GibHub:tensorflow-yolo-v3 <https://github.com/mystic123/tensorflow-yolo-v3>`_ to your PC (no OpenVINO needed).
2. Go to commit `ed60b90` if you have Problems.
3. Run the command

`python3 convert_weights_pb.py --class_names names.names --data_format NHWC --weights_file <PATH_TO_YOUR_CURRENT_YOLO_WEIGHTS_FILE> --tiny`

   - The required `names.names` file consists of all class names, with each one in its own line.
   - The `<PATH_TO_YOUR_CURRENT_YOLO_WEIGHTS_FILE>` represents the Darknet `.weights` file.
   - The `--tiny` setting stands for `tiny-yolo` which is the currently used architecture.
   - The output file will be written in the current folder as a `frozen_darknet_yolov3_model.pb` names file.

Convert Tensorflow format to OpenVINO
-------------------------------------
1. Source OpenVINO in your shell. Run `source /opt/intel/openvino/bin/setupvars.sh`.
2. Create an config like the `Example json config` below.
3. Run

`sudo python3 /opt/intel/openvino/deployment_tools/model_optimizer/mo_tf.py --input_model <PATH_TO_YOUR_FROZEN_TENSORFLOW_MODEL> --output_dir <YOUR_OUTPUT_DIR> --data_type FP16 --batch 1 --tensorflow_use_custom_operations_config <PATH_TO_YOUR_CONFIG_JSON>`

   - The tensorflow model path is set in the `<PATH_TO_YOUR_FROZEN_TENSORFLOW_MODEL>` part.
   - The model output path is saved in `<YOUR_OUTPUT_DIR>`.
   - The precision is set by replacing `FP16` with e.g. `FP32`.
   - Set the path to your freshly created `.json` config file (see *Example `.json` config* ) in `<PATH_TO_YOUR_CONFIG_JSON>`.

Example `.json` config
----------------------

In Darkent these values are located in the `.cfg` files.

.. code-block:: json

    [
    {
        "id": "TFYOLOV3",
        "match_kind": "general",
        "custom_attributes": {
        "classes": 2,
        "anchors": [17, 27, 8,140, 16,124, 37, 56, 18,211, 32,218, 83,105, 66,288, 142,185],
        "coords": 4,
        "num": 9,
        "masks": [[3, 4, 5], [0, 1, 2]],
        "entry_points": ["detector/yolo-v3-tiny/Reshape", "detector/yolo-v3-tiny/Reshape_4"]
        }
    }
    ]
